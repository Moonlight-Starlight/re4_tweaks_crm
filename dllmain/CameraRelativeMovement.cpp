//#define CAMERARELATIVEMOVEMENT_DEBUG // dinput8.log tracing. Uncomment to enable.

#include <chrono>
#include "dllmain.h"
#include "Settings.h"
#include "Patches.h"
#include "Game.h"
#include "GameFlags.h"
#include "CameraRelativeMovement.h"

// Moves Leon where the left stick points, relative to the camera, instead of tank controls.
//
// ControllerTweaks.cpp's StoreRawAnalog hook runs inside PadXInputRead, before the game turns the raw stick words into KEY_* bits.
// It calls CameraRelativeMovement_ProcessStick, which turns Leon (ang_A0.y) toward the wanted heading and rewrites the left stick
// to a plain forward / back push (or neutral), so the game never tank-turns him. PadRead is a digital 4-way pad: past ~60% tilt a
// direction is on and tilt does not change speed, so the synthetic push is always full scale.
//
// Chase: the left stick is read relative to the live chase camera. Pushing the left stick down walks backwards.
// Detached: the right stick orbits the camera, which stays put when Leon turns. CameraTweaks.cpp's calcDepressionRatioAnalogRX hook
//           asks CameraRelativeMovement_GetPinnedCameraOffset for the offset that holds the camera at its world yaw.
//
// Raw pad layout, relative to the esi the hook sees: -0x46 u8 left trigger, -0x44 s16 LX, -0x42 s16 LY, -0x40 s16 RX.
// Only Xinput pads are supported.
//
// Unverified: the Flags_STATUS gating below only has the names to go by, and 0xA in CameraControl::r0_34 being the chase camera.

static camrel::State g_state;
static bool g_wasDetached = false;
static bool g_wasAiming = false;
static float g_camTarget = 0.0f; // Detached: world yaw the camera is pinned to
static bool g_pinActive = false;
static std::chrono::steady_clock::time_point g_pinStamp;

// Anything here means "not free-roaming gameplay". The game's own PadRead treats STA_SUB_SCRN, STA_TITLE and STA_TYPEWRITER as "UI active".
#define GATE(f) { Flags_STATUS::f, #f }
static const struct { Flags_STATUS flag; const char* name; } g_blockingFlags[] = {
	GATE(STA_TITLE), GATE(STA_TYPEWRITER), GATE(STA_SUB_SCRN), GATE(STA_DIEDEMO), GATE(STA_ITEM_GET), GATE(STA_MOVIE_ON),
	GATE(STA_MOVIE2_ON), GATE(STA_EVENT), GATE(STA_LOOK_THROUGH), GATE(STA_SCOPE_CAMERA), GATE(STA_PL_LADDER), GATE(STA_PL_SWIM),
	GATE(STA_PL_BOAT), GATE(STA_RIDE_GONDOLA), GATE(STA_PL_CATCHED), GATE(STA_PL_CATCHHOLD), GATE(STA_PL_EM_ACTION), GATE(STA_SUSPEND),
};

// nullptr if the option may act right now, otherwise the reason it may not
static const char* GateReason()
{
	if (LastUsedDevice() != InputDevices::XinputController)
		return "not xinput";

	cPlayer* player = PlayerPtr();
	if (player == nullptr)
		return "no player";

	// Scripted events (candelabra jump, ...) take Leon over. The game's own PlGetStatus reports them with this bit.
	if (player->stat_468 & 2)
		return "player event";

	// Pushing a statue, crate, ...: Leon stays locked to the object and the game ignores the stick, so leave it alone.
	// The push is action state 9 (r_no_1_FD), from its first frame until it ends, including when the object is stuck.
	if (player->r_no_0_FC == 0 && player->r_no_1_FD == 9)
		return "pushing object";

	if (OptionOpenFlag())
		return "options menu";

	for (const auto& g : g_blockingFlags)
	{
		if (FlagIsSet(GlobalPtr()->flags_STATUS_0_501C, uint32_t(g.flag)))
			return g.name;
	}

	return nullptr;
}

static float GetLiveCameraYaw()
{
	const CAMERA_POINT& cp = GlobalPtr()->Camera_74.CamPoint_A4;
	return atan2f(cp.Target_C.x - cp.Campos_0.x, cp.Target_C.z - cp.Campos_0.z);
}

static bool IsChaseCamera()
{
	return CamCtrl != nullptr && CamCtrl->r0_34 == 0xA; // CameraControl::Move case 0xA -> CameraQuasiFPS::move
}

void CameraRelativeMovement_ProcessStick(uintptr_t pad)
{
	int16_t* rawLX = (int16_t*)(pad - 0x44);
	int16_t* rawLY = (int16_t*)(pad - 0x42);
	int16_t rawRX = *(int16_t*)(pad - 0x40);
	uint8_t leftTrigger = *(uint8_t*)(pad - 0x46);

	const char* reason = GateReason();

#ifdef CAMERARELATIVEMOVEMENT_DEBUG
	{
		static const char* lastReason = reinterpret_cast<const char*>(1); // never a real value, forces the first log
		if (reason != lastReason)
		{
			spd::log()->info("{} -> {}{}", __FUNCTION__, reason ? "inactive: " : "ACTIVE", reason ? reason : "");
			lastReason = reason;
		}
	}
#endif

	if (reason)
	{
		g_state = camrel::State();
		g_pinActive = g_wasDetached = g_wasAiming = false;
		return;
	}

	cPlayer* player = PlayerPtr();
	float dt = GlobalPtr()->deltaTime_70;
	float liveCamYaw = GetLiveCameraYaw();
	bool detached = (re4t::cfg->iCameraRelativeMode == CameraRelativeModes::CamRel_Detached) && IsChaseCamera();

	// Aim button, with the raw trigger as an early warning (KEY_KAMAE is only set later this frame)
	bool aiming = (leftTrigger > 30) || ((Key_btn_on() & ((uint64_t)KEY_BTN::KEY_KAMAE | (uint64_t)KEY_BTN::KEY_KAMAE_KNIFE)) != 0);

	if (detached != g_wasDetached)
	{
		g_state = camrel::State();
		if (detached)
			g_camTarget = liveCamYaw; // pin the camera where it is, so switching on never jumps it
		g_wasDetached = detached;
	}

	if (detached)
	{
		if (aiming)
		{
			// The aim camera is built from Leon's facing: face where the camera was looking, then let the pin follow him
			if (!g_wasAiming)
				player->ang_A0.y = camrel::WrapAngle(g_camTarget - camrel::kShoulderOffset);

			g_camTarget = camrel::WrapAngle(player->ang_A0.y + camrel::kShoulderOffset);
		}
		else
		{
			g_camTarget = camrel::WrapAngle(g_camTarget + camrel::OrbitStep(rawRX / 32767.0f, re4t::cfg->fCameraRelativeOrbitSpeed, dt));
		}

		g_pinStamp = std::chrono::steady_clock::now();
	}

	g_wasAiming = detached && aiming;
	g_pinActive = detached;

	// Aiming (and the knife) keep the game's own stick handling
	if (aiming)
	{
		g_state = camrel::State();
		return;
	}

	camrel::Config cfg;
	cfg.turnGain = re4t::cfg->fCameraRelativeTurnGain;
	cfg.maxTurnRate = re4t::cfg->fCameraRelativeMaxTurnRate;
	cfg.driftLeft = re4t::cfg->fCameraRelativeDriftLeft;
	cfg.driftRight = re4t::cfg->fCameraRelativeDriftRight;

	float stickX = *rawLX / 32767.0f;
	float stickY = *rawLY / 32767.0f;
	camrel::Output out = detached ? camrel::Update(g_state, stickX, stickY, g_camTarget, player->ang_A0.y, dt, camrel::DetachedConfig(cfg))
	                              : camrel::Update(g_state, stickX, stickY, liveCamYaw, player->ang_A0.y, dt, cfg);

#ifdef CAMERARELATIVEMOVEMENT_DEBUG
	{
		static int counter = 0;
		if ((counter++ % 30) == 0)
			spd::log()->info("{} -> [{}] stick({:.2f}, {:.2f}) camTarget {:.3f} camYaw {:.3f} plYaw {:.3f} fwd {} back {}", __FUNCTION__,
				detached ? "detached" : "chase", stickX, stickY, g_camTarget, liveCamYaw, player->ang_A0.y, out.forward, out.backward);
	}
#endif

	// The game never sees the real stick
	*rawLX = 0;
	*rawLY = out.forward ? 32767 : (out.backward ? -32767 : 0);

	if (out.active)
		player->ang_A0.y = out.newYaw;
}

// Called from the camera hook. While Detached mode pins the camera, returns true and the right stick unit that holds it still.
bool CameraRelativeMovement_GetPinnedCameraOffset(float* outUnit)
{
	// ProcessStick runs every frame while it is in charge. If it stopped (pad unplugged, hook not reached), stop pinning.
	if (!g_pinActive || std::chrono::steady_clock::now() - g_pinStamp > std::chrono::milliseconds(100))
	{
		g_pinActive = false;
		return false;
	}

	cPlayer* player = PlayerPtr();
	if (player == nullptr || !IsChaseCamera())
		return false;

	// The camera builds itself on its own copy of the player matrix, so pin against that matrix's yaw
	CameraQuasiFPS& cam = CamCtrl->m_QuasiFPS_278;
	*outUnit = camrel::PinnedCameraOffsetUnit(g_camTarget, camrel::MatrixYaw(cam.m_pl_mat_178[0][2], cam.m_pl_mat_178[2][2]));

	// CameraQuasiFPS::move low-pass filters the camera's sideways offset (~9 frames), which drags the camera along whenever the offset
	// changes quickly, as it does while Leon turns under a pinned camera. The filter and the camera smoother are skipped on the
	// camera's first frame, so claim every frame is a first frame (move() clears the flag itself).
	cam.m_init_flag_1B6 = 1;
	return true;
}
