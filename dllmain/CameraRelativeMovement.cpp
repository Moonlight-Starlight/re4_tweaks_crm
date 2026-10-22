//#define CAMERARELATIVEMOVEMENT_DEBUG // dinput8.log tracing. Uncomment to enable.

#include <chrono>
#include <cmath>
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
//           asks CameraRelativeMovement_GetPinnedCameraOffset to keep the camera at its world yaw. Pushing the left stick past the
//           run radius also sets the game's KEY_RUN (CameraRelativeMovement_PressRun), whatever button is mapped to run.
//
// Raw pad layout, relative to the esi the hook sees: -0x48 u16 buttons, -0x46 u8 left trigger, -0x44 s16 LX, -0x42 s16 LY, -0x40 s16 RX.
// Only Xinput pads are supported.
//
// CameraControl::r0_34 is 0xA for the chase camera and 0x11 for the aim camera (seen in game). Unverified: the Flags_STATUS gating below
// only has the names to go by.

static camrel::State g_state;
static bool g_wasDetached = false;
static bool g_wasAiming = false;
static float g_camTarget = 0.0f; // Detached: world yaw the camera is pinned to
static bool g_pinActive = false;
static std::chrono::steady_clock::time_point g_pinStamp;

static bool g_running = false; // Detached: the left stick is past the run radius
static std::chrono::steady_clock::time_point g_runStamp;
static std::chrono::steady_clock::time_point g_lastAim;

// Camera yaw minus base matrix yaw while the base is rotated to hold the pin. Measured at 0.178 in game and constant: the shoulder
// offset plus the tiny right stick value sent below. It has to stay a constant: the pad (ProcessStick) and the camera run on
// different threads, so correcting it from the measured camera yaw made the camera ring.
static const float kPinnedCamOffset = camrel::kShoulderOffset + 0.003f;
static int g_baseOff = 0; // where the base matrix sits in CameraQuasiFPS::move's stack frame, in bytes below its ebp (0 = not found yet)

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

	// Pushing a statue, crate, ...: Leon stays locked to the object and the game ignores the left stick, so leave it alone.
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
		g_pinActive = g_wasDetached = g_wasAiming = g_running = false;
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
		{
			// Pin the camera where it is, so switching on never jumps it. Right after aiming the camera is rebuilt behind Leon's
			// shoulder, and the game's own camera can glitch while it switches back (a 180 when swapping to the knife just before
			// scoping), so pin to Leon's facing instead of the live camera.
			bool justAimed = std::chrono::steady_clock::now() - g_lastAim < std::chrono::milliseconds(1500);
			g_camTarget = justAimed ? camrel::WrapAngle(player->ang_A0.y + camrel::kShoulderOffset) : liveCamYaw;
		}
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

	// Aiming keeps the game's own left stick handling
	if (aiming)
	{
		g_state = camrel::State();
		g_running = false;
		g_lastAim = std::chrono::steady_clock::now();
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

	// Detached: a full push runs (the run button still works too), only while walking forward
	[[maybe_unused]] bool wasRunning = g_running;
	g_running = detached && out.forward && camrel::RunByTilt(g_running, stickX, stickY, re4t::cfg->fCameraRelativeRunRadius);
	if (g_running)
		g_runStamp = std::chrono::steady_clock::now();

#ifdef CAMERARELATIVEMOVEMENT_DEBUG
	{
		static int counter = 0;
		if ((counter++ % 30) == 0)
			spd::log()->info("{} -> [{}] stick({:.2f}, {:.2f}) tilt {:.2f} camTarget {:.3f} camYaw {:.3f} plYaw {:.3f} fwd {} back {} run {}", __FUNCTION__,
				detached ? "detached" : "chase", stickX, stickY, sqrtf(stickX * stickX + stickY * stickY), g_camTarget, liveCamYaw, player->ang_A0.y, out.forward, out.backward, g_running);
		if (g_running != wasRunning)
			spd::log()->info("{} -> run {} at tilt {:.2f} (radius {:.2f}), KEY_RUN {}", __FUNCTION__, g_running ? "on" : "off",
				sqrtf(stickX * stickX + stickY * stickY), re4t::cfg->fCameraRelativeRunRadius, (Key_btn_on() & (uint64_t)KEY_BTN::KEY_RUN) != 0);
	}
#endif

	// The game never sees the real left stick
	*rawLX = 0;
	*rawLY = out.forward ? 32767 : (out.backward ? -32767 : 0);

	if (out.active)
		player->ang_A0.y = out.newYaw;
}

// Called from ControllerTweaks.cpp's hook near the end of PadRead, once the game has built this frame's key state. While Detached and past
// the run radius, sets KEY_RUN in the game's key words, as the game itself does for its analog camera keys. This follows the player's
// layout, and unlike a pad button it doesn't press the other keys that button has (A is also KEY_B and KEY_OK).
void CameraRelativeMovement_PressRun(uint32_t* const (&keyWords)[4])
{
	// ProcessStick runs every frame while it is in charge. If it stopped (option off, pad unplugged), stop pressing.
	if (!g_running || std::chrono::steady_clock::now() - g_runStamp > std::chrono::milliseconds(100))
		return;

	for (uint32_t* word : keyWords)
		*word |= (uint32_t)KEY_BTN::KEY_RUN;
}

// Detached pinning: CameraQuasiFPS::move builds its base matrix into a stack local (calcBaseMatrix(this, m)), copied from m_pl_mat_178 and then maybe
// tilted to the floor. The offset path only ever sees the camera object, so find that local through the stack: our hook runs inside
// calcDepressionRatio, whose saved ebp is move()'s frame. The IDA frame puts m at ebp-0x34 but nothing here relies on that: scan move's
// frame for a 3x4 float matrix with orthonormal columns that sits where m_pl_mat_178 does and faces about the same way.
static bool LooksLikeBase(const float* m, const float* pl)
{
	for (int i = 0; i < 12; i++)
		if (!std::isfinite(m[i]) || fabsf(m[i]) > 1.0e7f)
			return false;

	for (int c = 0; c < 3; c++)
	{
		float len = m[c] * m[c] + m[4 + c] * m[4 + c] + m[8 + c] * m[8 + c];
		if (fabsf(len - 1.0f) > 0.02f)
			return false;
	}

	float dx = m[3] - pl[3], dz = m[11] - pl[11];
	if (dx * dx + dz * dz > 150.0f * 150.0f)
		return false;

	return fabsf(camrel::WrapAngle(camrel::MatrixYaw(m[2], m[10]) - camrel::MatrixYaw(pl[2], pl[10]))) < 0.6f;
}

static float* FindBaseMatrix(uintptr_t calcEbp, const CameraQuasiFPS& cam)
{
	uintptr_t moveEbp = *(uintptr_t*)calcEbp;
	if (moveEbp <= calcEbp || moveEbp - calcEbp > 0x1000 || (moveEbp & 3))
		return nullptr;

	const float* pl = &cam.m_pl_mat_178[0][0];

	// Known place first
	if (g_baseOff != 0)
	{
		float* m = (float*)(moveEbp - g_baseOff);
		if (LooksLikeBase(m, pl))
			return m;
		g_baseOff = 0;
	}

	float* found = nullptr;
	int foundOff = 0, count = 0;
	for (int off = 0x1A0; off >= 0x30; off -= 4)
	{
		float* m = (float*)(moveEbp - off);
		if (LooksLikeBase(m, pl))
		{
			count++;
			if (!found || off == 0x34)
				found = m, foundOff = off;
#ifdef CAMERARELATIVEMOVEMENT_DEBUG
			spd::log()->info("{} -> base matrix candidate at move ebp-0x{:X}: pos({:.1f},{:.1f},{:.1f}) yaw {:.3f}", __FUNCTION__, off, m[3], m[7], m[11], camrel::MatrixYaw(m[2], m[10]));
#endif
		}
	}

#ifdef CAMERARELATIVEMOVEMENT_DEBUG
	spd::log()->info("{} -> frame scan: calc ebp {:#x}, move ebp {:#x}, return address {:#x}, {} candidate(s), using ebp-0x{:X}", __FUNCTION__,
		calcEbp, moveEbp, *(uintptr_t*)(moveEbp + 4), count, foundOff);
#endif

	// Several candidates and none at the expected place: don't guess
	if (count > 1 && foundOff != 0x34)
		return nullptr;

	g_baseOff = foundOff;
	return found;
}

// Called from the camera hook with calcDepressionRatio's ebp. While Detached mode pins the camera, returns true and the right stick unit
// to feed the game.
bool CameraRelativeMovement_GetPinnedCameraOffset(float* outUnit, uintptr_t calcEbp)
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

	CameraQuasiFPS& cam = CamCtrl->m_QuasiFPS_278;

	// Preferred: turn the camera's base matrix so that base yaw + kPinnedCamOffset lands on the pin. The camera's own sideways offset
	// then stays constant, so its low-pass filter, smoother and weapon swap blend keep working normally.
	static bool warned = false;
	if (float* m = FindBaseMatrix(calcEbp, cam))
	{
		float baseYaw = camrel::MatrixYaw(m[2], m[10]);
		float delta = camrel::WrapAngle(g_camTarget - baseYaw - kPinnedCamOffset);
		float c = cosf(delta), s = sinf(delta);
		for (int col = 0; col < 3; col++)
		{
			float x = m[col], z = m[8 + col];
			m[col] = x * c + z * s;
			m[8 + col] = -x * s + z * c;
		}

		// A constant, never exactly zero right stick value keeps the sideways offset steady (zero makes the game ease it back to 0)
		*outUnit = 0.002f;

#ifdef CAMERARELATIVEMOVEMENT_DEBUG
		static int counter = 0;
		if ((counter++ % 10) == 0)
			spd::log()->info("{} -> [base] camTarget {:.3f} liveCamYaw {:.3f} baseYaw {:.3f} -> rotated by {:.3f} plYaw {:.3f} residual {:.3f}", __FUNCTION__,
				g_camTarget, GetLiveCameraYaw(), baseYaw, delta, camrel::MatrixYaw(cam.m_pl_mat_178[0][2], cam.m_pl_mat_178[2][2]), camrel::WrapAngle(GetLiveCameraYaw() - g_camTarget));
#endif
		return true;
	}

	if (!warned)
	{
		warned = true;
		spd::log()->warn("{} -> base matrix not found in the camera's stack frame, offsetting the camera instead", __FUNCTION__);
	}

	// Fallback: offset the camera against its player matrix. CameraQuasiFPS::move low-pass filters the camera's sideways offset (~9 frames),
	// which drags the camera along whenever the offset changes quickly, as it does while Leon turns under a pinned camera. The filter and
	// the camera smoother are skipped on the camera's first frame, so claim every frame is a first frame (move() clears the flag itself).
	// The price is the smoothing and the weapon swap blend.
	*outUnit = camrel::PinnedCameraOffsetUnit(g_camTarget, camrel::MatrixYaw(cam.m_pl_mat_178[0][2], cam.m_pl_mat_178[2][2]));
	cam.m_init_flag_1B6 = 1;
	return true;
}
