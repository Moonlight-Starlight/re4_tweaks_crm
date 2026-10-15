#define MODERNMOVE_DEBUG // debug logging to dinput8.log. REMOVE THIS LINE for the final build.

#include <iostream>
#include <chrono>
#include "dllmain.h"
#include "Settings.h"
#include "Patches.h"
#include "Game.h"
#include "GameFlags.h"
#include "CameraRelativeMovement.h"

// "CameraRelativeMovement": moves Leon in the direction the left stick points, relative to the camera, instead of tank controls.
//
// How it works:
//  ControllerTweaks.cpp's StoreRawAnalog hook runs inside PadXinputRead right before the game turns the raw left-stick words
//  into its deadzoned floats and KEY_* bits. It calls CameraRelativeMovement_ProcessStick from there, which does two things:
//    1. rotates the player (ang_A0.y) toward the heading the stick asks for, using the same direct-yaw-write approach as SmoothAnalogTurning
//    2. rewrites the raw left-stick words to a synthetic "straight forward" push (or neutral), so the game's own state machine
//       starts/stops walking and never tries to tank-turn.
//
// Modes (see CameraRelativeMovement.h for the maths):
//  Chase:    the stick is read relative to the live chase camera.
//  Detached: the right stick orbits the camera, which no longer follows Leon. CameraTweaks.cpp's calcDepressionRatioAnalogRX hook
//            asks CameraRelativeMovement_GetPinnedCameraOffset() for the offset that holds the camera at the chosen world yaw.
//
// Raw pad layout, relative to the esi the hook sees (the game's own XInput state, copied verbatim from XInputGetState):
//  -0x46 u8 left trigger, -0x45 u8 right trigger, -0x44 s16 LX, -0x42 s16 LY, -0x40 s16 RX, -0x3E s16 RY
//
// UNVERIFIED ASSUMPTIONS, check them in-game with MODERNMOVE_DEBUG defined:
//  - the Flags_STATUS entries used for gating mean what their names suggest
//  - the camera type value 0xA in CameraControl::r0_34 is the over-the-shoulder chase camera (CameraQuasiFPS)
//  - Detached: the camera base follows Leon's ang_A0.y without a frame of lag (a lag would show as camTarget/camYaw not matching in the log)

static camrel::State g_state;
static bool g_lastDetached = false;

// Detached camera
static bool g_pinActive = false;
static std::chrono::steady_clock::time_point g_pinStamp;
static bool g_pinInit = false;
static bool g_wasAiming = false;
static float g_camTarget = 0.0f;

struct GateFlag
{
	Flags_STATUS flag;
	const char* name;
};

// Anything here means "not free-roaming gameplay". The game's own PadRead treats STA_SUB_SCRN, STA_TITLE and STA_TYPEWRITER as "UI active".
static const GateFlag g_blockingFlags[] = {
	{ Flags_STATUS::STA_TITLE, "STA_TITLE" },
	{ Flags_STATUS::STA_TYPEWRITER, "STA_TYPEWRITER" },
	{ Flags_STATUS::STA_SUB_SCRN, "STA_SUB_SCRN" },
	{ Flags_STATUS::STA_DIEDEMO, "STA_DIEDEMO" },
	{ Flags_STATUS::STA_ITEM_GET, "STA_ITEM_GET" },
	{ Flags_STATUS::STA_MOVIE_ON, "STA_MOVIE_ON" },
	{ Flags_STATUS::STA_MOVIE2_ON, "STA_MOVIE2_ON" },
	{ Flags_STATUS::STA_EVENT, "STA_EVENT" },
	{ Flags_STATUS::STA_LOOK_THROUGH, "STA_LOOK_THROUGH" },
	{ Flags_STATUS::STA_SCOPE_CAMERA, "STA_SCOPE_CAMERA" },
	{ Flags_STATUS::STA_PL_LADDER, "STA_PL_LADDER" },
	{ Flags_STATUS::STA_PL_SWIM, "STA_PL_SWIM" },
	{ Flags_STATUS::STA_PL_BOAT, "STA_PL_BOAT" },
	{ Flags_STATUS::STA_RIDE_GONDOLA, "STA_RIDE_GONDOLA" },
	{ Flags_STATUS::STA_PL_CATCHED, "STA_PL_CATCHED" },
	{ Flags_STATUS::STA_PL_CATCHHOLD, "STA_PL_CATCHHOLD" },
	{ Flags_STATUS::STA_PL_EM_ACTION, "STA_PL_EM_ACTION" },
	{ Flags_STATUS::STA_SUSPEND, "STA_SUSPEND" },
};

// Returns nullptr if the option may act right now, otherwise the reason it may not
static const char* GateReason()
{
	if (LastUsedDevice() != InputDevices::XinputController)
		return "not xinput";

	if (PlayerPtr() == nullptr)
		return "no player";

	if (OptionOpenFlag())
		return "options menu";

	for (const GateFlag& g : g_blockingFlags)
	{
		if (FlagIsSet(GlobalPtr()->flags_STATUS_0_501C, uint32_t(g.flag)))
			return g.name;
	}

	return nullptr;
}

#ifdef MODERNMOVE_DEBUG
static void LogGateChange(const char* reason)
{
	static const char* lastReason = reinterpret_cast<const char*>(1); // never equal to a real value, forces the first log
	if (reason != lastReason)
	{
		if (reason)
			spd::log()->info("{} -> inactive ({})", __FUNCTION__, reason);
		else
			spd::log()->info("{} -> ACTIVE", __FUNCTION__);
		lastReason = reason;
	}
}
#endif

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
	int16_t* rawRX = (int16_t*)(pad - 0x40);
	uint8_t leftTrigger = *(uint8_t*)(pad - 0x46);

	const char* reason = GateReason();

#ifdef MODERNMOVE_DEBUG
	LogGateChange(reason);
#endif

	if (reason)
	{
		camrel::Reset(g_state);
		g_pinActive = false;
		g_pinInit = false;
		g_wasAiming = false;
		return;
	}

	cPlayer* player = PlayerPtr();
	float dt = GlobalPtr()->deltaTime_70;
	float liveCamYaw = GetLiveCameraYaw();

	bool detached = (re4t::cfg->iCameraRelativeMode == CameraRelativeModes::CamRel_Detached) && IsChaseCamera();

	// Aim button, with the raw trigger as an early warning (KEY_KAMAE is only set later this frame)
	bool aiming = (leftTrigger > 30) || ((Key_btn_on() & ((uint64_t)KEY_BTN::KEY_KAMAE | (uint64_t)KEY_BTN::KEY_KAMAE_KNIFE)) != 0);

	if (detached != g_lastDetached)
	{
		camrel::Reset(g_state);
		g_lastDetached = detached;
	}

	if (detached)
	{
		if (!g_pinInit)
		{
			g_camTarget = liveCamYaw; // pin the camera exactly where it is, so switching on never jumps it
			g_pinInit = true;
		}

		if (aiming)
		{
			// The aim camera is built from Leon's facing, so face where the camera was looking before it takes over.
			if (!g_wasAiming)
				player->ang_A0.y = camrel::WrapAngle(g_camTarget - camrel::kShoulderOffset);

			// While aiming the game puts the camera at (Leon's facing + shoulder), so the pin follows it. That also means the
			// camera is where we left it when the aim button is released.
			g_camTarget = camrel::WrapAngle(player->ang_A0.y + camrel::kShoulderOffset);
		}
		else
		{
			g_camTarget = camrel::WrapAngle(g_camTarget + camrel::OrbitStep(*rawRX / 32767.0f, re4t::cfg->fCameraRelativeOrbitSpeed, dt));
		}

		g_wasAiming = aiming;
		g_pinActive = true;
		g_pinStamp = std::chrono::steady_clock::now();
	}
	else
	{
		g_pinActive = false;
		g_pinInit = false;
		g_wasAiming = false;
	}

	// Aiming (and the knife) keep the game's own stick handling
	if (aiming)
	{
		camrel::Reset(g_state);
		return;
	}

	float stickX = *rawLX / 32767.0f;
	float stickY = *rawLY / 32767.0f;
	float plYaw = player->ang_A0.y;

	camrel::Config cfg;
	cfg.turnGain = re4t::cfg->fCameraRelativeTurnGain;
	cfg.maxTurnRate = re4t::cfg->fCameraRelativeMaxTurnRate;

	camrel::Output out;
	if (detached)
		out = camrel::Update(g_state, stickX, stickY, g_camTarget, plYaw, dt, camrel::DetachedConfig(cfg));
	else
		out = camrel::Update(g_state, stickX, stickY, liveCamYaw, plYaw, dt, cfg);

#ifdef MODERNMOVE_DEBUG
	{
		static int counter = 0;
		if ((counter++ % 30) == 0)
		{
			if (detached)
				spd::log()->info("{} -> [detached] stick({:.2f}, {:.2f}) camTarget {:.3f} camYaw {:.3f} plYaw {:.3f} active {} fwd {}", __FUNCTION__, stickX, stickY, g_camTarget, liveCamYaw, plYaw, out.active, out.forward);
			else
				spd::log()->info("{} -> [chase] stick({:.2f}, {:.2f}) camYaw {:.3f} plYaw {:.3f} active {} fwd {}", __FUNCTION__, stickX, stickY, liveCamYaw, plYaw, out.active, out.forward);
		}
	}
#endif

	// The game must never see the real stick: neutral (or inside our deadzone) -> zero it, otherwise synthetic forward
	if (!out.active)
	{
		*rawLX = 0;
		*rawLY = 0;
		return;
	}

	player->ang_A0.y = out.newYaw;

	// PadRead turns the stick into a digital 4-way pad: after its own deadzone remap the stick must be pushed past roughly 60% of full tilt
	// before KEY_FORWARD is set, and tilt has no effect on speed beyond that. So the synthetic push is always full scale.
	*rawLX = 0;
	*rawLY = out.forward ? 32767 : 0;
}

// Called from the camera hook (CameraTweaks.cpp). Returns true, and the fMousePosX-style offset that holds the camera still, while Detached mode is pinning it.
bool CameraRelativeMovement_GetPinnedCameraOffset(float* outUnit)
{
	if (!g_pinActive)
		return false;

	// ProcessStick runs every frame while it is in charge. If it stopped (pad unplugged, hook not reached), don't keep pinning.
	if (std::chrono::steady_clock::now() - g_pinStamp > std::chrono::milliseconds(100))
	{
		g_pinActive = false;
		return false;
	}

	cPlayer* player = PlayerPtr();
	if (player == nullptr || !IsChaseCamera())
		return false;

	*outUnit = camrel::PinnedCameraOffsetUnit(g_camTarget, player->ang_A0.y);
	return true;
}
