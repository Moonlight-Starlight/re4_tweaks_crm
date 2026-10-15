#pragma once
// Pure math for the "CameraRelativeMovement" option. No game or Windows dependencies, so it can be unit-tested on its own.
//
// Conventions (from the game's own walk code and the SmoothAnalogTurning code in ControllerTweaks.cpp):
//  - Yaw is in radians, and pushing the stick RIGHT decreases the player's ang_A0.y (left turn = +yaw).
//  - Stick: x positive = right, y positive = up/forward, each in [-1, 1].
//  - dt is the game's deltaTime_70, where 1.0 is one 30 FPS frame.
//
// Two camera modes share the same steering code:
//
//  CHASE: RE4's camera chases Leon, so the camera swings after him as he turns. The stick is read relative to the LIVE
//  camera every frame and Leon steers toward that heading at a speed proportional to how far off he is. Holding the stick
//  a little to the right keeps rotating Leon (and the camera behind him) to the right; letting it go back to forward stops
//  the rotation and Leon walks straight.
//   - The over-the-shoulder camera sits a few degrees off Leon's facing (~0.17 rad, measured from a log). That much is
//     subtracted from the camera yaw, otherwise stick-forward would slowly curve him. On a fixed camera angle this means
//     stick-forward ends up within that same ~10 degrees of the camera's axis, which is the price of not special-casing it.
//   - Pushing the stick backwards turns Leon around to face the camera. As the camera then swings behind him, a live
//     reference would flip him around again forever, so while the stick stays in the rear zone the reference is frozen.
//
//  DETACHED: the camera does not follow Leon. The right stick orbits it, and the stick direction is read relative to that
//  camera yaw. Because the camera stays put, there is no feedback loop and no rear-zone special case. The camera is held
//  still by overriding the chase camera's offset every frame (see PinnedCameraOffsetUnit below).

#include <algorithm>
#include <cmath>

namespace camrel
{
	constexpr float kPi = 3.14159265358979323846f;
	constexpr float kTwoPi = 2.0f * kPi;

	// Defaults (also the defaults of the user-facing settings)
	constexpr float kDefaultTurnGain = 0.10f;
	constexpr float kDefaultMaxTurnRate = 0.20f;
	constexpr float kDefaultOrbitSpeed = 0.07f;
	constexpr float kShoulderOffset = 0.175f; // camera yaw minus Leon's yaw with a settled chase camera (measured 0.167-0.184 rad)

	struct Config
	{
		float deadzone = 0.25f;         // stick magnitude below which the stick counts as neutral
		float turnGain = kDefaultTurnGain;       // fraction of the remaining heading error closed per 30 FPS frame (higher = snappier steering)
		float maxTurnRate = kDefaultMaxTurnRate; // cap on radians turned per 30 FPS frame (vanilla walk-turn is ~0.042)
		float facingThreshold = 1.22f;  // only move forward once within this many radians (~70 deg) of the target heading
		float shoulderOffset = kShoulderOffset;
		float rearEnter = 2.0f;         // stick angle from straight-up beyond which the reference is frozen (~115 deg)
		float rearExit = 1.8f;          // ...and unfrozen again (~103 deg)
	};

	struct State
	{
		bool rearFrozen = false;
		float frozenRef = 0.0f;
	};

	struct Output
	{
		bool active = false;   // false = stick is neutral, leave the game's own controls alone
		float newYaw = 0.0f;   // player yaw to write back (only meaningful if active)
		float targetYaw = 0.0f;
		bool forward = false;  // game should treat this frame as "walk forward"
		float magnitude = 0.0f;
	};

	// Wrap an angle into (-pi, pi]
	inline float WrapAngle(float a)
	{
		a = std::fmod(a, kTwoPi);
		if (a > kPi)
			a -= kTwoPi;
		else if (a <= -kPi)
			a += kTwoPi;
		return a;
	}

	inline void Reset(State& s)
	{
		s = State();
	}

	// Config for DETACHED mode: the camera yaw handed to Update() is the pinned camera's, which has no shoulder offset to
	// remove, and nothing ever swings behind Leon, so the rear-zone freeze is switched off.
	inline Config DetachedConfig(Config c)
	{
		c.shoulderOffset = 0.0f;
		c.rearEnter = 10.0f;
		c.rearExit = 10.0f;
		return c;
	}

	inline Output Update(State& s, float stickX, float stickY, float cameraYaw, float playerYaw, float dt, const Config& cfg = Config())
	{
		Output out;
		out.newYaw = playerYaw;

		float mag = std::sqrt(stickX * stickX + stickY * stickY);
		out.magnitude = mag;

		if (mag < cfg.deadzone)
		{
			s.rearFrozen = false;
			return out;
		}

		// 0 = straight up, positive = clockwise (towards the right)
		float stickAngle = std::atan2(stickX, stickY);

		// Live reference: the camera yaw, minus the shoulder offset (so a settled chase camera gives exactly "Leon's facing")
		float liveRef = WrapAngle(cameraYaw - cfg.shoulderOffset);

		float absAngle = std::fabs(stickAngle);
		if (!s.rearFrozen && absAngle > cfg.rearEnter)
		{
			s.rearFrozen = true;
			s.frozenRef = liveRef;
		}
		else if (s.rearFrozen && absAngle < cfg.rearExit)
		{
			s.rearFrozen = false;
		}

		float ref = s.rearFrozen ? s.frozenRef : liveRef;

		// Right is a decrease in yaw, hence the minus
		float target = WrapAngle(ref - stickAngle);
		float err = WrapAngle(target - playerYaw);

		// Proportional steering, framerate independent, with a cap
		float k = 1.0f - std::pow(1.0f - cfg.turnGain, dt);
		float maxStep = cfg.maxTurnRate * dt;
		float step = std::max(-maxStep, std::min(maxStep, err * k));

		out.active = true;
		out.targetYaw = target;
		out.newYaw = WrapAngle(playerYaw + step);
		out.forward = std::fabs(err - step) < cfg.facingThreshold; // error remaining after this frame's turn

		return out;
	}

	// ---- DETACHED camera ----

	// From CameraQuasiFPS::calcDepressionRatio in the game (read from the IDA export), for a right-stick value v:
	//     direction_ratio = (-v / C_RANGE_1097) * 1.0471976      (radians, applied as a rotation about Y relative to Leon's base matrix)
	// re4_tweaks' CameraImprovements sets C_RANGE_1097 = 42.3 and feeds v = fMousePosX * 127, so one unit of fMousePosX is:
	constexpr float kRatioPerUnit = (127.0f / 42.3f) * 1.0471976f; // ~3.144 rad, i.e. a unit of +-1 spans +-pi

	// The camera ends up at (Leon's base yaw + shoulder offset + direction_ratio). To hold it at a chosen world yaw, the offset has to be
	//     direction_ratio = cameraTarget - (playerYaw + shoulderOffset)
	// This returns the matching fMousePosX-style unit in [-1, 1] (0 is never returned, because exactly 0 would trigger the game's
	// "stick idle" branch, which only decays the old offset instead of replacing it).
	inline float PinnedCameraOffsetUnit(float cameraTarget, float playerYaw, float shoulderOffset = kShoulderOffset)
	{
		float ratio = WrapAngle(cameraTarget - (playerYaw + shoulderOffset));
		float u = -ratio / kRatioPerUnit;
		u = std::max(-1.0f, std::min(1.0f, u));
		if (u == 0.0f)
			u = 1e-6f;
		return u;
	}

	// The offset the game applies for a given unit (inverse of the above, used by tests)
	inline float AppliedDirectionRatio(float unit)
	{
		return -unit * kRatioPerUnit;
	}

	// Right stick -> camera orbit. rx: right stick X in [-1, 1] (positive = right). Small deflections are damped for fine control.
	// Pushing right turns the camera to the right, which is a decrease in yaw.
	inline float OrbitStep(float rx, float speed, float dt, float deadzone = 0.20f)
	{
		float a = std::fabs(rx);
		if (a <= deadzone)
			return 0.0f;
		float v = (a - deadzone) / (1.0f - deadzone);
		v = std::min(v, 1.0f);
		v *= v;
		float dir = rx > 0.0f ? -1.0f : 1.0f;
		return dir * v * speed * dt;
	}
}
