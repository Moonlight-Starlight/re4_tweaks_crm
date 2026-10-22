#pragma once
// Maths for CameraRelativeMovement. No game or Windows dependencies, so it can be tested on its own.
//
// Conventions: yaw in radians, and pushing the left stick right DECREASES the player's ang_A0.y.
// Left stick x is positive right, y positive up, both in [-1, 1]. dt is the game's deltaTime_70 (1.0 = one 30 FPS frame).

#include <algorithm>
#include <cmath>

namespace camrel
{
	constexpr float kPi = 3.14159265358979323846f;
	constexpr float kShoulderOffset = 0.175f; // camera yaw minus Leon's yaw with a settled chase camera (measured 0.167-0.184)

	struct Config
	{
		float deadzone = 0.25f;        // left stick magnitude below which it counts as neutral
		float turnGain = 0.10f;        // fraction of the remaining heading error closed per 30 FPS frame
		float maxTurnRate = 0.20f;     // radians turned per 30 FPS frame, at most
		float facingThreshold = 1.22f; // only walk once within this many radians of the target heading
		float shoulderOffset = kShoulderOffset;
		float driftLeft = 0.0f;        // sideways left stick deflection ignored to the left / right (stick drift)
		float driftRight = 0.0f;
		bool backpedal = true;         // Chase: the rear of the left stick walks backwards. Detached: all directions walk forward.
		float rearEnter = 2.0f;        // left stick angle from straight up where the rear zone starts (~115 deg)...
		float rearExit = 1.8f;         // ...and ends (~103 deg)
	};

	struct State
	{
		bool inRear = false;
	};

	struct Output
	{
		bool active = false;    // false: left stick is neutral
		float newYaw = 0.0f;    // player yaw to write back
		bool forward = false;   // report "left stick forward" to the game
		bool backward = false;  // report "left stick back" to the game
	};

	inline float WrapAngle(float a)
	{
		a = std::fmod(a, 2.0f * kPi);
		if (a > kPi)
			a -= 2.0f * kPi;
		else if (a <= -kPi)
			a += 2.0f * kPi;
		return a;
	}

	// Detached: the camera yaw passed to Update() is the pinned one, so there is no shoulder offset, and every direction walks forward.
	inline Config DetachedConfig(Config c)
	{
		c.shoulderOffset = 0.0f;
		c.backpedal = false;
		return c;
	}

	// Removes the first `drift` of the left stick's sideways deflection and rescales the rest to the full range.
	inline float RemoveDrift(float x, float driftLeft, float driftRight)
	{
		float drift = x < 0.0f ? driftLeft : driftRight;
		float a = std::fabs(x);
		if (a <= drift)
			return 0.0f;
		a = (a - drift) / (1.0f - drift);
		return x < 0.0f ? -a : a;
	}

	inline Output Update(State& s, float stickX, float stickY, float cameraYaw, float playerYaw, float dt, const Config& cfg = Config())
	{
		Output out;
		out.newYaw = playerYaw;

		if (std::sqrt(stickX * stickX + stickY * stickY) < cfg.deadzone)
		{
			s.inRear = false;
			return out;
		}

		float x = RemoveDrift(stickX, cfg.driftLeft, cfg.driftRight);
		float stickAngle = std::atan2(x, stickY); // 0 = up, positive = right, +-pi = down
		float absAngle = std::fabs(stickAngle);

		if (!cfg.backpedal)
			s.inRear = false;
		else if (!s.inRear && absAngle > cfg.rearEnter)
			s.inRear = true;
		else if (s.inRear && absAngle < cfg.rearExit)
			s.inRear = false;

		float k = 1.0f - std::pow(1.0f - cfg.turnGain, dt);
		float maxStep = cfg.maxTurnRate * dt;
		out.active = true;

		if (s.inRear)
		{
			// Backpedal: Leon keeps his facing and the sideways part of the left stick turns him. This is a turn rate rather than a
			// heading to reach, so it never fights the game's own quick turn (back + run).
			float side = std::atan2(x, -stickY); // positive = right
			float step = std::max(-maxStep, std::min(maxStep, -side * k));
			out.newYaw = WrapAngle(playerYaw + step);
			out.backward = true;
			return out;
		}

		// Steer toward the left stick direction relative to the live camera (minus the shoulder offset, so a settled chase camera
		// gives exactly Leon's facing).
		float target = WrapAngle(cameraYaw - cfg.shoulderOffset - stickAngle);
		float err = WrapAngle(target - playerYaw);
		float step = std::max(-maxStep, std::min(maxStep, err * k));

		// Walking straight: ignore tiny errors so the camera settling cannot make him hunt left and right
		if (x == 0.0f && std::fabs(err) < 0.04f)
			step = 0.0f;

		out.newYaw = WrapAngle(playerYaw + step);
		out.forward = std::fabs(err - step) < cfg.facingThreshold; // error left after this frame's turn
		return out;
	}

	// ---- Detached camera ----

	// CameraQuasiFPS::calcDepressionRatio turns a right stick value v into a camera yaw offset of (-v / C_RANGE) * 1.0471976 radians, relative to
	// Leon's base matrix. With C_RANGE = 42.3 and v = unit * 127 (the float path CameraImprovements uses), one unit is about pi radians.
	constexpr float kRatioPerUnit = (127.0f / 42.3f) * 1.0471976f;

	// Yaw of the player matrix the chase camera is built on (its Z axis, same convention as ang_A0.y).
	inline float MatrixYaw(float m02, float m22)
	{
		return std::atan2(m02, m22);
	}

	// The unit that puts the camera at cameraTarget: camera yaw = base yaw + shoulder offset + offset. Never exactly 0, which the game
	// treats as "right stick idle" and only decays the old offset.
	inline float PinnedCameraOffsetUnit(float cameraTarget, float baseYaw)
	{
		float u = -WrapAngle(cameraTarget - (baseYaw + kShoulderOffset)) / kRatioPerUnit;
		u = std::max(-1.0f, std::min(1.0f, u));
		return u == 0.0f ? 1e-6f : u;
	}

	// Right stick X -> camera yaw change. Squared response for fine control, right turns the camera right (yaw decreases).
	inline float OrbitStep(float rx, float speed, float dt, float deadzone = 0.20f)
	{
		float a = std::fabs(rx);
		if (a <= deadzone)
			return 0.0f;
		float v = std::min((a - deadzone) / (1.0f - deadzone), 1.0f);
		return (rx > 0.0f ? -1.0f : 1.0f) * v * v * speed * dt;
	}

	// ---- Detached run ----

	// Run when the left stick is pushed past runRadius, and keep running until it drops 0.10 below that, so it doesn't flicker at the
	// edge. A radius of 1.0 turns it off (the left stick only reaches 1.0 at the very rim, and not on every pad).
	inline bool RunByTilt(bool wasRunning, float stickX, float stickY, float runRadius, float deadzone = 0.25f)
	{
		if (runRadius >= 0.995f)
			return false;

		float tilt = std::sqrt(stickX * stickX + stickY * stickY);
		float exitRadius = std::max(deadzone + 0.05f, runRadius - 0.10f);
		return wasRunning ? tilt > exitRadius : tilt >= runRadius;
	}
}
