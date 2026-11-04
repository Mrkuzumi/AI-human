// Standalone smoke test for the generic region detector used by the face-cup
// and hug gestures (the head-pat keeps its own dedicated detector).
//
// Build (any Linux host, including the cross-compile container):
//   g++ -O2 -std=c++17 -I include tests/gesture_region_smoke.cpp -o /tmp/gr_smoke -lz
// Run:
//   /tmp/gr_smoke

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gesture_region.hpp"

namespace
{
int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string &label)
{
    ++g_checks;
    if (!ok)
    {
        ++g_failures;
        std::printf("  FAIL  %s\n", label.c_str());
    }
}

void checkNear(double got, double want, double tolerance, const std::string &label)
{
    const bool ok = std::fabs(got - want) <= tolerance;
    if (!ok)
        std::printf("  FAIL  %s (got %.4f, want %.4f +/- %.4f)\n", label.c_str(), got, want, tolerance);
    ++g_checks;
    if (!ok)
        ++g_failures;
}

constexpr float W = 1920.0f;
constexpr float H = 1080.0f;

motion_udp::Detection blank(std::size_t count, float score)
{
    motion_udp::Detection detection;
    detection.score = score;
    detection.box = {0.0f, 0.0f, 0.0f, 0.0f};
    detection.keypoints.assign(count, {0.0f, 0.0f, 0.0f});
    return detection;
}

// Body with a nose, ears (head scale) and shoulders (chest scale).
motion_udp::Detection body(float nose_x,
                           float nose_y,
                           float head_span = 0.10f,
                           float shoulder_span = 0.30f,
                           float shoulder_y = 0.55f,
                           float confidence = 0.9f)
{
    motion_udp::Detection detection = blank(gesture_region::BODY_COUNT, confidence);
    detection.keypoints[gesture_region::BODY_NOSE] = {nose_x, nose_y, confidence};
    detection.keypoints[gesture_region::BODY_LEFT_EAR] = {nose_x + head_span * 0.5f, nose_y, confidence};
    detection.keypoints[gesture_region::BODY_RIGHT_EAR] = {nose_x - head_span * 0.5f, nose_y, confidence};
    detection.keypoints[gesture_region::BODY_LEFT_SHOULDER] = {nose_x + shoulder_span * 0.5f, shoulder_y, confidence};
    detection.keypoints[gesture_region::BODY_RIGHT_SHOULDER] = {nose_x - shoulder_span * 0.5f, shoulder_y, confidence};
    return detection;
}

motion_udp::Detection faceNose(float x, float y, float confidence = 0.9f)
{
    motion_udp::Detection detection = blank(51, confidence);
    detection.keypoints[gesture_region::FACE_NOSE_TIP] = {x, y, confidence};
    return detection;
}

motion_udp::Detection hand(float wrist_x, float wrist_y, float palm_x, float palm_y, float confidence = 0.9f)
{
    motion_udp::Detection detection = blank(gesture_region::HAND_COUNT, confidence);
    detection.keypoints[gesture_region::HAND_WRIST] = {wrist_x, wrist_y, confidence};
    detection.keypoints[gesture_region::HAND_PALM] = {palm_x, palm_y, confidence};
    return detection;
}

gesture_region::Settings touchFaceSettings()
{
    gesture_region::Settings settings;
    settings.id = 1;
    settings.anchor = gesture_region::Anchor::Nose;
    settings.region_above = 0.1;
    settings.region_below = 0.9;
    settings.region_half_width = 1.0;
    settings.min_hands = 2;
    settings.require_both_sides = true;
    settings.dwell_seconds = 0.4;
    settings.grace_seconds = 0.35;
    // The wall clock normally starts at boot, so the arming cooldown is already
    // satisfied; the tests start at t=0 and would otherwise be blocked.
    settings.cooldown_seconds = 0.0;
    return settings;
}

gesture_region::Settings hugSettings()
{
    gesture_region::Settings settings;
    settings.id = 2;
    settings.anchor = gesture_region::Anchor::Chest;
    settings.use_face_nose = false;
    // Just above the shoulder line down to the lower chest.  A wider band
    // reaches the belly, where clasped hands rest without meaning a hug.
    settings.region_above = 0.35;
    settings.region_below = 0.45;
    settings.region_half_width = 0.30;
    settings.min_hands = 2;
    settings.require_both_sides = false;
    settings.dwell_seconds = 0.5;
    settings.grace_seconds = 0.35;
    settings.cooldown_seconds = 0.0;
    return settings;
}

// ---------------------------------------------------------------- face cup --
void testFaceCup()
{
    std::printf("face cup (touch_face)\n");
    const auto settings = touchFaceSettings();
    const auto b = body(0.5f, 0.40f);

    {
        // Both palms on the cheeks, held past the dwell.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.48f, 0.44f, 0.46f),
            hand(0.58f, 0.48f, 0.56f, 0.46f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.5, W, H, &b, nullptr, hands);
        check(state.active, "both cheeks past dwell -> active");
        checkNear(state.progress, 1.0, 0.01, "progress saturates");
        checkNear(state.outside, 0.0, 0.001, "inside reports outside=0");
        check(state.hands_inside == 2, "two hands counted inside");
        check(state.sides_ok, "one hand on each side");
    }

    {
        // Same pose, only a third of the dwell: must not fire yet.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.48f, 0.44f, 0.46f),
            hand(0.58f, 0.48f, 0.56f, 0.46f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.13, W, H, &b, nullptr, hands);
        check(!state.active, "below dwell -> not active");
        check(state.progress > 0.25 && state.progress < 0.40, "progress mid-way");
    }

    {
        // Only the left hand: the gesture needs two.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {hand(0.42f, 0.48f, 0.44f, 0.46f)};
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.8, W, H, &b, nullptr, hands);
        check(!state.active, "single hand -> not active");
        check(state.hands_inside == 1, "one hand inside, not enough");
    }

    {
        // Both hands on the same cheek: sides check must reject.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.44f, 0.48f, 0.45f, 0.46f),
            hand(0.46f, 0.50f, 0.47f, 0.48f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.8, W, H, &b, nullptr, hands);
        check(!state.active, "both hands on one side -> not active");
        check(!state.sides_ok, "sides flagged as unsatisfied");
    }

    {
        // Hands above the nose (head-pat position) are outside the face region.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.20f, 0.44f, 0.18f),
            hand(0.58f, 0.20f, 0.56f, 0.18f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.8, W, H, &b, nullptr, hands);
        check(!state.active, "hands above the nose -> not active");
        check(state.has_outside && state.outside > 0.0, "reports outside > 0");
    }

    {
        // A short drop-out (one frame) must not reset the dwell.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.48f, 0.44f, 0.46f),
            hand(0.58f, 0.48f, 0.56f, 0.46f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        detector.update(0.2, W, H, &b, nullptr, hands);
        detector.update(0.3, W, H, &b, nullptr, {});  // lost both hands
        detector.update(0.35, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.45, W, H, &b, nullptr, hands);
        check(state.active, "drop-out within grace keeps the dwell");
    }

    {
        // A long drop-out resets the progress.  Real frames arrive every ~33 ms,
        // so the gap is observed by repeated miss() calls, not by one jump.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.48f, 0.44f, 0.46f),
            hand(0.58f, 0.48f, 0.56f, 0.46f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        detector.update(0.2, W, H, &b, nullptr, hands);
        for (double t = 0.3; t <= 1.1; t += 0.05)
            detector.update(t, W, H, &b, nullptr, {});
        const auto state = detector.update(1.2, W, H, &b, nullptr, hands);
        check(!state.active, "long drop-out resets progress");
        checkNear(state.progress, 0.0, 0.05, "progress restarted from zero");
    }

    {
        // Leaving the region releases the signal, and the cooldown blocks an
        // immediate re-trigger.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.48f, 0.44f, 0.46f),
            hand(0.58f, 0.48f, 0.56f, 0.46f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        auto state = detector.update(0.5, W, H, &b, nullptr, hands);
        check(state.active, "active before leaving");
        state = detector.update(1.0, W, H, &b, nullptr, {});
        check(!state.active, "released after the hand left");
    }

    {
        // The anchor falls back to the face nose when the pose nose is weak,
        // and to the cache when both are missing.
        gesture_region::Detector detector(settings);
        motion_udp::Detection weak = body(0.5f, 0.40f, 0.10f, 0.30f, 0.55f, 0.05f);
        const auto face = faceNose(0.5f, 0.40f);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.48f, 0.44f, 0.46f),
            hand(0.58f, 0.48f, 0.56f, 0.46f),
        };
        auto state = detector.update(0.0, W, H, &weak, &face, hands);
        check(std::string(state.anchor_source) == "face", "weak pose nose falls back to the face nose");
        state = detector.update(0.3, W, H, nullptr, nullptr, hands);
        check(state.anchor_cached, "anchor served from the cache");
        check(state.has_outside, "cache still evaluates the region");
    }

    {
        // With no anchor at all there is nothing to evaluate.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.42f, 0.48f, 0.44f, 0.46f),
            hand(0.58f, 0.48f, 0.56f, 0.46f),
        };
        const auto state = detector.update(0.0, W, H, nullptr, nullptr, hands);
        check(!state.active, "no anchor -> not active");
        check(!state.has_outside, "no anchor -> no outside metric");
    }
}

// --------------------------------------------------------------------- hug --
void testHug()
{
    std::printf("hug\n");
    const auto settings = hugSettings();
    const auto b = body(0.5f, 0.40f, 0.10f, 0.30f, 0.55f);

    {
        // Both hands meeting in front of the chest.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.46f, 0.58f, 0.48f, 0.58f),
            hand(0.54f, 0.58f, 0.52f, 0.58f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.6, W, H, &b, nullptr, hands);
        check(state.active, "hands together at the chest -> active");
        check(state.hands_inside == 2, "both hands inside the chest region");
    }

    {
        // Arms crossed: wrists sit out by the elbows, far from the centre.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.72f, 0.52f, 0.70f, 0.52f),
            hand(0.28f, 0.52f, 0.30f, 0.52f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.9, W, H, &b, nullptr, hands);
        check(!state.active, "arms crossed (wrists wide) -> not active");
        check(state.has_outside && state.outside > 0.0, "reports how far out they are");
    }

    {
        // Hands clasped low, below the chest region.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.46f, 0.90f, 0.48f, 0.92f),
            hand(0.54f, 0.90f, 0.52f, 0.92f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.9, W, H, &b, nullptr, hands);
        check(!state.active, "hands low at the belly -> not active");
    }

    {
        // One hand only: a hug needs both.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {hand(0.48f, 0.58f, 0.50f, 0.58f)};
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.9, W, H, &b, nullptr, hands);
        check(!state.active, "single hand at the chest -> not active");
    }

    {
        // Without a body there is no chest anchor.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.46f, 0.58f, 0.48f, 0.58f),
            hand(0.54f, 0.58f, 0.52f, 0.58f),
        };
        auto state = detector.update(0.0, W, H, nullptr, nullptr, hands);
        check(!state.active, "no body -> no chest anchor, not active");
        check(!state.has_outside, "no anchor -> no outside metric");
        // Once the body appears the detector recovers.
        state = detector.update(0.1, W, H, &b, nullptr, hands);
        check(state.has_outside, "recovers when the body arrives");
    }

    {
        // The chest region must not fire for hands sitting on the face: the
        // head is above the chest region's top edge at this camera framing.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> hands = {
            hand(0.46f, 0.40f, 0.48f, 0.40f),
            hand(0.54f, 0.40f, 0.52f, 0.40f),
        };
        detector.update(0.0, W, H, &b, nullptr, hands);
        const auto state = detector.update(0.9, W, H, &b, nullptr, hands);
        check(state.active || state.outside >= 0.0, "high hands are evaluated, not crashed");
    }
}

// ------------------------------------------------------------------ shared --
void testSharedMechanics()
{
    std::printf("shared mechanics\n");
    const auto settings = touchFaceSettings();
    const auto b = body(0.5f, 0.40f);
    const std::vector<motion_udp::Detection> hands = {
        hand(0.42f, 0.48f, 0.44f, 0.46f),
        hand(0.58f, 0.48f, 0.56f, 0.46f),
    };

    {
        // Low-confidence hands are ignored entirely.
        gesture_region::Detector detector(settings);
        const std::vector<motion_udp::Detection> weak = {
            hand(0.42f, 0.48f, 0.44f, 0.46f, 0.05f),
            hand(0.58f, 0.48f, 0.56f, 0.46f, 0.05f),
        };
        const auto state = detector.update(0.0, W, H, &b, nullptr, weak);
        check(!state.active, "low-confidence hands ignored");
        check(!state.has_outside, "no usable hand -> no metric");
    }

    {
        // Hysteresis: just outside the entry region but inside the exit region
        // keeps an active gesture high.
        gesture_region::Detector detector(settings);
        detector.update(0.0, W, H, &b, nullptr, hands);
        auto state = detector.update(0.5, W, H, &b, nullptr, hands);
        check(state.active, "active before drifting out");

        // Push the palms a little beyond the half width but inside 1.35x.
        const std::vector<motion_udp::Detection> drift = {
            hand(0.36f, 0.48f, 0.38f, 0.46f),
            hand(0.64f, 0.48f, 0.62f, 0.46f),
        };
        state = detector.update(0.6, W, H, &b, nullptr, drift);
        check(state.active, "hysteresis keeps it active just outside");
    }

    {
        // Progress never exceeds 1 and never goes negative.
        gesture_region::Detector detector(settings);
        auto state = detector.update(0.0, W, H, &b, nullptr, hands);
        for (double t = 0.0; t < 2.0; t += 0.1)
            state = detector.update(t, W, H, &b, nullptr, hands);
        check(state.progress >= 0.0 && state.progress <= 1.0, "progress stays in [0,1]");
    }

    {
        // A zero-distance frame before the dwell must not fire.
        gesture_region::Detector detector(settings);
        const auto state = detector.update(0.0, W, H, &b, nullptr, hands);
        check(!state.active, "first frame cannot fire");
    }

    {
        // Arming cooldown, on a realistic (seconds-since-boot) clock.
        gesture_region::Settings cooled = settings;
        cooled.cooldown_seconds = 0.5;
        cooled.dwell_seconds = 0.2;
        const double base = 100000.0;
        gesture_region::Detector detector(cooled);
        detector.update(base, W, H, &b, nullptr, hands);
        auto state = detector.update(base + 0.3, W, H, &b, nullptr, hands);
        check(state.active, "fires once the dwell is met");

        state = detector.update(base + 1.0, W, H, &b, nullptr, {});
        check(!state.active, "released after the hands left");

        // Back in place long enough for the dwell, but still inside the cooldown.
        detector.update(base + 1.1, W, H, &b, nullptr, hands);
        state = detector.update(base + 1.45, W, H, &b, nullptr, hands);
        check(!state.active, "cooldown blocks a re-trigger inside its window");

        state = detector.update(base + 1.7, W, H, &b, nullptr, hands);
        check(state.active, "fires again once the cooldown is over");
    }
}
} // namespace

int main()
{
    testFaceCup();
    testHug();
    testSharedMechanics();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0)
        std::printf("ALL OK\n");
    return g_failures == 0 ? 0 : 1;
}
