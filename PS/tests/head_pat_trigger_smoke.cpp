// Standalone smoke test for the PS-side head-pat detector.
//
// It mirrors the scenarios covered by pc/tests/test_gesture_trigger.py so the
// C++ port can be compared against the Python reference without needing a
// camera, the NPU or the PC bridge.
//
// Build (on the board, or any aarch64/x86 Linux host):
//   g++ -O2 -std=c++17 -I include tests/head_pat_trigger_smoke.cpp -o /tmp/head_pat_smoke -lz
// Run:
//   /tmp/head_pat_smoke

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "head_pat_trigger.hpp"

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

motion_udp::Detection blank(std::size_t count, float score)
{
    motion_udp::Detection detection;
    detection.score = score;
    detection.box = {0.0f, 0.0f, 0.0f, 0.0f};
    detection.keypoints.assign(count, {0.0f, 0.0f, 0.0f});
    return detection;
}

// Normalized body with ears symmetric about the nose; span is ear-to-ear in
// normalized x, which the detector turns into pixels with the frame width.
motion_udp::Detection body(float nose_x, float nose_y, float span = 0.10f, float confidence = 0.9f)
{
    motion_udp::Detection detection = blank(head_pat::BODY_COUNT, confidence);
    detection.keypoints[head_pat::BODY_NOSE] = {nose_x, nose_y, confidence};
    detection.keypoints[head_pat::BODY_LEFT_EAR] = {nose_x + span * 0.5f, nose_y, confidence};
    detection.keypoints[head_pat::BODY_RIGHT_EAR] = {nose_x - span * 0.5f, nose_y, confidence};
    return detection;
}

motion_udp::Detection faceNose(float x, float y, float confidence = 0.9f)
{
    motion_udp::Detection detection = blank(51, confidence);
    detection.keypoints[head_pat::FACE_NOSE_TIP] = {x, y, confidence};
    return detection;
}

motion_udp::Detection hand(float wrist_x, float wrist_y, float palm_x, float palm_y, float confidence = 0.9f)
{
    motion_udp::Detection detection = blank(head_pat::HAND_COUNT, confidence);
    detection.keypoints[head_pat::HAND_WRIST] = {wrist_x, wrist_y, confidence};
    detection.keypoints[head_pat::HAND_PALM] = {palm_x, palm_y, confidence};
    return detection;
}

constexpr float FRAME_W = 1920.0f;
constexpr float FRAME_H = 1080.0f;

// Nose at the frame centre with a 0.10 normalized ear span -> 192 px head width,
// so the pat region spans x in [806, 1114] and y in [290, 578].
constexpr float NOSE_X = 0.5f;
constexpr float NOSE_Y = 0.5f;

head_pat::State step(head_pat::Trigger &trigger,
                     double now,
                     const motion_udp::Detection *body_detection,
                     const std::vector<motion_udp::Detection> &hands,
                     const motion_udp::Detection *face_detection = nullptr)
{
    return trigger.update(now, FRAME_W, FRAME_H, body_detection, face_detection, hands);
}

void testDwellLatches()
{
    std::printf("dwell latches after dwell_seconds\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> hands{hand(0.5f, 0.40f, 0.5f, 0.38f)};

    bool active = false;
    double now = 100.0;
    for (int i = 0; i < 20; ++i)
    {
        const auto state = step(trigger, now, &body_detection, hands);
        if (state.active)
        {
            active = true;
            checkNear(state.dwell, 1.0, 0.15, "latched at ~1.0s dwell");
            break;
        }
        now += 0.1;
    }
    check(active, "gesture became active");
}

void testSignalStaysHigh()
{
    std::printf("signal stays high while the hand rests in the region\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> hands{hand(0.5f, 0.40f, 0.5f, 0.38f)};

    double now = 200.0;
    for (int i = 0; i < 15; ++i)
        step(trigger, now + i * 0.1, &body_detection, hands);

    for (int i = 15; i < 40; ++i)
        check(step(trigger, now + i * 0.1, &body_detection, hands).active, "still active at step " + std::to_string(i));
}

void testBriefDropoutKeepsDwell()
{
    std::printf("dropout shorter than grace keeps the dwell\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> inside{hand(0.5f, 0.40f, 0.5f, 0.38f)};

    double now = 300.0;
    for (int i = 0; i < 5; ++i) // 0.5s of dwell
        step(trigger, now + i * 0.1, &body_detection, inside);

    for (int i = 0; i < 2; ++i) // 0.2s gap, under the 0.25s grace
        step(trigger, now + 0.5 + i * 0.1, &body_detection, {});

    const auto state = step(trigger, now + 0.7, &body_detection, inside);
    check(state.dwell > 0.55, "dwell survived the brief dropout");
    checkNear(state.progress, 0.7, 0.2, "progress reflects the preserved dwell");
}

void testLongDropoutResets()
{
    std::printf("dropout longer than grace resets the dwell\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> inside{hand(0.5f, 0.40f, 0.5f, 0.38f)};

    double now = 400.0;
    for (int i = 0; i < 5; ++i)
        step(trigger, now + i * 0.1, &body_detection, inside);

    // 0.6s without any hand data (finger model dropped out entirely).
    step(trigger, now + 0.5, &body_detection, {});
    step(trigger, now + 1.1, &body_detection, {});

    const auto state = step(trigger, now + 1.2, &body_detection, inside);
    check(state.dwell < 0.2, "dwell was reset after the long gap");
    check(!state.active, "not active after the reset");
}

void testReleaseAfterLeaving()
{
    std::printf("release drops the signal once the hand leaves for release_seconds\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> inside{hand(0.5f, 0.40f, 0.5f, 0.38f)};
    const std::vector<motion_udp::Detection> outside{hand(0.5f, 0.90f, 0.5f, 0.88f)};

    double now = 500.0;
    for (int i = 0; i < 13; ++i)
        step(trigger, now + i * 0.1, &body_detection, inside);
    check(trigger.active(), "latched before leaving");

    // 0.2s outside is shorter than the 0.3s release, so the signal must hold.
    step(trigger, now + 1.3, &body_detection, outside);
    step(trigger, now + 1.4, &body_detection, outside);
    check(trigger.active(), "still active during the short excursion");

    // Past the release window it must drop.
    step(trigger, now + 1.6, &body_detection, outside);
    check(!trigger.active(), "released after leaving long enough");
}

void testWalkAwayReset()
{
    std::printf("losing the performer resets the detector\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> inside{hand(0.5f, 0.40f, 0.5f, 0.38f)};

    double now = 600.0;
    for (int i = 0; i < 13; ++i)
        step(trigger, now + i * 0.1, &body_detection, inside);
    check(trigger.active(), "latched while the performer was visible");

    // No body and no face for a while: the nose cache expires, then the miss
    // path must release the signal through the normal hysteresis.
    for (int i = 0; i < 20; ++i)
        step(trigger, now + 1.3 + i * 0.1, nullptr, inside);

    check(!trigger.active(), "released after the performer left");
}

void testGeometryRejectsFarHand()
{
    std::printf("geometry rejects a hand outside the head region\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);

    // Far below the nose (chest height) and far to the side: never inside.
    const std::vector<std::vector<motion_udp::Detection>> far_hands{
        {hand(0.5f, 0.90f, 0.5f, 0.88f)},
        {hand(0.05f, 0.50f, 0.05f, 0.48f)},
        {hand(0.5f, 0.10f, 0.5f, 0.12f)},
    };
    for (std::size_t i = 0; i < far_hands.size(); ++i)
    {
        head_pat::Trigger fresh;
        double now = 700.0;
        bool any_active = false;
        for (int step_index = 0; step_index < 25; ++step_index)
        {
            const auto state = step(fresh, now + step_index * 0.1, &body_detection, far_hands[i]);
            any_active = any_active || state.active;
            check(state.has_outside, "outside metric reported");
            check(state.outside > 0.0, "hand measured as outside case " + std::to_string(i));
        }
        check(!any_active, "never latched for far hand case " + std::to_string(i));
    }
}

void testWholeHandUsesWorstPoint()
{
    std::printf("whole-hand mode needs every tracked point inside\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    // Palm inside the region, wrist still down at chest level.
    const std::vector<motion_udp::Detection> hands{hand(0.5f, 0.95f, 0.5f, 0.38f)};

    const auto state = step(trigger, 800.0, &body_detection, hands);
    check(state.has_outside, "outside metric reported");
    check(state.outside > 0.0, "wrist outside keeps the whole hand out");

    // With whole_hand disabled only the palm matters, so the same pose is inside.
    head_pat::Settings settings;
    settings.whole_hand = false;
    head_pat::Trigger palm_only(settings);
    const auto palm_state = step(palm_only, 800.0, &body_detection, hands);
    check(palm_state.has_outside, "palm-only outside metric reported");
    checkNear(palm_state.outside, 0.0, 1e-6, "palm-only pose is inside");
}

void testHeadWidthCaching()
{
    std::printf("head width is cached while the ears are hidden\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> inside{hand(0.5f, 0.40f, 0.5f, 0.38f)};

    // One frame with visible ears teaches the detector the real span (192 px).
    const auto learned = step(trigger, 900.0, &body_detection, inside);
    checkNear(learned.head_width_px, 192.0, 1.0, "measured ear span");

    // Ears occluded: the cached span must be reused instead of the 0.20*height
    // fallback (216 px), which would balloon the region.
    auto hidden = body(NOSE_X, NOSE_Y);
    hidden.keypoints[head_pat::BODY_LEFT_EAR][2] = 0.0f;
    hidden.keypoints[head_pat::BODY_RIGHT_EAR][2] = 0.0f;
    const auto cached = step(trigger, 901.0, &hidden, inside);
    checkNear(cached.head_width_px, 192.0, 1.0, "cached span reused");

    // Past the cache window it falls back.
    const auto expired = step(trigger, 905.0, &hidden, inside);
    checkNear(expired.head_width_px, 0.20 * FRAME_H, 1.0, "fallback after cache expiry");
}

void testNoseFallsBackToFace()
{
    std::printf("nose anchor falls back to the face nose tip\n");
    head_pat::Trigger trigger;
    auto low_body = body(NOSE_X, NOSE_Y);
    low_body.keypoints[head_pat::BODY_NOSE][2] = 0.05f; // below min_confidence
    const auto face_detection = faceNose(0.5f, 0.45f);
    const std::vector<motion_udp::Detection> hands{hand(0.5f, 0.35f, 0.5f, 0.33f)};

    const auto state = step(trigger, 1000.0, &low_body, hands, &face_detection);
    check(std::string(state.nose_source) == "face", "nose came from the face model");
    checkNear(state.nose_y, 0.45 * FRAME_H, 1.0, "face nose tip used as the anchor");
}

void testLowConfidenceHandIgnored()
{
    std::printf("low-confidence hands are ignored\n");
    head_pat::Trigger trigger;
    const auto body_detection = body(NOSE_X, NOSE_Y);
    const std::vector<motion_udp::Detection> hands{hand(0.5f, 0.40f, 0.5f, 0.38f, 0.10f)};

    const auto state = step(trigger, 1100.0, &body_detection, hands);
    check(!state.has_outside, "no outside metric for a rejected hand");
    check(state.hand_index == -1, "no hand index for a rejected hand");
}
} // namespace

int main()
{
    std::printf("head_pat_trigger smoke test\n");
    testDwellLatches();
    testSignalStaysHigh();
    testBriefDropoutKeepsDwell();
    testLongDropoutResets();
    testReleaseAfterLeaving();
    testWalkAwayReset();
    testGeometryRejectsFarHand();
    testWholeHandUsesWorstPoint();
    testHeadWidthCaching();
    testNoseFallsBackToFace();
    testLowConfidenceHandIgnored();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0)
        std::printf("ALL OK\n");
    return g_failures == 0 ? 0 : 1;
}
