/* README:

    This is a three-model YOLOv8 pose, face, and finger NPU runtime using a single PLin SDI camera,
    and 1 HDMI display.

    Pipeline拓扑结构:
                                    +---------+
                                    |         |
                                    |         |
    +------------+  Q               |   NPU   |   Q               +---------------------+
    |SDI Camera 0| -- Msg(id=0) --> |         |  -- Msg(id=0) --> | RGB565 HDMI DISPLAY |
    +------------+                  |  Actor  |                   +---------------------+
                                    |         |
                                    |         |
                                    +---------+

    说明:
    1. 单路 PLin-SDICamera 通过 SDICameraInputActor 采集图像后，发送消息到同一个 message queue.
    2. NPU Actor 读取唯一的 message queue ，不区分source_id处理AI前向。
    3. NPU Actor 处理后的消息通过 MessageRouter，根据 source_id 路由到对应的 HDMI Display Actor。
    4. 每个摄像头和每个显示接收器都有唯一的 source_id (0)，确保消息正确路由。
    5. 这种设计允许系统灵活扩展，添加更多摄像头或处理节点，只需更新路由表。


    准备：
    1. 下载对应位流：https://download.fdwxhb.com/data/04_FPAI/02_Icraft/v3.36/%e5%8f%82%e8%80%83%e5%ae%9e%e7%8e%b0/%e6%82%9f%e7%a9%ba%e5%bc%80%e5%8f%91%e6%9d%bf/%e5%8d%95%e8%b7%afPLIN+pHDMI%e6%98%be%e7%a4%ba%e5%8f%82%e8%80%83%e5%ae%9e%e7%8e%b0/25060601/
    2. 将 BOOT.bin 放到 /root/bits 下，重启板子
    3. 确保两个网络摄像头开启，配置正确的编码格式和 URL 地址，在 examples/1_single_input+ai/PLin+SingleNet+HDMI/configs 目录下的文件中配置。
*/

// application includes
#include "postprocess_yolov8_pose.hpp"
#include "postprocess_yolov8_finger.hpp"
#include "yolov8_pose_npu_actor.hpp"
#include "motion_udp_sender.hpp"
#include "finger_udp_sender.hpp"
#include "frame_udp_sender.hpp"
#include "gesture_udp_sender.hpp"
#include "head_pat_trigger.hpp"
#include "gesture_region.hpp"
#include "bytetrack.hpp"
#include "primary_track_selector.hpp"

// pipeline includes
#include "pipeline/actor/message_router.hpp"
#include "pipeline/actor/sdicamera_input_actor.hpp"
#include "pipeline/actor/hdmi_display_actor.hpp"
#include "pipeline/io/input/sdicamera.hpp"
#include "pipeline/memory/buffer_manager.hpp"
// modelzoo_utils includes
#include "file_utils.hpp"
#include "et_device.hpp"
#include "fps_calculator.hpp"
#include "vis_helper.hpp"
#include "icraft_utils.hpp"
// icraft includes
#include <icraft-xrt/core/session.h>
#include "compile_fpai_target.hpp"
// 3rd-party dependency includes
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
// system libs
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <condition_variable>
#include <atomic>
#include <thread>
#include <functional>
#include <filesystem>
#include <csignal>
#include <cmath>
#include <mutex>
#include <optional>

using namespace icraft::xrt;
using namespace icraft::xir;
using namespace std::chrono;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

using namespace fpai;

const std::string DEMO_NAME = "yolov8-pose+yolov8-face+yolov8-finger+hdmi";

const size_t BUFFER_COUNT = 4;
const size_t CAMERA_COUNT = 1;

// ---- hwpre 寄存器 (PL 端 yolo_hw_preprocess 模块) ----
const uint64_t HWPRE_REG_BASE = 0x80090000;
const uint64_t HWPRE_ARB_CFG_OFFSET = 0x1C;          // MIG 仲裁器: arb_m0_slots
const uint32_t HWPRE_PIPELINE_VERSION = 0x20260729;  // 与 bit 内 hwpre VERSION 寄存器一致
// hwpre 几何 (由 bit 的 hwpre_top_external 参数决定):
// 1920x1080 → bilinear 等比缩放 → 626x352 → 左右各 pad 7 (值114) → 640x352
const int HWPRE_OUT_H = 352, HWPRE_OUT_W = 640, HWPRE_OUT_C = 3;
const float HWPRE_SCALE_X = 626.0f / 1920.0f;   // 缩放到原始 1920 的宽度还原系数
const float HWPRE_SCALE_Y = 352.0f / 1080.0f;   // 缩放到原始 1080 的高度还原系数
const float HWPRE_PAD_X   = 7.0f;
const float HWPRE_PAD_Y   = 0.0f;

constexpr bool SAVE = false; // 是否保存推理结果到文件，保存格式为: <class_id> <left> <top> <width> <height> <confidence>
constexpr bool DRAW = true; // 是否在输出上绘制检测框

namespace
{
volatile std::sig_atomic_t running = 1;

void stopOnSignal(int)
{
    running = 0;
}

float unitFloat(float value)
{
    return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 0.0f;
}

std::optional<motion_udp::Detection> motionDetectionAt(
    const std::vector<float> &scores,
    const std::vector<cv::Rect2f> &boxes,
    const std::vector<std::vector<std::vector<float>>> &keypoints,
    std::size_t index,
    std::size_t expected_keypoints)
{
    if (index >= scores.size() || index >= boxes.size() || index >= keypoints.size() ||
        keypoints[index].size() != expected_keypoints)
        return std::nullopt;

    const auto &box = boxes[index];
    const float x1 = unitFloat((box.x - HWPRE_PAD_X) / (HWPRE_OUT_W - 2.0f * HWPRE_PAD_X));
    const float y1 = unitFloat((box.y - HWPRE_PAD_Y) / HWPRE_OUT_H);
    const float x2 = unitFloat((box.x + box.width - HWPRE_PAD_X) / (HWPRE_OUT_W - 2.0f * HWPRE_PAD_X));
    const float y2 = unitFloat((box.y + box.height - HWPRE_PAD_Y) / HWPRE_OUT_H);

    motion_udp::Detection detection;
    detection.score = unitFloat(scores[index]);
    detection.box = {x1, y1, std::max(0.0f, x2 - x1), std::max(0.0f, y2 - y1)};
    detection.keypoints.reserve(expected_keypoints);
    for (const auto &point : keypoints[index])
    {
        if (point.size() < 3)
            return std::nullopt;
        detection.keypoints.push_back({
            unitFloat((point[0] - HWPRE_PAD_X) / (HWPRE_OUT_W - 2.0f * HWPRE_PAD_X)),
            unitFloat((point[1] - HWPRE_PAD_Y) / HWPRE_OUT_H),
            unitFloat(point[2]),
        });
    }
    return detection;
}

std::optional<motion_udp::Detection> selectMotionDetection(
    const std::vector<float> &scores,
    const std::vector<cv::Rect2f> &boxes,
    const std::vector<std::vector<std::vector<float>>> &keypoints,
    std::size_t expected_keypoints)
{
    std::size_t best = scores.size();
    for (std::size_t i = 0; i < scores.size(); ++i)
    {
        if (i >= boxes.size() || i >= keypoints.size() || keypoints[i].size() != expected_keypoints)
            continue;
        if (best == scores.size() || scores[i] > scores[best])
            best = i;
    }
    if (best == scores.size())
        return std::nullopt;
    return motionDetectionAt(scores, boxes, keypoints, best, expected_keypoints);
}

std::optional<motion_udp::Detection> selectFaceForBody(
    const std::vector<float> &scores,
    const std::vector<cv::Rect2f> &boxes,
    const std::vector<std::vector<std::vector<float>>> &keypoints,
    const motion_udp::Detection &body)
{
    std::optional<motion_udp::Detection> best;
    for (std::size_t i = 0; i < scores.size(); ++i)
    {
        auto face = motionDetectionAt(scores, boxes, keypoints, i, motion_udp::FACE_COUNT);
        if (!face)
            continue;
        const float cx = face->box[0] + face->box[2] * 0.5f;
        const float cy = face->box[1] + face->box[3] * 0.5f;
        const bool inside = cx >= body.box[0] && cx <= body.box[0] + body.box[2] &&
                            cy >= body.box[1] && cy <= body.box[1] + body.box[3];
        if (inside && (!best || face->score > best->score))
            best = std::move(face);
    }
    return best;
}

std::vector<finger_udp::Detection> selectFingerDetections(const YoloPostResult &results)
{
    const auto &scores = std::get<1>(results);
    const auto &boxes = std::get<2>(results);
    const auto &points = std::get<3>(results);
    std::vector<size_t> order;
    for (size_t i = 0; i < scores.size(); ++i)
        if (i < boxes.size() && i < points.size() && points[i].size() == finger_udp::POINT_COUNT)
            order.push_back(i);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return scores[a] > scores[b]; });
    std::vector<finger_udp::Detection> hands;
    for (size_t i : order)
    {
        // Reuse the existing HWPre inverse geometry and normalized-coordinate conversion.
        auto hand = selectMotionDetection({scores[i]}, {boxes[i]}, {points[i]}, finger_udp::POINT_COUNT);
        if (hand) hands.push_back(std::move(*hand));
        if (hands.size() == finger_udp::MAX_HANDS) break;
    }
    return hands;
}
}

// Use pose wrists as a fallback when the finger model misses a hand.
std::vector<finger_udp::Detection> selectPoseWristHands(const std::optional<motion_udp::Detection> &body)
{
    std::vector<finger_udp::Detection> hands;
    if (!body || body->keypoints.size() <= 10)
        return hands;
    for (const std::size_t idx : {std::size_t(9), std::size_t(10)})
    {
        const auto &wrist = body->keypoints[idx];
        if (wrist.size() < 3 || wrist[2] < 0.3f || !std::isfinite(wrist[0]) || !std::isfinite(wrist[1]))
            continue;
        finger_udp::Detection hand;
        hand.score = wrist[2];
        hand.keypoints.assign(21, wrist);
        hands.push_back(std::move(hand));
    }
    return hands;
}

// Pulse a held pose so Unity can replay the animation without requiring release.
struct GesturePulse
{
    bool on = false;
    double toggle_at = 0.0;
};
static GesturePulse g_pulse_head, g_pulse_face, g_pulse_hug;

static bool gesturePulse(GesturePulse &state, bool detected, double now,
                         double on_len, double off_len)
{
    if (!detected)
    {
        state.on = false;
        state.toggle_at = 0.0;
        return false;
    }
    if (state.toggle_at <= 0.0)
    {
        state.on = true;
        state.toggle_at = now + on_len;
    }
    else if (now >= state.toggle_at)
    {
        state.on = !state.on;
        state.toggle_at = now + (state.on ? on_len : off_len);
    }
    return state.on;
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, stopOnSignal);
    std::signal(SIGTERM, stopOnSignal);

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <config yaml>" << std::endl;
        return 1;
    }
    // ----------------- Load system configuration from YAML file -----------------
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(argv[1]);
    }
    catch (const YAML::BadFile &e)
    {
        spdlog::error("Failed to load config file {}: {}", argv[1], e.what());
        return -1;
    }

    // ----------------- 设置 spdlog 日志级别（默认级别为 info） -----------------
    auto log_level = spdlog::level::debug;
    spdlog::set_level(log_level);
    spdlog::info("Log level set to '{}'", spdlog::level::to_string_view(log_level));
    // --- 日志级别设置完毕 ---

    // ---------------------------- 加载各个Pipeline的配置 -----------------------------------
    const auto &sys_yaml = config["sys"];
    const auto &pose_yaml = config["pipeline"]["pose"];
    const auto &face_yaml = config["pipeline"]["face"];
    const auto &finger_yaml = config["pipeline"]["finger"];
    const auto &display_yaml = config["pipeline"]["display"];
    const auto &camera_yaml = config["pipeline"]["camera"];
    // Parse system configuration
    FPAIConfig fpai_cfg;
    fpai_cfg.device_url = sys_yaml["device"].as<std::string>();
    fpai_cfg.speed_mode = sys_yaml["speedMode"] ? sys_yaml["speedMode"].as<bool>() : false;
    fpai_cfg.compress_ftmp = sys_yaml["compressFtmp"] ? sys_yaml["compressFtmp"].as<bool>() : false;
    fpai_cfg.mmu_mode = sys_yaml["mmuMode"] ? sys_yaml["mmuMode"].as<bool>() : true;
    fpai_cfg.ocm_option = sys_yaml["ocm_option"] ? sys_yaml["ocm_option"].as<int>() : 4;
    fpai_cfg.run_backend = sys_yaml["run_backend"] ? sys_yaml["run_backend"].as<std::string>() : "buyi";
    fpai_cfg.enable_profile = sys_yaml["profile"] ? sys_yaml["profile"].as<bool>() : false;

    // Parse PLin configuration
    const int CAMERA_W = camera_yaml["width"].as<int>();
    const int CAMERA_H = camera_yaml["height"].as<int>();
    const int CAMERA_FPS = camera_yaml["fps"].as<int>();
    const bool VTC_ON = camera_yaml["vtc"] ? camera_yaml["vtc"].as<bool>() : false;

    // Parse encoder configuration
    const int FRAME_W = display_yaml["width"].as<int>();
    const int FRAME_H = display_yaml["height"].as<int>(); // PLin SDICamera推荐使用UYVY

    // Parse icore configuration
    Yolov5Config pose_cfg;
    pose_cfg.name = pose_yaml["model"] ? pose_yaml["model"].as<std::string>() : "yolov8-pose";
    if (pose_yaml["conf"])
        pose_cfg.CONF = pose_yaml["conf"].as<float>();
    if (pose_yaml["iou_thresh"])
        pose_cfg.IOU_THRESHOLD = pose_yaml["iou_thresh"].as<float>();
    if (pose_yaml["multilabel"])
        pose_cfg.MULTILABEL = pose_yaml["multilabel"].as<bool>();
    if (pose_yaml["names"])
        pose_cfg.NAMES_PATH = pose_yaml["names"].as<std::string>();
    if (fs::exists(pose_cfg.NAMES_PATH))
    {
        pose_cfg.LABELS = toVector(pose_cfg.NAMES_PATH);
        pose_cfg.N_CLASS = pose_cfg.LABELS.size();
        spdlog::info("Loaded {} class names from {}", pose_cfg.N_CLASS, pose_cfg.NAMES_PATH);
    }
    else
    {
        spdlog::warn("Names file {} does not exist. No class names loaded.", pose_cfg.NAMES_PATH);
    }
    if (pose_yaml["number_of_class"])
        pose_cfg.N_CLASS = pose_yaml["number_of_class"].as<int>();
    if (pose_yaml["fpga_nms"])
        pose_cfg.FPGA_NMS = pose_yaml["fpga_nms"].as<bool>();
    if (pose_yaml["detpost"])
        pose_cfg.DETPOST = pose_yaml["detpost"].as<bool>();
    if (pose_yaml["net_w"])
        pose_cfg.NET_W = pose_yaml["net_w"].as<int>();
    if (pose_yaml["net_h"])
        pose_cfg.NET_H = pose_yaml["net_h"].as<int>();
    pose_cfg.FRAME_W = FRAME_W;
    pose_cfg.FRAME_H = FRAME_H;
    pose_cfg.JSON_PATHS = {pose_yaml["json"].as<std::string>()};
    pose_cfg.RAW_PATHS = {pose_yaml["raw"].as<std::string>()};
    // YOLOv8 decoupled head is anchor-free; do not inherit Yolov5Config defaults.
    pose_cfg.ANCHORS.clear();
    const int NOP = pose_yaml["number_of_keypoint"] ? pose_yaml["number_of_keypoint"].as<int>() : 17;
    pose_cfg.view_except_op_ids = pose_yaml["view_except_op_ids"] ? pose_yaml["view_except_op_ids"].as<std::vector<int64_t>>() : pose_cfg.view_except_op_ids;

    Yolov5Config face_cfg;
    face_cfg.name = face_yaml["model"] ? face_yaml["model"].as<std::string>() : "yolov8-face";
    face_cfg.CONF = face_yaml["conf"] ? face_yaml["conf"].as<float>() : 0.35f;
    face_cfg.IOU_THRESHOLD = face_yaml["iou_thresh"] ? face_yaml["iou_thresh"].as<float>() : 0.45f;
    face_cfg.MULTILABEL = face_yaml["multilabel"] ? face_yaml["multilabel"].as<bool>() : false;
    face_cfg.N_CLASS = face_yaml["number_of_class"] ? face_yaml["number_of_class"].as<int>() : 1;
    face_cfg.NET_W = face_yaml["net_w"] ? face_yaml["net_w"].as<int>() : 640;
    face_cfg.NET_H = face_yaml["net_h"] ? face_yaml["net_h"].as<int>() : 352;
    face_cfg.FRAME_W = FRAME_W;
    face_cfg.FRAME_H = FRAME_H;
    face_cfg.NAMES_PATH = face_yaml["names"] ? face_yaml["names"].as<std::string>() : "names/yolov8-face.names";
    face_cfg.FPGA_NMS = face_yaml["fpga_nms"] ? face_yaml["fpga_nms"].as<bool>() : false;
    face_cfg.ANCHORS.clear();
    face_cfg.JSON_PATHS = {face_yaml["json"].as<std::string>()};
    face_cfg.RAW_PATHS = {face_yaml["raw"].as<std::string>()};
    face_cfg.LABELS = toVector(face_cfg.NAMES_PATH);

    Yolov5Config finger_cfg;
    finger_cfg.name = "yolov8-finger";
    finger_cfg.CONF = finger_yaml["conf"] ? finger_yaml["conf"].as<float>() : 0.35f;
    finger_cfg.IOU_THRESHOLD = finger_yaml["iou_thresh"] ? finger_yaml["iou_thresh"].as<float>() : 0.45f;
    finger_cfg.MULTILABEL = finger_yaml["multilabel"] ? finger_yaml["multilabel"].as<bool>() : false;
    finger_cfg.N_CLASS = finger_yaml["number_of_class"] ? finger_yaml["number_of_class"].as<int>() : 1;
    finger_cfg.NET_W = finger_yaml["net_w"] ? finger_yaml["net_w"].as<int>() : 640;
    finger_cfg.NET_H = finger_yaml["net_h"] ? finger_yaml["net_h"].as<int>() : 352;
    finger_cfg.FRAME_W = FRAME_W;
    finger_cfg.FRAME_H = FRAME_H;
    finger_cfg.NAMES_PATH = finger_yaml["names"] ? finger_yaml["names"].as<std::string>() : "names/yolov8-finger.names";
    finger_cfg.FPGA_NMS = finger_yaml["fpga_nms"] ? finger_yaml["fpga_nms"].as<bool>() : false;
    finger_cfg.ANCHORS.clear();
    finger_cfg.JSON_PATHS = {finger_yaml["json"].as<std::string>()};
    finger_cfg.RAW_PATHS = {finger_yaml["raw"].as<std::string>()};
    finger_cfg.LABELS = toVector(finger_cfg.NAMES_PATH);
    if (NOP != 17 || face_cfg.N_CLASS != 1 || finger_cfg.N_CLASS != 1 ||
        (face_yaml["number_of_keypoint"] && face_yaml["number_of_keypoint"].as<int>() != 51) ||
        (finger_yaml["number_of_keypoint"] && finger_yaml["number_of_keypoint"].as<int>() != 21))
    {
        spdlog::error("This runtime requires body 17, face 51, hand 21 keypoints and one face/hand class");
        return 1;
    }

    // Parse hwpre configuration
    HwpreDebugConfig hwpre_dbg;
    HwpreRuntimeStats hwpre_stats;
    uint32_t arb_m0_slots = 3;
    if (config["pipeline"] && config["pipeline"]["hwpre"])
    {
        const auto &hd = config["pipeline"]["hwpre"];
        if (hd["debug"])           hwpre_dbg.enabled          = hd["debug"].as<bool>();
        if (hd["debug_frames"])    hwpre_dbg.debug_frames     = hd["debug_frames"].as<int>();
        if (hd["debug_interval"])  hwpre_dbg.debug_interval   = hd["debug_interval"].as<int>();
        if (hd["fatal_on_timeout"])hwpre_dbg.fatal_on_timeout = hd["fatal_on_timeout"].as<bool>();
        if (hd["timeout_ms"])      hwpre_dbg.timeout_ms       = hd["timeout_ms"].as<int>();
        if (hd["arb_m0_slots"])    arb_m0_slots = hd["arb_m0_slots"].as<uint32_t>();
    }

    bool motion_udp_enabled = false;
    std::string motion_udp_host = "192.168.126.100";
    int motion_udp_port = 39540;
    if (config["pipeline"] && config["pipeline"]["motion_udp"])
    {
        const auto &udp = config["pipeline"]["motion_udp"];
        if (udp["enabled"]) motion_udp_enabled = udp["enabled"].as<bool>();
        if (udp["host"]) motion_udp_host = udp["host"].as<std::string>();
        if (udp["port"]) motion_udp_port = udp["port"].as<int>();
    }
    if (motion_udp_enabled && (motion_udp_port < 1 || motion_udp_port > 65535))
    {
        spdlog::error("Invalid motion UDP port: {}", motion_udp_port);
        return 1;
    }

    bool finger_udp_enabled = false;
    std::string finger_udp_host = motion_udp_host;
    int finger_udp_port = 39542;
    if (config["pipeline"]["finger_udp"])
    {
        const auto &udp = config["pipeline"]["finger_udp"];
        if (udp["enabled"]) finger_udp_enabled = udp["enabled"].as<bool>();
        if (udp["host"]) finger_udp_host = udp["host"].as<std::string>();
        if (udp["port"]) finger_udp_port = udp["port"].as<int>();
    }
    if (finger_udp_enabled && (finger_udp_port < 1 || finger_udp_port > 65535 ||
                              (motion_udp_enabled && finger_udp_port == motion_udp_port)))
    {
        spdlog::error("Invalid or conflicting finger UDP port: {}", finger_udp_port);
        return 1;
    }

    bool frame_udp_enabled = false;
    std::string frame_udp_host = motion_udp_host;
    int frame_udp_port = 39541;
    int frame_udp_width = 640;
    int frame_udp_height = 360;
    int frame_udp_quality = 55;
    int frame_udp_stride = 3;
    if (config["pipeline"] && config["pipeline"]["frame_udp"])
    {
        const auto &udp = config["pipeline"]["frame_udp"];
        if (udp["enabled"]) frame_udp_enabled = udp["enabled"].as<bool>();
        if (udp["host"]) frame_udp_host = udp["host"].as<std::string>();
        if (udp["port"]) frame_udp_port = udp["port"].as<int>();
        if (udp["width"]) frame_udp_width = udp["width"].as<int>();
        if (udp["height"]) frame_udp_height = udp["height"].as<int>();
        if (udp["quality"]) frame_udp_quality = udp["quality"].as<int>();
        if (udp["stride"]) frame_udp_stride = udp["stride"].as<int>();
    }
    if (frame_udp_enabled &&
        (frame_udp_port < 1 || frame_udp_port > 65535 || frame_udp_width < 64 ||
         frame_udp_height < 64 || frame_udp_quality < 1 || frame_udp_quality > 100 ||
         frame_udp_stride < 1))
    {
        spdlog::error("Invalid frame UDP configuration");
        return 1;
    }

    bool gesture_udp_enabled = false;
    std::string gesture_udp_host = motion_udp_host;
    int gesture_udp_port = 39543;
    head_pat::Settings head_pat_settings;
    // Region detectors for the other two interactions.  Both share their
    // enabled/host/port with the block above and only carry their own geometry.
    gesture_region::Settings touch_face_settings;
    gesture_region::Settings hug_settings;
    touch_face_settings.id = gesture_udp::ID_TOUCH_FACE;
    touch_face_settings.anchor = gesture_region::Anchor::Nose;
    touch_face_settings.region_above = 0.1;
    touch_face_settings.region_below = 0.9;
    touch_face_settings.region_half_width = 1.0;
    touch_face_settings.min_hands = 2;
    touch_face_settings.require_both_sides = true;
    touch_face_settings.dwell_seconds = 0.4;
    // Also accept pose wrists in the neck/chest box when finger landmarks drop out.
    gesture_region::Settings touch_face_pose_settings;
    touch_face_pose_settings.id = gesture_udp::ID_TOUCH_FACE;
    touch_face_pose_settings.anchor = gesture_region::Anchor::Chest;
    touch_face_pose_settings.region_above = 0.55;
    touch_face_pose_settings.region_below = 0.35;
    touch_face_pose_settings.region_half_width = 0.60;
    touch_face_pose_settings.min_hands = 2;
    touch_face_pose_settings.require_both_sides = true;
    touch_face_pose_settings.dwell_seconds = 0.4;
    touch_face_pose_settings.grace_seconds = 0.35;
    hug_settings.id = gesture_udp::ID_HUG;
    hug_settings.anchor = gesture_region::Anchor::Chest;
    hug_settings.use_face_nose = false;
    // Open-armed hug: both wrists roughly level with the shoulders (a narrow
    // vertical band) and clearly away from the centre line, one on each side.
    // The inner ring is what keeps clasped or crossed hands from matching.
    hug_settings.region_above = 0.15;
    hug_settings.region_below = 0.20;
    hug_settings.region_inner_half_width = 0.45;
    hug_settings.region_half_width = 2.50;
    hug_settings.min_hands = 2;
    hug_settings.require_both_sides = true;
    hug_settings.dwell_seconds = 0.5;
    hug_settings.grace_seconds = 0.40;
    bool touch_face_enabled = true;
    bool hug_enabled = true;
    bool head_pat_enabled = true;
    if (config["pipeline"] && config["pipeline"]["gesture_udp"])
    {
        const auto &udp = config["pipeline"]["gesture_udp"];
        if (udp["enabled"]) gesture_udp_enabled = udp["enabled"].as<bool>();
        if (udp["host"]) gesture_udp_host = udp["host"].as<std::string>();
        if (udp["port"]) gesture_udp_port = udp["port"].as<int>();
        if (udp["dwell_seconds"]) head_pat_settings.dwell_seconds = udp["dwell_seconds"].as<double>();
        if (udp["region_above"]) head_pat_settings.region_above = udp["region_above"].as<double>();
        if (udp["region_below"]) head_pat_settings.region_below = udp["region_below"].as<double>();
        if (udp["region_half_width"]) head_pat_settings.region_half_width = udp["region_half_width"].as<double>();
        if (udp["min_confidence"]) head_pat_settings.min_confidence = udp["min_confidence"].as<double>();

        // Every region key is optional; anything omitted keeps the default
        // above, so a config written before these gestures existed still runs.
        auto apply_region = [&](const YAML::Node &node, gesture_region::Settings &settings) {
            if (!node) return;
            if (node["enabled"]) settings.enabled = node["enabled"].as<bool>();
            if (node["dwell_seconds"]) settings.dwell_seconds = node["dwell_seconds"].as<double>();
            if (node["grace_seconds"]) settings.grace_seconds = node["grace_seconds"].as<double>();
            if (node["release_seconds"]) settings.release_seconds = node["release_seconds"].as<double>();
            if (node["cooldown_seconds"]) settings.cooldown_seconds = node["cooldown_seconds"].as<double>();
            if (node["region_above"]) settings.region_above = node["region_above"].as<double>();
            if (node["region_below"]) settings.region_below = node["region_below"].as<double>();
            if (node["region_half_width"]) settings.region_half_width = node["region_half_width"].as<double>();
            if (node["region_inner_half_width"]) settings.region_inner_half_width = node["region_inner_half_width"].as<double>();
            if (node["min_hands"]) settings.min_hands = node["min_hands"].as<int>();
            if (node["require_both_sides"]) settings.require_both_sides = node["require_both_sides"].as<bool>();
            if (node["whole_hand"]) settings.whole_hand = node["whole_hand"].as<bool>();
            if (node["min_confidence"]) settings.min_confidence = node["min_confidence"].as<double>();
            if (node["anchor_cache_seconds"]) settings.anchor_cache_seconds = node["anchor_cache_seconds"].as<double>();
        };
        apply_region(udp["touch_face"], touch_face_settings);
        apply_region(udp["touch_face_pose"], touch_face_pose_settings);
        apply_region(udp["hug"], hug_settings);
        if (udp["touch_face"] && udp["touch_face"]["enabled"])
            touch_face_enabled = udp["touch_face"]["enabled"].as<bool>();
        if (udp["hug"] && udp["hug"]["enabled"])
            hug_enabled = udp["hug"]["enabled"].as<bool>();
        if (udp["head_pat"] && udp["head_pat"]["enabled"])
            head_pat_enabled = udp["head_pat"]["enabled"].as<bool>();
    }
    if (gesture_udp_enabled &&
        (gesture_udp_port < 1 || gesture_udp_port > 65535 ||
         (motion_udp_enabled && gesture_udp_port == motion_udp_port) ||
         (finger_udp_enabled && gesture_udp_port == finger_udp_port) ||
         (frame_udp_enabled && gesture_udp_port == frame_udp_port)))
    {
        spdlog::error("Invalid or conflicting gesture UDP port: {}", gesture_udp_port);
        return 1;
    }
    if (gesture_udp_enabled && !motion_udp_enabled)
    {
        spdlog::error("Gesture detection requires motion_udp to select the controller person");
        return 1;
    }

    // ---------------------------- 正式开始配置pipeline和加载网络 -----------------------------------
    // 1. FPAI设备初始化
    auto device = Device::Open(fpai_cfg.device_url.c_str());
    auto fpai_device = device.cast<FPAIDevice>();
#if defined(USE_BUYI_BACKEND)
    fpai_device.mmuModeSwitch(fpai_cfg.mmu_mode); // 关闭MMU
#endif

    // ---- HWPRE preflight ----
#if defined(USE_ZG330_BACKEND)
    {
        spdlog::info("[HWPRE][PREFLIGHT] checking VERSION + STATUS...");
        auto &reg = fpai_device.defaultRegRegion();
        uint32_t ver  = reg.read(HWPRE_REG_BASE + 0x18, false);
        uint32_t stat = reg.read(HWPRE_REG_BASE + 0x04, false);
        spdlog::info("[HWPRE][PREFLIGHT] VERSION=0x{:08x} STATUS=0x{:08x}", ver, stat);
        if (ver != HWPRE_PIPELINE_VERSION)
        {
            spdlog::error("[FATAL][PREFLIGHT] VERSION mismatch: got=0x{:08x} expected=0x{:08x}. Abort.",
                          ver, HWPRE_PIPELINE_VERSION);
            return 5;
        }
        if (stat & 0x2)
        {
            spdlog::error("[FATAL][PREFLIGHT] HWPRE busy=1. Abort.");
            return 6;
        }
        if (stat & 0x1)
        {
            spdlog::warn("[HWPRE][PREFLIGHT] stale DONE latch, clearing...");
            reg.write(HWPRE_REG_BASE + 0x00, 0x4, false);
            stat = reg.read(HWPRE_REG_BASE + 0x04, false);
            if (stat & 0x2)
            {
                spdlog::error("[FATAL][PREFLIGHT] still busy after clearing DONE. Abort.");
                return 6;
            }
        }
        reg.write(HWPRE_REG_BASE + HWPRE_ARB_CFG_OFFSET,
                  static_cast<uint32_t>(arb_m0_slots), false);
        uint32_t arb_readback =
            reg.read(HWPRE_REG_BASE + HWPRE_ARB_CFG_OFFSET, false) & 0xFFu;
        if (arb_readback != static_cast<uint32_t>(arb_m0_slots))
        {
            spdlog::error("[FATAL][PREFLIGHT] ARB_CFG readback mismatch: wrote={} read={}. Abort.",
                          arb_m0_slots, arb_readback);
            return 7;
        }
        spdlog::info("[HWPRE][PREFLIGHT] arbitration configured: m0_slots={}", arb_readback);
        spdlog::info("[HWPRE][PREFLIGHT] OK — idle, version correct.");
    }
#endif
    // 多个摄像头输入，多个寄存器组
    std::vector<uint64_t> sdicamera_base_addr_group = {
        0x40080000,
        0x40080400,
        0x40080800,
        0x40080C00};
    std::vector<uint64_t> image_make_base_addr_group = {
        0x80000400,
        0x80040000,
        0x80040400,
        0x80040800};

    // 2. 构建 ICORE Actor 的输入、输出队列

    ThreadSafeQueue<InputMessageForIcore> icore_input_queue(BUFFER_COUNT); // SDI Camera -> Icore
    ThreadSafeQueue<IcoreMessageForPost> icore_output_queue(BUFFER_COUNT); // Icore -> HDMI Display

    // 3. 使用 std::vector 存储 Actors 和相关对象
    // XXXActor 这类包含线程、锁或引用的对象是不可移动、不可拷贝的。
    // 不能将它们的对象本身直接存储在 std::vector 中, 必须将它们也改为存储 std::unique_ptr
    std::unique_ptr<SDICameraInputActor<FPAIDevice, FPAIBackend>> camera_actor;

    BufferManager buffer_manager(BUFFER_COUNT);

    // 6. 配置NPU Actor
    int icore_id = 0;
    MultiYolov5IcoreActor<FPAIDevice, FPAIBackend> icore_actor(icore_id,
                                                               fpai_device,
                                                               buffer_manager,
                                                               CAMERA_COUNT,
                                                               pose_cfg,
                                                               fpai_cfg,
                                                               face_cfg,
                                                               finger_cfg);
    icore_actor.bindInputQueue(&icore_input_queue);
    icore_actor.bindOutputQueue(&icore_output_queue);
    // ----------------- 为每个流创建独立的后处理状态 -----------------
    // Define the YOLOv5 post-processing function as a lambda.
    std::vector<YoloPostResult> last_results_group(CAMERA_COUNT); // Store last results for each camera
    std::vector<YoloPostResult> last_face_results_group(CAMERA_COUNT);
    std::vector<YoloPostResult> last_finger_results_group(CAMERA_COUNT);
    auto netinfos = icore_actor.getNetInfos();
    auto face_netinfos = icore_actor.getFaceNetInfos();
    auto finger_netinfos = icore_actor.getFingerNetInfos();
    // hwpre 输出形状必须与模型输入一致 (bit 参数: RESIZE 626x352 + PAD 7 → 640x352)
    if (netinfos[0].i_cubic[0].h != HWPRE_OUT_H ||
        netinfos[0].i_cubic[0].w != HWPRE_OUT_W ||
        netinfos[0].i_cubic[0].c != HWPRE_OUT_C)
    {
        spdlog::error("[FATAL] model input shape mismatch: expected {}x{}x{} (hwpre output), got {}x{}x{}. Abort.",
                      HWPRE_OUT_H, HWPRE_OUT_W, HWPRE_OUT_C,
                      netinfos[0].i_cubic[0].h, netinfos[0].i_cubic[0].w, netinfos[0].i_cubic[0].c);
        return 2;
    }
    spdlog::info("[SHAPE-OK] model input H={} W={} C={}", HWPRE_OUT_H, HWPRE_OUT_W, HWPRE_OUT_C);
    const int bbox_info_channel = 64;
    int NOA = pose_cfg.ANCHORS.empty() ? 1 : static_cast<int>(pose_cfg.ANCHORS[0].size());
    const std::vector<int> ori_out_channles = {pose_cfg.N_CLASS, bbox_info_channel, NOP * 3};
    std::vector<int> real_out_channles = _getAnchorLength(
        std::vector<int>(ori_out_channles), netinfos[0].detpost_bit, NOA).second;
    if (face_netinfos[0].i_cubic[0].h != HWPRE_OUT_H ||
        face_netinfos[0].i_cubic[0].w != HWPRE_OUT_W ||
        face_netinfos[0].i_cubic[0].c != HWPRE_OUT_C)
    {
        spdlog::error("[FATAL] yolov8-face input shape mismatch: expected {}x{}x{}, got {}x{}x{}",
                      HWPRE_OUT_H, HWPRE_OUT_W, HWPRE_OUT_C,
                      face_netinfos[0].i_cubic[0].h, face_netinfos[0].i_cubic[0].w,
                      face_netinfos[0].i_cubic[0].c);
        return 2;
    }
    constexpr int FACE_NOP = 51;
    constexpr int FACE_CLASS = 1;
    const int face_bbox_info_channel = 64;
    const std::vector<int> face_out_channels = {FACE_CLASS, face_bbox_info_channel, FACE_NOP * 3};
    const int face_noa = face_cfg.ANCHORS.empty() ? 1 : static_cast<int>(face_cfg.ANCHORS[0].size());
    std::vector<int> face_real_out_channels = _getAnchorLength(
        std::vector<int>(face_out_channels), face_netinfos[0].detpost_bit, face_noa).second;
    if (finger_cfg.NET_W != HWPRE_OUT_W || finger_cfg.NET_H != HWPRE_OUT_H ||
        finger_netinfos[0].i_cubic[0].h != HWPRE_OUT_H ||
        finger_netinfos[0].i_cubic[0].w != HWPRE_OUT_W ||
        finger_netinfos[0].i_cubic[0].c != HWPRE_OUT_C || !finger_netinfos[0].DetPost_on)
    {
        spdlog::error("[FATAL] yolov8-finger requires the validated 640x352x3 channel64 DetPost model");
        return 2;
    }
    constexpr int FINGER_NOP = 21;
    constexpr int FINGER_CLASS = 1;
    const int finger_bbox_info_channel = 64;
    std::vector<int> finger_real_out_channels = _getAnchorLength(
        std::vector<int>{FINGER_CLASS, finger_bbox_info_channel, FINGER_NOP * 3},
        finger_netinfos[0].detpost_bit, 1).second;
    spdlog::info("[TRINET] same-frame serial sessions: pose(17) -> face(51) -> finger(21), HWPre once");
    std::vector<FPSCalculator> fps_calculators(CAMERA_COUNT); // 为每个流创建一个FPS计算器

    // 7. 配置单路流水线的输入和输出
    int i = 0;
    // 配置摄像头输入，为每个编码器创建独立的配置副本
    // 配置摄像头
    int camera_id = i;
    // hwpre 路径: PL 端 cmos_resize 必须直通 (net=1920x1080), 全帧送 hwpre 做预处理;
    // 若配成 NET 尺寸 (如 640x352), cmos_resize 会抽点, hwpre 按 1920x1080 计数收不满帧 → 饿死 timeout
    int cam_pl_w = CAMERA_W;
    int cam_pl_h = CAMERA_H;
    auto camera = std::make_unique<GenericSDICamera>(camera_id, fpai_device, CAMERA_W, CAMERA_H, camera_fmt::RGB565,
                                                     FRAME_W, FRAME_H, camera_fmt::RGB565, crop_position::center, // RGB565格式用于hdmi显示
                                                     cam_pl_w, cam_pl_h, crop_position::center,   // PL端输入必须经过hardResizePL
                                                     sdicamera_base_addr_group[i], VTC_ON);
#if defined(USE_BUYI_BACKEND)
    camera_actor = std::make_unique<SDICameraInputActor<FPAIDevice, FPAIBackend>>(
        camera->getSourceId(),
        std::move(camera),
        fpai_device,
        buffer_manager,
        icore_actor.getImkSessionGroups()[i],
        image_make_base_addr_group[i]);
#elif defined(USE_ZG330_BACKEND)
    try
    {
        camera_actor = std::make_unique<SDICameraInputActor<FPAIDevice, FPAIBackend>>(
            camera->getSourceId(),
            std::move(camera),
            fpai_device,
            buffer_manager,
            icore_actor.getIcoreSessionGroups()[i], // pass ref of whole icore sessions
            0,                // imagemake_reg_base: 禁用 ImageMake (USE_HWPRE=1 的 bit 无 ImageMake 视频流)
            HWPRE_REG_BASE,   // hwpre 寄存器基址 → 启用 hwpre 预处理路径
            &icore_actor.getFaceSessionGroup(i),
            &icore_actor.getFingerSessionGroup(i));
    }
    catch (const std::exception &e)
    {
        spdlog::error("[FATAL] hwpre init failed: {}. Abort.", e.what());
        return 3;
    }
#endif
    camera_actor->bindOutputQueue(&icore_input_queue);

    // hwpre ↔ NPU forward 共享 mutex: 防止 NPU 读 PLDDR 与 hwpre 写 PLDDR 竞争
    std::mutex hwpre_mutex;
    camera_actor->setHwpreMutex(&hwpre_mutex);
    icore_actor.setHwpreMutex(&hwpre_mutex);
    camera_actor->setHwpreDebug(&hwpre_dbg, &hwpre_stats);
    std::atomic<bool> g_runtime_error{false};
    camera_actor->setRuntimeErrorFlag(&g_runtime_error);

    std::unique_ptr<motion_udp::Sender> motion_sender;
    uint32_t motion_sequence = 0;
    bytetrack::TrackingConfig pose_tracking_config;
    pose_tracking_config.track_high_thresh = pose_cfg.CONF;
    pose_tracking_config.new_track_thresh = pose_cfg.CONF;
    pose_tracking_config.track_low_thresh = std::min(0.1f, pose_cfg.CONF);
    pose_tracking_config.track_buffer = 30;
    bytetrack::BYTETracker pose_tracker(pose_tracking_config);
    int pose_tracking_frame = 0;
    PrimaryTrackSelector primary_track_selector(15);
    if (motion_udp_enabled)
    {
        motion_sender = std::make_unique<motion_udp::Sender>(motion_udp_host, static_cast<uint16_t>(motion_udp_port));
        if (!motion_sender->ready())
        {
            spdlog::error("Motion UDP disabled: {}", motion_sender->lastError());
            motion_sender.reset();
        }
        else
        {
            spdlog::info("Motion UDP target: {}:{} (DMV2 + ByteTrack person IDs)", motion_udp_host, motion_udp_port);
        }
    }

    std::unique_ptr<finger_udp::Sender> finger_sender;
    if (finger_udp_enabled)
    {
        finger_sender = std::make_unique<finger_udp::Sender>(finger_udp_host, static_cast<uint16_t>(finger_udp_port));
        if (!finger_sender->ready())
        {
            spdlog::error("Finger UDP disabled: {}", finger_sender->lastError());
            finger_sender.reset();
        }
        else spdlog::info("Finger UDP target: {}:{} (FNG1, same sequence/capture timestamp as DMV1)",
                          finger_udp_host, finger_udp_port);
    }

    std::unique_ptr<frame_udp::Sender> frame_sender;
    uint32_t frame_sequence = 0;
    uint64_t frame_post_count = 0;
    if (frame_udp_enabled)
    {
        frame_sender = std::make_unique<frame_udp::Sender>(
            frame_udp_host, static_cast<uint16_t>(frame_udp_port),
            frame_udp_width, frame_udp_height, frame_udp_quality);
        if (!frame_sender->ready())
        {
            spdlog::error("Frame UDP disabled: {}", frame_sender->lastError());
            frame_sender.reset();
        }
        else
        {
            spdlog::info("Frame UDP target: {}:{} {}x{} quality={} stride={}",
                         frame_udp_host, frame_udp_port, frame_udp_width, frame_udp_height,
                         frame_udp_quality, frame_udp_stride);
        }
    }

    // Human-readable list of the gestures this run publishes, for the startup log.
    auto gesture_list_summary = [](bool head_pat, bool touch_face, bool hug) {
        std::string text;
        auto add = [&text](const char *name) {
            if (!text.empty()) text += " + ";
            text += name;
        };
        if (head_pat) add("touch_head");
        if (touch_face) add("touch_face");
        if (hug) add("hug");
        return text.empty() ? std::string("none") : text;
    };

    std::unique_ptr<gesture_udp::Sender> gesture_sender;
    std::unique_ptr<head_pat::Trigger> head_pat_trigger;
    std::unique_ptr<gesture_region::Detector> touch_face_detector;
    std::unique_ptr<gesture_region::Detector> touch_face_pose_detector;
    std::unique_ptr<gesture_region::Detector> hug_detector;
    uint32_t gesture_sequence = 0;
    if (gesture_udp_enabled)
    {
        gesture_sender = std::make_unique<gesture_udp::Sender>(
            gesture_udp_host, static_cast<uint16_t>(gesture_udp_port));
        if (!gesture_sender->ready())
        {
            spdlog::error("Gesture UDP disabled: {}", gesture_sender->lastError());
            gesture_sender.reset();
        }
        else
        {
            if (head_pat_enabled)
                head_pat_trigger = std::make_unique<head_pat::Trigger>(head_pat_settings);
            if (touch_face_enabled && touch_face_settings.enabled)
                touch_face_detector = std::make_unique<gesture_region::Detector>(touch_face_settings);
            if (touch_face_enabled && touch_face_pose_settings.enabled)
                touch_face_pose_detector = std::make_unique<gesture_region::Detector>(touch_face_pose_settings);
            if (hug_enabled && hug_settings.enabled)
                hug_detector = std::make_unique<gesture_region::Detector>(hug_settings);
            spdlog::info("Gesture UDP target: {}:{} (GST1, {} on PS)",
                         gesture_udp_host, gesture_udp_port,
                         gesture_list_summary(head_pat_trigger != nullptr,
                                              touch_face_detector != nullptr,
                                              hug_detector != nullptr));
        }
    }

    // 创建用于yolov5的后处理函数“适配器”lambda， 签名完美匹配 PostProcessingFunc
    HDMIDisplayActor<FPAIDevice, IcoreMessageForPost>::PostProcessingFunc pose_post_processor =
        // 捕获所有“实现”需要的额外变量
        // 注意：这里的参数列表必须与 PostProcessingFunc 定义完全匹配
        // 使用[&]可以简化捕获列表
        [&](const IcoreMessageForPost &post_msg, cv::Mat &cvmat_to_draw)
    {
        auto ts_start = std::chrono::steady_clock::now();
        if (post_msg.icore_tensors.empty() || post_msg.secondary_icore_tensors.empty() ||
            post_msg.tertiary_icore_tensors.empty())
        {
            //LOG_WARN("[PostProcessor]", "Icore tensors are empty, cannot perform post-processing.");
            return;
        }
        // 从消息中获取source_id，这是查找正确上下文的关键
        int source_id = post_msg.meta.source_id;

        // 检查source_id的有效性，防止越界
        if (source_id < 0 || source_id >= static_cast<int>(CAMERA_COUNT))
        {
            //LOG_ERROR("[PostProcessor]", "Invalid source_id {} received.", source_id);
            return;
        }
        // 调用真正的实现，并传入所有捕获的参数
        auto &netinfo = netinfos[source_id];
        auto &last_results = last_results_group[source_id];
        auto &fps_calc = fps_calculators[source_id];

        fps_calc.tick();
        //LOG_DEBUG("[AI POST]", "[{}] Before post_detpost_plin, last_results size: {}", source_id, std::get<0>(last_results).size());
        YoloPostResult post_results;
        auto ts_before_post = std::chrono::steady_clock::now();
        auto ts_before_duration = std::chrono::duration_cast<std::chrono::microseconds>(ts_before_post - ts_start);
        post_results = post_detpost_plin(
            post_msg.icore_tensors, last_results, netinfo,
            pose_cfg.CONF, pose_cfg.IOU_THRESHOLD, pose_cfg.MULTILABEL,
            pose_cfg.FPGA_NMS, pose_cfg.N_CLASS, NOP, pose_cfg.ANCHORS,
            device, real_out_channles, bbox_info_channel);

        // 同一帧的 yolov8-face 输出：与姿态模型串行完成，不启动第二条摄像头/HWPre 通路。
        YoloPostResult face_results = post_detpost_plin(
            post_msg.secondary_icore_tensors,
            last_face_results_group[source_id], face_netinfos[source_id],
            face_cfg.CONF, face_cfg.IOU_THRESHOLD, face_cfg.MULTILABEL,
            face_cfg.FPGA_NMS, FACE_CLASS, FACE_NOP, face_cfg.ANCHORS,
            device, face_real_out_channels, face_bbox_info_channel);
        if (!SAVE)
            last_face_results_group[source_id] = face_results;
        const auto face_ids = std::get<0>(face_results);
        const auto face_scores = std::get<1>(face_results);
        const auto face_boxes = std::get<2>(face_results);
        const auto &face_keypoints = std::get<3>(face_results);
        YoloPostResult finger_results = finger_postprocess::post_detpost_plin(
            post_msg.tertiary_icore_tensors,
            last_finger_results_group[source_id], finger_netinfos[source_id],
            finger_cfg.CONF, finger_cfg.IOU_THRESHOLD, finger_cfg.MULTILABEL,
            finger_cfg.FPGA_NMS, FINGER_CLASS, FINGER_NOP, finger_cfg.ANCHORS,
            device, finger_real_out_channels, finger_bbox_info_channel);
        if (!SAVE) last_finger_results_group[source_id] = finger_results;
        const auto &finger_scores = std::get<1>(finger_results);
        const auto &finger_boxes = std::get<2>(finger_results);
        const auto &finger_keypoints = std::get<3>(finger_results);
// Update last_results after the call, should be comment if do metric validation for yolov5_npu_actor by loading test images.
        if (!SAVE)
            last_results = post_results;

        std::vector<int> id_list = std::get<0>(post_results);
        std::vector<float> score_list = std::get<1>(post_results);
        std::vector<cv::Rect2f> box_list = std::get<2>(post_results);
        const auto &key_point_list = std::get<3>(post_results);
        const uint32_t current_motion_sequence = motion_sequence++;
        const auto emit_time_ns = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
        const auto capture_time_ns = duration_cast<nanoseconds>(system_clock::duration(post_msg.meta.timestamp)).count();
        std::optional<motion_udp::Detection> controller_body;
        std::optional<motion_udp::Detection> controller_face;
        std::vector<finger_udp::Detection> finger_hands;
        if (finger_sender || head_pat_trigger || touch_face_detector || hug_detector)
            finger_hands = selectFingerDetections(finger_results);
        if (motion_sender)
        {
            static uint64_t send_failures = 0;
            auto send_motion = [&](uint32_t track_id, const motion_udp::Detection *body,
                                   const motion_udp::Detection *face)
            {
                const bool sent = motion_sender->sendFrame(
                    current_motion_sequence,
                    static_cast<uint64_t>(std::max<int64_t>(0, capture_time_ns)),
                    static_cast<uint64_t>(std::max<int64_t>(0, emit_time_ns)),
                    static_cast<uint16_t>(CAMERA_W), static_cast<uint16_t>(CAMERA_H),
                    static_cast<uint8_t>(source_id), body, face, track_id);
                if (!sent && (++send_failures == 1 || send_failures % 120 == 0))
                    spdlog::warn("Motion UDP send failed ({} total): {}", send_failures, motion_sender->lastError());
            };

            std::vector<std::size_t> pose_indices;
            std::vector<bytetrack::Detection> pose_detections;
            for (std::size_t i = 0; i < score_list.size(); ++i)
            {
                if (!motionDetectionAt(score_list, box_list, key_point_list, i, motion_udp::POSE_COUNT))
                    continue;
                pose_indices.push_back(i);
                pose_detections.emplace_back(box_list[i], score_list[i], 0);
            }
            std::vector<std::shared_ptr<bytetrack::STrack>> active_tracks;
            std::vector<bytetrack::TrackAssignment> assignments;
            pose_tracker.update(pose_detections, ++pose_tracking_frame, active_tracks, assignments);
            std::sort(assignments.begin(), assignments.end(), [](const auto &a, const auto &b) {
                return a.track_id < b.track_id;
            });

            std::vector<int> visible_track_ids;
            visible_track_ids.reserve(assignments.size());
            for (const auto &assignment : assignments)
                visible_track_ids.push_back(assignment.track_id);
            const int previous_primary_track_id = primary_track_selector.boundTrackId();
            const int controller_track_id = primary_track_selector.update(visible_track_ids);
            if (primary_track_selector.boundTrackId() != previous_primary_track_id)
                spdlog::info("Primary person rebound: ByteTrack {} -> logical track 1",
                             primary_track_selector.boundTrackId());

            bool controller_sent = false;
            for (const auto &assignment : assignments)
            {
                if (assignment.detection_idx < 0 ||
                    assignment.detection_idx >= static_cast<int>(pose_indices.size()))
                    continue;
                auto body = motionDetectionAt(score_list, box_list, key_point_list,
                                              pose_indices[assignment.detection_idx], motion_udp::POSE_COUNT);
                if (!body)
                    continue;
                const uint32_t track_id = primary_track_selector.logicalTrackId(
                    assignment.track_id, controller_track_id);
                auto face = track_id == 1U
                    ? selectFaceForBody(face_scores, face_boxes, face_keypoints, *body)
                    : std::nullopt;
                if (track_id == 1U)
                {
                    controller_body = *body;
                    controller_face = face;
                }
                send_motion(track_id, &*body, face ? &*face : nullptr);
                controller_sent = controller_sent || assignment.track_id == controller_track_id;
            }
            if (!controller_sent)
                send_motion(1U, nullptr, nullptr);
        }
        if (finger_sender)
        {
            const bool sent = finger_sender->sendFrame(
                current_motion_sequence,
                static_cast<uint64_t>(std::max<int64_t>(0, capture_time_ns)),
                static_cast<uint64_t>(std::max<int64_t>(0, emit_time_ns)),
                static_cast<uint16_t>(CAMERA_W), static_cast<uint16_t>(CAMERA_H),
                static_cast<uint8_t>(source_id), finger_hands);
            static uint64_t failures = 0;
            if (!sent && (++failures == 1 || failures % 120 == 0))
                spdlog::warn("Finger UDP send failed ({} total): {}", failures, finger_sender->lastError());
        }
        if (gesture_sender && (head_pat_trigger || touch_face_detector || hug_detector))
        {
            const double now_seconds =
                duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
            const motion_udp::Detection *body_arg = controller_body ? &*controller_body : nullptr;
            const motion_udp::Detection *face_arg = controller_face ? &*controller_face : nullptr;

            head_pat::State pat;
            gesture_region::State face_state;
            gesture_region::State hug_state;
            const bool pat_present = head_pat_trigger != nullptr;
            std::vector<finger_udp::Detection> pat_hands = finger_hands;
            for (auto &pose_hand : selectPoseWristHands(controller_body))
                pat_hands.push_back(std::move(pose_hand));
            const bool face_present = touch_face_detector != nullptr;
            const bool hug_present = hug_detector != nullptr;
            if (pat_present)
                pat = head_pat_trigger->update(now_seconds, static_cast<float>(CAMERA_W),
                                               static_cast<float>(CAMERA_H), body_arg, face_arg,
                                               pat_hands);
            if (face_present)
                face_state = touch_face_detector->update(now_seconds, static_cast<float>(CAMERA_W),
                                                         static_cast<float>(CAMERA_H), body_arg,
                                                         face_arg, finger_hands);
            if (touch_face_pose_detector != nullptr)
            {
                const auto pose_hands = selectPoseWristHands(controller_body);
                const gesture_region::State pose_face = touch_face_pose_detector->update(
                    now_seconds, static_cast<float>(CAMERA_W), static_cast<float>(CAMERA_H),
                    body_arg, nullptr, pose_hands);
                const bool merged_active = face_state.active || pose_face.active;
                const double merged_progress = std::max(face_state.progress, pose_face.progress);
                if (pose_face.progress > face_state.progress)
                {
                    face_state.outside = pose_face.outside;
                    face_state.has_outside = pose_face.has_outside;
                }
                face_state.active = merged_active;
                face_state.progress = merged_progress;
            }
            if (hug_present)
                hug_state = hug_detector->update(now_seconds, static_cast<float>(CAMERA_W),
                                                 static_cast<float>(CAMERA_H), body_arg, face_arg,
                                                 finger_hands);

            // One interaction at a time.  The same hands can sit in two regions at
            // once (sliding from the crown down onto the cheeks, say), so the most
            // specific gesture wins and the others are still published, but low,
            // which keeps their progress visible for tuning.
            uint16_t winner = 0xFFFFU;
            if (hug_present && hug_state.active)
                winner = gesture_udp::ID_HUG;
            else if (face_present && face_state.active)
                winner = gesture_udp::ID_TOUCH_FACE;
            else if (pat_present && pat.active)
                winner = gesture_udp::ID_HEAD_PAT;

            std::vector<gesture_udp::Record> records;
            records.reserve(3);
            if (pat_present)
            {
                gesture_udp::Record record;
                record.id = gesture_udp::ID_HEAD_PAT;
                record.active = gesturePulse(g_pulse_head, winner == gesture_udp::ID_HEAD_PAT,
                                             now_seconds, 1.6, 0.4) ? 1U : 0U;
                record.flags = pat.has_outside ? gesture_udp::FLAG_METRIC_VALID : 0U;
                record.progress = static_cast<float>(pat.progress);
                record.metric = static_cast<float>(pat.outside);
                records.push_back(record);
            }
            if (face_present)
            {
                gesture_udp::Record record;
                record.id = gesture_udp::ID_TOUCH_FACE;
                record.active = gesturePulse(g_pulse_face, winner == gesture_udp::ID_TOUCH_FACE,
                                             now_seconds, 2.5, 0.4) ? 1U : 0U;
                record.flags = face_state.has_outside ? gesture_udp::FLAG_METRIC_VALID : 0U;
                record.progress = static_cast<float>(face_state.progress);
                record.metric = static_cast<float>(face_state.outside);
                records.push_back(record);
            }
            if (hug_present)
            {
                gesture_udp::Record record;
                record.id = gesture_udp::ID_HUG;
                record.active = gesturePulse(g_pulse_hug, winner == gesture_udp::ID_HUG,
                                             now_seconds, 3.0, 0.4) ? 1U : 0U;
                record.flags = hug_state.has_outside ? gesture_udp::FLAG_METRIC_VALID : 0U;
                record.progress = static_cast<float>(hug_state.progress);
                record.metric = static_cast<float>(hug_state.outside);
                records.push_back(record);
            }

            static uint64_t gesture_failures = 0;
            const bool sent = gesture_sender->sendFrame(
                gesture_sequence++,
                static_cast<uint64_t>(std::max<int64_t>(0, capture_time_ns)),
                static_cast<uint64_t>(std::max<int64_t>(0, emit_time_ns)),
                static_cast<uint16_t>(CAMERA_W), static_cast<uint16_t>(CAMERA_H),
                static_cast<uint8_t>(source_id), records);
            if (!sent && (++gesture_failures == 1 || gesture_failures % 120 == 0))
                spdlog::warn("Gesture UDP send failed ({} total): {}", gesture_failures,
                             gesture_sender->lastError());

            static bool previous_pat = false;
            if (pat_present && pat.active != previous_pat)
            {
                previous_pat = pat.active;
                if (pat.has_outside)
                    spdlog::info("[GESTURE] touch_head {} progress={:.2f} dwell={:.2f}s outside={:.2f} "
                                 "published={} head={:.0f}px nose={}{} hand={}",
                                 pat.active ? "ON " : "OFF", pat.progress, pat.dwell, pat.outside,
                                 winner == gesture_udp::ID_HEAD_PAT ? 1 : 0,
                                 pat.head_width_px, pat.nose_source,
                                 pat.nose_cached ? " (cached)" : "", pat.hand_index);
                else
                    spdlog::info("[GESTURE] touch_head {} progress={:.2f} dwell={:.2f}s outside=none "
                                 "published={} head={:.0f}px nose={}{} hand={}",
                                 pat.active ? "ON " : "OFF", pat.progress, pat.dwell,
                                 winner == gesture_udp::ID_HEAD_PAT ? 1 : 0,
                                 pat.head_width_px, pat.nose_source,
                                 pat.nose_cached ? " (cached)" : "", pat.hand_index);
            }

            auto log_region_edge = [](const char *name, bool present, bool active, bool *previous,
                                      bool published, const gesture_region::State &state) {
                if (!present || active == *previous)
                    return;
                *previous = active;
                if (state.has_outside)
                    spdlog::info("[GESTURE] {} {} progress={:.2f} dwell={:.2f}s outside={:.2f} "
                                 "published={} hands={} sides_ok={} anchor={}{}",
                                 name, active ? "ON " : "OFF", state.progress, state.dwell, state.outside,
                                 published ? 1 : 0, state.hands_inside, state.sides_ok, state.anchor_source,
                                 state.anchor_cached ? " (cached)" : "");
                else
                    spdlog::info("[GESTURE] {} {} progress={:.2f} dwell={:.2f}s outside=none "
                                 "published={} anchor={}{}",
                                 name, active ? "ON " : "OFF", state.progress, state.dwell,
                                 published ? 1 : 0, state.anchor_source,
                                 state.anchor_cached ? " (cached)" : "");
            };
            static bool previous_face = false;
            static bool previous_hug = false;
            log_region_edge("touch_face", face_present, face_state.active, &previous_face,
                            winner == gesture_udp::ID_TOUCH_FACE, face_state);
            log_region_edge("hug", hug_present, hug_state.active, &previous_hug,
                            winner == gesture_udp::ID_HUG, hug_state);
        }
        if (frame_sender && frame_post_count++ % static_cast<uint64_t>(frame_udp_stride) == 0)
        {
            const auto capture_time_ns = duration_cast<nanoseconds>(
                system_clock::duration(post_msg.meta.timestamp)).count();
            const bool sent = frame_sender->sendFrame(
                frame_sequence++, static_cast<uint64_t>(std::max<int64_t>(0, capture_time_ns)),
                static_cast<uint8_t>(source_id), cvmat_to_draw);
            static uint64_t frame_send_failures = 0;
            if (!sent && (++frame_send_failures == 1 || frame_send_failures % 120 == 0))
                spdlog::warn("Frame UDP send failed ({} total): {}",
                             frame_send_failures, frame_sender->lastError());
        }
        //LOG_DEBUG("[AI POST]", "[{}] After post_detpost_plin, last_results size: {}", source_id, std::get<0>(last_results).size());
        /*
         * user prompt: 定义一个lambda函数，将检测结果以yolo格式保存成txt，
         * 然后利用post_msg.meta.timestamp转成000000000139.jpg这样的名字，保存成txt。输出路径是./io/output/coco_val2017
         */
        if (SAVE)
        {
            auto save_yolo_result = [&](const std::vector<int> &ids, const std::vector<float> &scores, const std::vector<cv::Rect2f> &boxes)
            {
                std::string output_dir = "./io/output/coco_val2017";
                if (!fs::exists(output_dir))
                {
                    fs::create_directories(output_dir);
                }
                std::string filename;
                int org_width = CAMERA_W;
                int org_height = CAMERA_H;
                int net_width = pose_cfg.NET_W;
                int net_height = pose_cfg.NET_H;
                if (post_msg.image_name == "")
                {
                    filename = fmt::format("{}/{}.txt", output_dir, post_msg.meta.timestamp);
                }
                else
                {
                    org_width = post_msg.org_width;
                    org_height = post_msg.org_height;
                    net_width = post_msg.net_width;
                    net_height = post_msg.net_height;
                    filename = fmt::format("{}/{}.txt", output_dir, post_msg.image_name);
                }
                LOG_INFO("[TEST]", "Save YOLO result to {}", filename);
                std::ofstream outfile(filename, std::ios::out | std::ios::trunc);

                // 预处理是 hwpre letterbox (1920x1080 → bilinear 626x352 → 左右各 pad 7), 按实际几何还原
                if (outfile.is_open())
                {
                    std::cout << "Original image size: [" << org_width << "x" << org_height << "], Net input size: [" << net_width << "x" << net_height << "]" << std::endl;
                    for (size_t k = 0; k < boxes.size(); ++k)
                    {
                        // 还原到原始图像尺度: x_orig = (x_net - 7) * 1920/626, y_orig = y_net * 1080/352
                        float left = (boxes[k].x - HWPRE_PAD_X) / HWPRE_SCALE_X;
                        float top = (boxes[k].y - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                        float width = boxes[k].width / HWPRE_SCALE_X;
                        float height = boxes[k].height / HWPRE_SCALE_Y;
                        outfile << ids[k] << " " << left << " " << top << " " << width << " " << height << " " << scores[k] << "\n";
                    }
                    outfile.close();
                }
            };
            save_yolo_result(id_list, score_list, box_list);
        }
        auto ts_after_post = std::chrono::steady_clock::now();
        auto ts_after_duration = std::chrono::duration_cast<std::chrono::microseconds>(ts_after_post - ts_before_post);
        if (DRAW)
        {
            for (int index = 0; index < box_list.size(); ++index)
            {
                // hwpre letterbox 还原: net 坐标 (640x352) → 原始 1920x1080
                // x_orig = (x_net - 7) * 1920/626, y_orig = y_net * 1080/352
                float x1 = (box_list[index].tl().x - HWPRE_PAD_X) / HWPRE_SCALE_X;
                float y1 = (box_list[index].tl().y - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                float w = box_list[index].width / HWPRE_SCALE_X;
                float h = box_list[index].height / HWPRE_SCALE_Y;
                int id = id_list[index];
                cv::Scalar color = classColor(id);
                double font_scale = 1;
                int thickness = 3;
                cv::rectangle(cvmat_to_draw, cv::Rect2f(x1, y1, w, h), color, thickness, cv::LINE_8, 0);
                // std::cout << "Draw box(xywh): [" << x1 << "," << y1 << "," << w << "," << h << "] for class " << pose_cfg.LABELS[id] << std::endl;
                std::string s = pose_cfg.LABELS[id_list[index]] + ":" + std::to_string(int(round(score_list[index] * 100))) + "%";
                cv::Size s_size = cv::getTextSize(s, cv::FONT_HERSHEY_COMPLEX, font_scale, 1, 0);
                cv::putText(cvmat_to_draw, s, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_DUPLEX, font_scale, cv::Scalar(255, 255, 255), 1); // 重要！thickness一定设置为1，否则绘图很慢
            }
        }

        if (DRAW)
        {
            for (size_t index = 0; index < face_boxes.size(); ++index)
            {
                float x1 = (face_boxes[index].tl().x - HWPRE_PAD_X) / HWPRE_SCALE_X;
                float y1 = (face_boxes[index].tl().y - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                float w = face_boxes[index].width / HWPRE_SCALE_X;
                float h = face_boxes[index].height / HWPRE_SCALE_Y;
                cv::rectangle(cvmat_to_draw, cv::Rect2f(x1, y1, w, h),
                              cv::Scalar(255, 0, 255), 2, cv::LINE_8, 0);
                cv::putText(cvmat_to_draw,
                            fmt::format("yolov8-face:{:.0f}%", face_scores[index] * 100.0f),
                            cv::Point(static_cast<int>(x1), static_cast<int>(y1) - 5),
                            cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 0, 255), 1);
            }
            for (const auto &one_face_kpt : face_keypoints)
            {
                for (const auto &kpt : one_face_kpt)
                {
                    if (kpt.size() < 3 || kpt[2] <= 0.5f)
                        continue;
                    float x = (kpt[0] - HWPRE_PAD_X) / HWPRE_SCALE_X;
                    float y = (kpt[1] - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                    cv::circle(cvmat_to_draw,
                               cv::Point(static_cast<int>(x), static_cast<int>(y)),
                               2, cv::Scalar(255, 0, 255), -1);
                }
            }
        }
        if (DRAW)
        {
            for (const auto &oneobj_kpt : key_point_list)
            {
                for (const auto &kpt : oneobj_kpt)
                {
                    if (kpt.size() < 3 || kpt[2] <= 0.5f)
                        continue;
                    // hwpre letterbox 还原关键点: (x_net - 7) * 1920/626, y_net * 1080/352
                    float x = (kpt[0] - HWPRE_PAD_X) / HWPRE_SCALE_X;
                    float y = (kpt[1] - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                    cv::circle(cvmat_to_draw, cv::Point(static_cast<int>(x), static_cast<int>(y)),
                               4, cv::Scalar(0, 255, 0), -1);
                }
                for (const auto &sk : kCocoSkeleton)
                {
                    const int a = sk[0] - 1;
                    const int b = sk[1] - 1;
                    if (a < 0 || b < 0 || a >= static_cast<int>(oneobj_kpt.size()) ||
                        b >= static_cast<int>(oneobj_kpt.size()) || oneobj_kpt[a].size() < 3 ||
                        oneobj_kpt[b].size() < 3 || oneobj_kpt[a][2] <= 0.5f || oneobj_kpt[b][2] <= 0.5f)
                        continue;
                    float ax = (oneobj_kpt[a][0] - HWPRE_PAD_X) / HWPRE_SCALE_X;
                    float ay = (oneobj_kpt[a][1] - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                    float bx = (oneobj_kpt[b][0] - HWPRE_PAD_X) / HWPRE_SCALE_X;
                    float by = (oneobj_kpt[b][1] - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                    cv::line(cvmat_to_draw,
                             cv::Point(static_cast<int>(ax), static_cast<int>(ay)),
                             cv::Point(static_cast<int>(bx), static_cast<int>(by)),
                             cv::Scalar(0, 255, 255), 2);
                }
            }
        }

        if (DRAW)
        {
            for (size_t index = 0; index < finger_boxes.size(); ++index)
            {
                const auto &b = finger_boxes[index];
                const float x = (b.x - HWPRE_PAD_X) / HWPRE_SCALE_X;
                const float y = (b.y - HWPRE_PAD_Y) / HWPRE_SCALE_Y;
                cv::rectangle(cvmat_to_draw,
                              cv::Rect2f(x, y, b.width / HWPRE_SCALE_X, b.height / HWPRE_SCALE_Y),
                              cv::Scalar(0, 200, 255), 2);
                cv::putText(cvmat_to_draw, fmt::format("yolov8-finger:{:.0f}%", finger_scores[index] * 100.0f),
                            cv::Point(static_cast<int>(x), static_cast<int>(y) - 5),
                            cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0, 200, 255), 1);
            }
            for (const auto &hand : finger_keypoints)
            {
                auto visible = [&](int k) {
                    return k >= 0 && k < static_cast<int>(hand.size()) && hand[k].size() >= 3 &&
                           hand[k][2] > 0.5f && std::isfinite(hand[k][0]) && std::isfinite(hand[k][1]);
                };
                auto point = [&](int k) {
                    return cv::Point(static_cast<int>((hand[k][0] - HWPRE_PAD_X) / HWPRE_SCALE_X),
                                     static_cast<int>((hand[k][1] - HWPRE_PAD_Y) / HWPRE_SCALE_Y));
                };
                for (int k = 0; k < static_cast<int>(hand.size()); ++k)
                    if (visible(k)) cv::circle(cvmat_to_draw, point(k), 3, cv::Scalar(0, 200, 255), -1);
                for (const auto &edge : finger_postprocess::kHandSkeleton)
                    if (visible(edge[0]) && visible(edge[1]))
                        cv::line(cvmat_to_draw, point(edge[0]), point(edge[1]), cv::Scalar(255, 200, 0), 2);
            }
        }

        auto bgr_color = cv::Scalar(0, 0, 0);
        drawTextOnTwoCorners(cvmat_to_draw, fmt::format("FPS: {:.1f}", fps_calc.getFPS()), DEMO_NAME, bgr_color);
        // DEBUG code, should be removed later
        // cv::imwrite("output_source_" + std::to_string(source_id) + "_" + get_timestamp_string() + ".jpg", cvmat_to_draw);
        auto ts_draw_end = std::chrono::steady_clock::now();
        auto ts_draw_duration = std::chrono::duration_cast<std::chrono::microseconds>(ts_draw_end - ts_after_post);
        auto ts_total = std::chrono::duration_cast<std::chrono::microseconds>(ts_draw_end - ts_start);
        //LOG_DEBUG("[AI POST]", "[{}] Post-processing time: total={}ms (before_post={}ms, after_post={}ms, draw={}ms)",
        //          source_id,
        //          ts_total.count() / 1000.0f,
        //          ts_before_duration.count() / 1000.0f,
        //          ts_after_duration.count() / 1000.0f,
        //          ts_draw_duration.count() / 1000.0f);
    };

    // 配置HDMI输出
    int hdmi_id = 0;
    auto hdmi_display = std::make_unique<RGB565HDMIDisplay<FPAIDevice>>(hdmi_id, fpai_device, FRAME_W, FRAME_H);

    HDMIDisplayActor<FPAIDevice, IcoreMessageForPost> display_actor(hdmi_display->getSinkId(),
                                                                    std::move(hdmi_display),
                                                                    fpai_device,
                                                                    buffer_manager,
                                                                    camera_actor->getChunkGroupId(),
                                                                    pose_post_processor);
    display_actor.bindInputQueue(&icore_output_queue);

    // 9. 启动所有Actors

    // Keep initialization diagnostics, but suppress per-frame SDK info/debug once streaming starts.
    spdlog::set_level(spdlog::level::warn);
    camera_actor->start();
    display_actor.start();
    icore_actor.start();

    spdlog::info("All actors started...");

    while (running)
        std::this_thread::sleep_for(200ms);

    camera_actor->stop();
    icore_actor.stop();
    display_actor.stop();

    spdlog::info("All actors stopped...");
    Device::Close(device);

    return 0;
}
