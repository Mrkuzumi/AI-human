#pragma once

#include "pipeline/actor/base_actors.hpp"
#include "pipeline/base/messages.hpp"
#include "pipeline/base/enums.hpp"
#include "pipeline/memory/buffer_manager.hpp"
#include "pipeline/base/thread_safe_queue.hpp"

#include "NetInfo.hpp"
#include "icraft_utils.hpp"
#include "log_utils.hpp"

#include <icraft-xrt/core/session.h>
#include "compile_fpai_target.hpp"

#include <atomic>
#include <thread>
#include <vector>
#include <array>
#include <memory>
#include <string>
#include <chrono>
#include <cassert>
#include <mutex>
#include <stdexcept>

namespace fpai
{
    struct Yolov5Config
    {
        std::string name = "yolov8-pose";
        float CONF = 0.35;
        float IOU_THRESHOLD = 0.45;
        bool MULTILABEL = false;
        int N_CLASS = 80; // coco 80 classes
        bool FPGA_NMS = true;
        bool DETPOST = true;
        std::vector<std::string> JSON_PATHS;
        std::vector<std::string> RAW_PATHS;
        int NET_W = 640;
        int NET_H = 640;
        int FRAME_W = 1920;
        int FRAME_H = 1080;
        std::string NAMES_PATH = "names/yolov8-pose.names";
        std::vector<std::string> LABELS{};
        crop_position CROP_POS = crop_position::center;
        std::vector<std::vector<std::vector<float>>> ANCHORS = {{{10, 13}, {16, 30}, {33, 23}},
                                                                {{30, 61}, {62, 45}, {59, 119}},
                                                                {{116, 90}, {156, 198}, {373, 326}}};
        std::vector<int64_t> view_except_op_ids = {0, 673, 674, 292, 675, 679, 663};
    };

    void PrintYolov5Config(const Yolov5Config &cfg)
    {
        spdlog::info("{} Configuration:", cfg.name);
        spdlog::info("  Name: {}", cfg.name);
        spdlog::info("  Confidence Threshold: {}", cfg.CONF);
        spdlog::info("  IOU Threshold: {}", cfg.IOU_THRESHOLD);
        spdlog::info("  Multi-label: {}", cfg.MULTILABEL);
        spdlog::info("  Number of Classes: {}", cfg.N_CLASS);
        spdlog::info("  FPGA NMS: {}", cfg.FPGA_NMS);
        spdlog::info("  DETPOST: {}", cfg.DETPOST);
        spdlog::info("  Network Width: {}", cfg.NET_W);
        spdlog::info("  Network Height: {}", cfg.NET_H);
        spdlog::info("  Frame Width: {}", cfg.FRAME_W);
        spdlog::info("  Frame Height: {}", cfg.FRAME_H);
        spdlog::info("  Names Path: {}", cfg.NAMES_PATH);
        spdlog::info("  JSON Paths: ");
        for (const auto &path : cfg.JSON_PATHS)
        {
            spdlog::info("    - {}", path);
        }
        spdlog::info("  RAW Paths: ");
        for (const auto &path : cfg.RAW_PATHS)
        {
            spdlog::info("    - {}", path);
        }
        spdlog::info("  Anchors: ");
        for (const auto &scale : cfg.ANCHORS)
        {
            std::string anchor_str;
            for (const auto &anchor : scale)
            {
                anchor_str += "[" + std::to_string(anchor[0]) + ", " + std::to_string(anchor[1]) + "] ";
            }
            spdlog::info("    - {}", anchor_str);
        }
        spdlog::info("  Crop Position: {}", (cfg.CROP_POS == crop_position::center) ? "center" : "top-left");
        spdlog::info("  View Except Op IDs: ");
        for (const auto &op_id : cfg.view_except_op_ids)
        {
            spdlog::info("    - {}", op_id);
        }
    }

    // 构建多网络NPU Actor
    /**
     * @brief 多路 YOLOv5 NPU 推理 Actor 类
     *
     * 该类继承自 BaseActor，专门用于在 NPU 上执行 YOLOv5 目标检测模型的推理。
     * 它支持同时管理多个网络实例（通过 net_count 指定），处理输入队列中的预处理后数据，
     * 调用底层推理引擎（Session）进行计算，并将推理结果（Tensor）发送到输出队列供后处理使用。
     *
     * 主要功能：
     * - 加载和解析 YOLOv5 模型文件 (JSON/RAW)
     * - 针对特定后端 (Buyi/ZG330) 创建和配置推理 Session
     * - 维护多路网络的上下文和资源
     * - 执行推理主循环 (loop)
     *
     * @tparam DeviceType 硬件设备类型
     * @tparam BackendType 推理后端类型
     */
    template <typename DeviceType, typename BackendType>
    class MultiYolov5IcoreActor : public BaseActor<InputMessageForIcore, IcoreMessageForPost>
    {
    public:
        static constexpr std::string_view LogP = "[ICORE]";
        static constexpr int OPT_DEBUG_FLAGS = BIT_MASK_0;
        // OPTIONS: BIT_MASK_0 | BIT_MASK_1 | BIT_MASK_2 | BIT_MASK_3 | BIT_MASK_4 | BIT_MASK_5 | BIT_MASK_6 | BIT_MASK_7;
        // BIT_MASK_0: log info
        // BIT_MASK_1: dump network json
        // BIT_MASK_2: dump input/output tensors as images, or dump ftmps of network
        // BIT_MASK_3: replace ftmp from file before inference for zg330

    public:
        // 构造函数加载神经网络并进行多网络拆分
        MultiYolov5IcoreActor(
            int icore_id,
            DeviceType &device,
            BufferManager &buffer_manager,
            const size_t net_count,
            const Yolov5Config &yolov5_cfg = Yolov5Config(),
            const FPAIConfig &fpai_cfg = FPAIConfig(),
            const Yolov5Config &face_cfg = Yolov5Config(),
            const Yolov5Config &finger_cfg = Yolov5Config())
            : BaseActor(buffer_manager),
              device_(device),
              icore_id_(icore_id),
              FRAME_W_(yolov5_cfg.FRAME_W),
              FRAME_H_(yolov5_cfg.FRAME_H),
              NET_COUNT_(net_count),
              NET_W_(yolov5_cfg.NET_W),
              NET_H_(yolov5_cfg.NET_H),
              CHUNK_COUNT_{this->buffer_manager_.getChunkCount()},
              yolov5_cfg_(yolov5_cfg),
              face_cfg_(face_cfg),
              finger_cfg_(finger_cfg),
              fpai_cfg_(fpai_cfg),
              network_groups_{net_count},
              imk_network_groups_{net_count},
              netinfos_{net_count},
              network_session_groups_{net_count},
              imk_session_groups_{net_count},
              icore_session_groups_{net_count},
              face_network_groups_{net_count},
              face_netinfos_{net_count},
              face_session_groups_{net_count},
              finger_network_groups_{net_count},
              finger_netinfos_{net_count},
              finger_session_groups_{net_count}
        {
            if (icore_id_ < 0 || icore_id_ >= NET_COUNT_)
            {
                throw std::runtime_error("MultiYolov5IcoreActor icore_id out of range");
            }
            // 加载网络
            if (yolov5_cfg_.JSON_PATHS.size() != yolov5_cfg_.RAW_PATHS.size())
            {
                throw std::runtime_error("Yolov5Config JSON_PATHS and RAW_PATHS should have match");
            }
            if (yolov5_cfg_.JSON_PATHS.size() < NET_COUNT_)
            {
                throw std::runtime_error("Yolov5Config JSON_PATHS and RAW_PATHS should have more than NET_COUNT");
            }
            if (face_cfg_.JSON_PATHS.size() != face_cfg_.RAW_PATHS.size() ||
                face_cfg_.JSON_PATHS.size() < NET_COUNT_)
            {
                throw std::runtime_error("yolov8-face JSON_PATHS and RAW_PATHS should match NET_COUNT");
            }
            PrintYolov5Config(yolov5_cfg_);
            if (finger_cfg_.JSON_PATHS.size() != finger_cfg_.RAW_PATHS.size() ||
                finger_cfg_.JSON_PATHS.size() < NET_COUNT_)
                throw std::runtime_error("yolov8-finger JSON_PATHS and RAW_PATHS should match NET_COUNT");
            for (int cam_num = icore_id_; cam_num < NET_COUNT_; cam_num++) // CHECK: loop should not exceed net_count
            {
                spdlog::info("{} net{} json: {}, raw: {}", yolov5_cfg_.name, cam_num, yolov5_cfg_.JSON_PATHS[cam_num], yolov5_cfg_.RAW_PATHS[cam_num]);
                auto inet_yolov5 = loadNetwork(yolov5_cfg_.JSON_PATHS[cam_num], yolov5_cfg_.RAW_PATHS[cam_num]);
                network_groups_[cam_num] = inet_yolov5;
                NetInfo netinfo = NetInfo(inet_yolov5);
                std::cout << "Has imagemake: " << (netinfo.ImageMake_on ? "Yes" : "No") << std::endl;
                std::cout << "Has detpost: " << (netinfo.DetPost_on ? "Yes" : "No") << std::endl;
                if (netinfo.DetPost_on)
                {
                    // 更新detpost conf
                    updateDetpost(netinfo, yolov5_cfg_.CONF);
                }
                netinfos_[cam_num] = netinfo;
#if defined(USE_BUYI_BACKEND)
                // BUYI架构有Icraft支持ImageMake
                auto imk_netview = inet_yolov5.viewByOpId(netinfo.ImageMakes_[0]->op_id, netinfo.ImageMakes_[0]->op_id);
                imk_network_groups_[cam_num] = imk_netview.toNetwork();
#elif defined(USE_ZG330_BACKEND)
                // ZG330架构没有ImageMake算子，仅提供NetWork对象
#endif
                face_network_groups_[cam_num] = loadNetwork(face_cfg_.JSON_PATHS[cam_num], face_cfg_.RAW_PATHS[cam_num]);
                face_netinfos_[cam_num] = NetInfo(face_network_groups_[cam_num]);
                if (face_netinfos_[cam_num].DetPost_on)
                    updateDetpost(face_netinfos_[cam_num], face_cfg_.CONF);
                finger_network_groups_[cam_num] = loadNetwork(finger_cfg_.JSON_PATHS[cam_num], finger_cfg_.RAW_PATHS[cam_num]);
                finger_netinfos_[cam_num] = NetInfo(finger_network_groups_[cam_num]);
                if (finger_netinfos_[cam_num].DetPost_on)
                    updateDetpost(finger_netinfos_[cam_num], finger_cfg_.CONF);
            }
            // 构建sessions per network
            for (int cam_index = icore_id_; cam_index < NET_COUNT_; cam_index++) // TODO: loop should not exceed net_count
            {
                auto network_sessions = std::vector<icraft::xrt::Session>(CHUNK_COUNT_);
                auto imk_sessions = std::vector<icraft::xrt::Session>(CHUNK_COUNT_);
                auto icore_sessions = std::vector<icraft::xrt::Session>(CHUNK_COUNT_);
                auto face_sessions = std::vector<icraft::xrt::Session>(CHUNK_COUNT_);
                auto finger_sessions = std::vector<icraft::xrt::Session>(CHUNK_COUNT_);

                for (int i = 0; i < CHUNK_COUNT_; i++)
                {
#if defined(USE_BUYI_BACKEND)
                    // 创建session,将网络拆分为imagemake和icore
                    network_sessions[i] = icraft::xrt::Session::Create<BackendType, icraft::xrt::HostBackend>(network_groups_[cam_index], {this->device_, icraft::xrt::HostDevice::Default()});
                    network_sessions[i].apply();
                    imk_sessions[i] = network_sessions[i].sub(netinfos_[cam_index].inp_shape_opid + 1, netinfos_[cam_index].inp_shape_opid + 2);
                    icore_sessions[i] = network_sessions[i].sub(netinfos_[cam_index].inp_shape_opid + 2);
#elif defined(USE_ZG330_BACKEND)
                    auto net_viewed = network_groups_[cam_index].view(netinfos_[cam_index].inp_shape_opid + 1); // 移除input、以及Reshape、Cast算子，通过Icraft-show查看获得
                    // auto net_viewed = network_groups_[cam_index].viewExcept({0}); // 移除input、以及Reshape、Cast算子，通过Icraft-show查看获得
                    // auto net_viewed = network_groups_[cam_index].viewExcept({0, 668, 655, 682}); // 配合注入ftmp使用，需要移除多一个Cast算子
                    // auto net_sess = icraft::xrt::Session::Create<BackendType>(net_viewed, {this->device_});
                    // listIOProcessOps(net_sess);
                    auto net_sess = initSession("zg330", net_viewed, this->device_,
                                                fpai_cfg_.ocm_option, netinfos_[cam_index].mmu || fpai_cfg_.mmu_mode,
                                                fpai_cfg_.speed_mode, fpai_cfg_.compress_ftmp);
                    auto icore_session = net_sess;
                    net_sess.apply(); // session执行前必须进行apply部署操作
                    // network_sessions[i] = net_sess.sub(netinfos_[cam_index].inp_shape_opid + 3); // 去掉前三个算子input reshape cast
                    network_sessions[i] = net_sess; // 取消input和output算子
#endif
                    // yolov8-face 使用独立 session/输入 PLDDR 区；输入 Actor 负责同帧复制。
#if defined(USE_BUYI_BACKEND)
                    face_sessions[i] = icraft::xrt::Session::Create<BackendType, icraft::xrt::HostBackend>(
                        face_network_groups_[cam_index], {this->device_, icraft::xrt::HostDevice::Default()});
                    face_sessions[i].apply();
#elif defined(USE_ZG330_BACKEND)
                    auto face_viewed = face_network_groups_[cam_index].view(face_netinfos_[cam_index].inp_shape_opid + 1);
                    face_sessions[i] = initSession("zg330", face_viewed, this->device_,
                                                   fpai_cfg_.ocm_option,
                                                   face_netinfos_[cam_index].mmu || fpai_cfg_.mmu_mode,
                                                   fpai_cfg_.speed_mode, fpai_cfg_.compress_ftmp);
                    face_sessions[i].apply();
#endif
                    face_sessions[i].enableTimeProfile(fpai_cfg_.enable_profile);
#if defined(USE_ZG330_BACKEND)
                    auto finger_viewed = finger_network_groups_[cam_index].view(finger_netinfos_[cam_index].inp_shape_opid + 1);
                    finger_sessions[i] = initSession("zg330", finger_viewed, this->device_,
                                                     fpai_cfg_.ocm_option,
                                                     finger_netinfos_[cam_index].mmu || fpai_cfg_.mmu_mode,
                                                     fpai_cfg_.speed_mode, fpai_cfg_.compress_ftmp);
                    finger_sessions[i].apply();
                    finger_sessions[i].enableTimeProfile(fpai_cfg_.enable_profile);
#else
                    throw std::runtime_error("This three-model HWPre runtime supports ZG330 only");
#endif
                    // 开启计时功能
                    network_sessions[i].enableTimeProfile(fpai_cfg_.enable_profile);
                }
                network_session_groups_[cam_index] = network_sessions;
#if defined(USE_BUYI_BACKEND)
                imk_session_groups_[cam_index] = imk_sessions;
                icore_session_groups_[cam_index] = icore_sessions;
#elif defined(USE_ZG330_BACKEND)
                // ZG330架构没有imagemake，全部算子都在icore中
                icore_session_groups_[cam_index] = network_sessions;
#endif
                face_session_groups_[cam_index] = face_sessions;
                finger_session_groups_[cam_index] = finger_sessions;
                LOG_INFO(LogP, "MultiYolov5IcoreActor initialized for net{} with {} chunks.", cam_index, CHUNK_COUNT_);
            }
#if defined(USE_ZG330_BACKEND)
            imk_session_groups_.clear(); // make sure imk_session_groups is empty
#endif
        }

        ~MultiYolov5IcoreActor()
        {
            this->stop();
        }

        // 设置 hwpre ↔ YOLO forward 共享 mutex（仅 hwpre 路径使用）
        // 防止 NPU 读 PLDDR input tensor 与 hwpre 写 PLDDR 竞争
        void setHwpreMutex(std::mutex *m)
        {
            hwpre_mutex_ = m;
        }

    protected:
        void loop() override
        {
            startfps_ = std::chrono::steady_clock::now();
            std::stringstream ss;
            ss << std::this_thread::get_id();
            uint64_t id = std::stoull(ss.str());
            LOG_INFO(LogP, "Icore thread start!, id={}", id);
            static std::vector<std::string> val_images;
            static std::vector<std::string> val_image_names;
            static int TEST_IMG_IDX = 0;
            if (OPT_DEBUG_FLAGS & BIT_MASK_3)
            {
                std::ifstream infile("/home/cjn/data/cocoval_2017.txt");
                // std::ifstream infile("/home/cjn/data/cocoval_2017_single.txt");
                if (infile.is_open())
                {
                    std::string line;
                    while (std::getline(infile, line))
                    {
                        if (!line.empty())
                        {
                            if (line.back() == '\r')
                                line.pop_back(); // Handle Windows line endings
                            val_images.push_back("/home/cjn/data/val2017/" + line);
                            val_image_names.push_back(line.substr(0, line.find_last_of('.')));
                        }
                    }
                    infile.close();
                    LOG_INFO(LogP, "Loaded {} validation images.", val_images.size());
                }
                else
                {
                    LOG_WARN(LogP, "Failed to open validation image list at /home/cjn/data/cocoval_2017.txt");
                }
            }
            if (!this->input_queue_)
                throw std::runtime_error("IcoreActor: invalid input_queue_");
            while (!this->stop_flag_)
            {
                auto t_start = std::chrono::steady_clock::now();
                InputMessageForIcore input_msg;
                if (!this->input_queue_->wait_and_pop(input_msg))
                {
                    throw std::runtime_error("IcoreActor: Failed to pop from input queue");
                }

                auto t_end_pop = std::chrono::steady_clock::now();
                auto wait_pop_duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end_pop - t_start);
                // Create the message on the heap using std::make_unique
                IcoreMessageForPost post_msg;
                post_msg.meta = input_msg.meta;
                if (input_msg.meta.error_input)
                {
                    // 直接传递错误帧，不执行AI
                    this->output_queue_->push(std::move(post_msg));
                    LOG_WARN(LogP, "[{}][Buff{}] Skip Icore forward due to error input frame. wait={}ms+icore={}ms",
                             input_msg.meta.source_id,
                             input_msg.meta.buffer_index,
                             float(wait_pop_duration.count()) / 1000);
                    continue;
                }

                // hwpre ↔ YOLO forward 共享 mutex: NPU 读 PLDDR input tensor 期间,
                // 阻止 hwpre 对同一 buffer 重新 arm (baseline_serial 模式)
                std::unique_lock<std::mutex> hwpre_lock;
                if (hwpre_mutex_)
                {
                    hwpre_lock = std::unique_lock<std::mutex>(*hwpre_mutex_);
                }

                // 从yolo中获取目标框，运行YOLO.forward
                if (OPT_DEBUG_FLAGS & BIT_MASK_3)
                {
                    auto backend = icore_session_groups_[input_msg.meta.source_id][input_msg.meta.buffer_index]->backends[0].template cast<BackendType>();
                    auto forwards = icore_session_groups_[input_msg.meta.source_id][input_msg.meta.buffer_index].getForwards();
                    auto input_op = std::get<0>(forwards[0]);
                    auto i_shape = input_op->inputs[0].tensorType()->shape;
                    auto v_id = input_op->inputs[0]->v_id;
                    std::cout << "Inject ftmp for " << v_id << ", shape: ";
                    for (auto dim : i_shape)
                    {
                        std::cout << dim << " ";
                    }
                    std::cout << std::endl;
                    replaceFtmp(backend, v_id, val_images[TEST_IMG_IDX], post_msg.org_width, post_msg.org_height, i_shape);
                    post_msg.net_width = i_shape[3];
                    post_msg.net_height = i_shape[2];
                    post_msg.image_name = val_image_names[TEST_IMG_IDX];
                    TEST_IMG_IDX++;
                    if (TEST_IMG_IDX >= val_images.size())
                    {
                        LOG_WARN(LogP, "Reached the end of validation images, stopping.");
                        this->stop_flag_.store(true);
                    }
                    else
                    {
                        LOG_WARN(LogP, "Processing validation image {}/{}: {}", TEST_IMG_IDX, val_images.size(), val_image_names[TEST_IMG_IDX]);
                    }
                }
                post_msg.icore_tensors = icore_session_groups_[input_msg.meta.source_id][input_msg.meta.buffer_index].forward(input_msg.image_tensors);

                // 手动同步
                for (auto &&tensor : post_msg.icore_tensors)
                {
                    tensor.waitForReady(std::chrono::milliseconds(10000));
                }
                // 保持与单模型 runtime 相同的设备同步边界，再启动第二个 session。
                this->device_.reset(1);
                if (input_msg.secondary_image_tensors.empty())
                    throw std::runtime_error("yolov8-face input tensor is missing");
                post_msg.secondary_icore_tensors =
                    face_session_groups_[input_msg.meta.source_id][input_msg.meta.buffer_index].forward(
                        input_msg.secondary_image_tensors);
                for (auto &&tensor : post_msg.secondary_icore_tensors)
                {
                    tensor.waitForReady(std::chrono::milliseconds(10000));
                }
                this->device_.reset(1);

                if (input_msg.tertiary_image_tensors.empty())
                    throw std::runtime_error("yolov8-finger input tensor is missing");
                post_msg.tertiary_icore_tensors =
                    finger_session_groups_[input_msg.meta.source_id][input_msg.meta.buffer_index].forward(
                        input_msg.tertiary_image_tensors);
                for (auto &&tensor : post_msg.tertiary_icore_tensors)
                    tensor.waitForReady(std::chrono::milliseconds(10000));
                this->device_.reset(1);

                if (OPT_DEBUG_FLAGS & BIT_MASK_2)
                {
                    static int dump_idx = 0;
                    // dumpOutputFtmp(inet_.viewExcept({0, net_cfg_.RGB_IMK_OP_ID, net_cfg_.IR_IMK_OP_ID}), post_msg.icore_tensors, "SFB", "./logs/icore_ftmp_dump/");
                    std::stringstream ss;
                    ss << "yolov5_" << std::setw(4) << std::setfill('0') << dump_idx;
                    dumpFtmps(ss.str(), icore_session_groups_[input_msg.meta.source_id][input_msg.meta.buffer_index]->backends[0].template cast<BackendType>());
                    /*
                    auto &memchunk = this->buffer_manager_.getChunk(post_msg.chunk_group_id, post_msg.buffer_index);
                    // dump out the icore input memchunk data
                    auto phy_addr = memchunk->begin.addr();
                    size_t bytesize = 640 * 640 * 4;
                    std::shared_ptr<int8_t[]> input0 = std::shared_ptr<int8_t[]>(new int8_t[bytesize]{0});
                    memchunk.read(reinterpret_cast<char *>(input0.get()), 0, bytesize);
                    std::ofstream out_file0("./logs/icore_input_memchunk/2590.ftmp", std::ios::out | std::ios::binary);
                    out_file0.write(reinterpret_cast<char *>(input0.get()), bytesize);
                    out_file0.close();
                    std::shared_ptr<int8_t[]> input1 = std::shared_ptr<int8_t[]>(new int8_t[bytesize]{0});
                    memchunk.read(reinterpret_cast<char *>(input1.get()), bytesize, bytesize);
                    std::ofstream out_file1("./logs/icore_input_memchunk/2592.ftmp", std::ios::out | std::ios::binary);
                    out_file1.write(reinterpret_cast<char *>(input1.get()), bytesize);
                    out_file1.close();
                    */
                    dump_idx++;
                    if (dump_idx >= 1)
                        this->stop_flag_.store(true);
                }
                // 将结果传递给下游
                // this->buffer_manager_.returnIndex(input_msg.meta.chunk_group_id, input_msg.meta.buffer_index); // DEBUG CODE
                this->output_queue_->push(std::move(post_msg));
                // Per-buffer timing log disabled to keep the streaming console quiet.
            }
        }

    public:
        std::vector<std::vector<icraft::xrt::Session>> &getImkSessionGroups()
        {
            return imk_session_groups_;
        }

        std::vector<std::vector<icraft::xrt::Session>> &getIcoreSessionGroups()
        {
            return icore_session_groups_;
        }

        std::vector<icraft::xrt::Session> &getFaceSessionGroup(size_t index = 0)
        {
            if (index >= face_session_groups_.size())
                throw std::out_of_range("yolov8-face session group index out of range");
            return face_session_groups_[index];
        }

        std::vector<NetInfo> &getFaceNetInfos()
        {
            return face_netinfos_;
        }

        std::vector<icraft::xrt::Session> &getFingerSessionGroup(size_t index = 0)
        {
            return finger_session_groups_.at(index);
        }

        std::vector<NetInfo> &getFingerNetInfos()
        {
            return finger_netinfos_;
        }

        std::vector<icraft::xir::Network> &getImkNetworks()
        {
            return imk_network_groups_;
        }

        std::vector<icraft::xir::Network> &getNetworks()
        {
            return network_groups_;
        }

        std::vector<NetInfo> &getNetInfos()
        {
            return netinfos_;
        }

    private:
        std::mutex *hwpre_mutex_ = nullptr;  // shared with SDICameraInputActor (hwpre 路径)

        // constants
        const int icore_id_;
        const int FRAME_W_;
        const int FRAME_H_;
        const int NET_W_;
        const int NET_H_;
        const int NET_COUNT_;
        const size_t CHUNK_COUNT_;
        // icraft members
        NetInfo yolov5_netinfo_;
        Yolov5Config yolov5_cfg_;
        Yolov5Config face_cfg_;
        Yolov5Config finger_cfg_;
        FPAIConfig fpai_cfg_;
        DeviceType &device_;

        std::string chunk_group_id_;
        // for multisession npu forward
        std::vector<icraft::xir::Network> imk_network_groups_;
        std::vector<icraft::xir::Network> network_groups_;
        std::vector<NetInfo> netinfos_;
        std::vector<std::vector<icraft::xrt::Session>> network_session_groups_;
        std::vector<std::vector<icraft::xrt::Session>> imk_session_groups_;
        std::vector<std::vector<icraft::xrt::Session>> icore_session_groups_;
        std::vector<icraft::xir::Network> face_network_groups_;
        std::vector<NetInfo> face_netinfos_;
        std::vector<std::vector<icraft::xrt::Session>> face_session_groups_;
        std::vector<icraft::xir::Network> finger_network_groups_;
        std::vector<NetInfo> finger_netinfos_;
        std::vector<std::vector<icraft::xrt::Session>> finger_session_groups_;

        // for computing
        icraft::xrt::Tensor tensor_placeholder_1_;

        // profiling members
        const int FPS_COUNT_NUM = 10;
        std::atomic<uint64_t> ai_num_ = 0;
        std::atomic<float> ai_fps_ = 0.f;
        std::chrono::steady_clock::time_point startfps_;
    };
}

