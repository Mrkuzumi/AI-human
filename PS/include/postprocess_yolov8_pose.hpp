#pragma once
#include <opencv2/opencv.hpp>
#include <icraft-xrt/core/tensor.h>
#include <icraft-xrt/dev/host_device.h>

#include "modelzoo_utils.hpp"
#include "yolov8_pose_utils.hpp"




std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> nms_hard(std::vector<cv::Rect2f>& box_list, std::vector<float>& socre_list, std::vector<int>& id_list, std::vector<std::vector<std::vector<float>>> key_point_list, const float& conf, const float& iou, icraft::xrt::Device& device) {
    std::vector<int> nms_indices;
    std::vector<std::pair<float, int> > score_index_vec;
    std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> num_res;
    if (box_list.size() == 0) return num_res;
    for (size_t i = 0; i < socre_list.size(); ++i) {
        if (socre_list[i] > conf) {
            score_index_vec.emplace_back(std::make_pair(socre_list[i], i));
        }
    }
    std::stable_sort(score_index_vec.begin(), score_index_vec.end(),
        [](const std::pair<float, int>& pair1, const std::pair<float, int>& pair2) {return pair1.first > pair2.first; });
    std::vector<int16_t> nms_pre_data;
    for (size_t i = 0; i < score_index_vec.size(); ++i) {
        const int idx = score_index_vec[i].second;
        auto x1 = box_list[idx].tl().x;
        if (x1 < 0) x1 = 0;
        auto y1 = box_list[idx].tl().y;
        if (y1 < 0) y1 = 0;
        auto x2 = box_list[idx].br().x;
        auto y2 = box_list[idx].br().y;
        nms_pre_data.push_back((int16_t)x1);
        nms_pre_data.push_back((int16_t)y1);
        nms_pre_data.push_back((int16_t)x2);
        nms_pre_data.push_back((int16_t)y2);
        nms_pre_data.push_back((int16_t)id_list[i]);
    }
    int box_num = score_index_vec.size();
    auto nms_data_cptr = nms_pre_data.data();
    auto uregion_ = device.getMemRegion("udma");
    auto udma_chunk_ = uregion_.malloc(10e6);
    auto mapped_base = udma_chunk_->begin.addr();
    udma_chunk_.write(0, (char*)nms_data_cptr, box_num * 10);
    //hard nms config
    float threshold_f = iou;
    uint64_t arbase = mapped_base;
    uint64_t awbase = mapped_base;
    uint64_t reg_base = 0x100001C00;
    //检查硬件的版本信息是否正确，不正确会抛出错误
    if (device.defaultRegRegion().read(reg_base + 0x008, true) != 0x23110200) {
        throw std::runtime_error("ERROR :: No NMS HardWare");
    }
    auto group_num = (uint64_t)ceilf((float)box_num / 16.f);
    if (group_num == 0) throw std::runtime_error("ERROR :: group_num == 0");
    auto last_araddr = arbase + group_num * 160 - 8;
    if (last_araddr < arbase) throw std::runtime_error("ERROR :: last_araddr < arbase");
    auto anchor_hpsize = (uint64_t)ceilf((float)box_num / 64.f);
    if (anchor_hpsize == 0) throw std::runtime_error("ERROR :: anchor_hpsize == 0");
    auto last_awaddr = awbase + anchor_hpsize * 8 - 8;
    if (last_awaddr < awbase) throw std::runtime_error("ERROR :: last_awaddr < awbase");

    auto threshold = (uint16_t)(threshold_f * pow(2, 15));
    //config reg
    device.defaultRegRegion().write(reg_base + 0x014, 1, true);
    device.defaultRegRegion().write(reg_base + 0x014, 0, true);
    device.defaultRegRegion().write(reg_base + 0x01C, arbase, true);
    device.defaultRegRegion().write(reg_base + 0x020, awbase, true);
    device.defaultRegRegion().write(reg_base + 0x024, last_araddr, true);
    device.defaultRegRegion().write(reg_base + 0x028, last_awaddr, true);
    device.defaultRegRegion().write(reg_base + 0x02C, group_num, true);
    device.defaultRegRegion().write(reg_base + 0x030, 1, true); //mode 
    device.defaultRegRegion().write(reg_base + 0x034, threshold, true);
    device.defaultRegRegion().write(reg_base + 0x038, anchor_hpsize, true);

    device.defaultRegRegion().write(reg_base + 0x0, 1, true);  //start
    uint64_t reg_done;
    auto start = std::chrono::steady_clock::now();
    do {
        reg_done = device.defaultRegRegion().read(reg_base + 0x004, true);
        std::chrono::duration<double, std::milli> duration = std::chrono::steady_clock::now() - start;
        if (duration.count() > 1000) {
            throw std::runtime_error("NMS Timeout!!!");
        }
    } while (reg_done == 0);
    uint64_t mask_size = (uint64_t)(ceilf((float)box_num / 8.f));
    char* mask = new char[64000];
    udma_chunk_.read(mask, 0, mask_size);

    for (int i = 0; i < score_index_vec.size(); ++i) {
        const int idx = score_index_vec[i].second;
        int mask_index = i / 8;
        if (i % 8 == 0 && ((mask[mask_index] & (uint8_t)1) != 0))
            nms_indices.emplace_back(idx);
        else if (i % 8 == 1 && ((mask[mask_index] & (uint8_t)2) != 0))
            nms_indices.emplace_back(idx);
        else if (i % 8 == 2 && ((mask[mask_index] & (uint8_t)4) != 0))
            nms_indices.emplace_back(idx);
        else if (i % 8 == 3 && ((mask[mask_index] & (uint8_t)8) != 0))
            nms_indices.emplace_back(idx);
        else if (i % 8 == 4 && ((mask[mask_index] & (uint8_t)16) != 0))
            nms_indices.emplace_back(idx);
        else if (i % 8 == 5 && ((mask[mask_index] & (uint8_t)32) != 0))
            nms_indices.emplace_back(idx);
        else if (i % 8 == 6 && ((mask[mask_index] & (uint8_t)64) != 0))
            nms_indices.emplace_back(idx);
        else if (i % 8 == 7 && ((mask[mask_index] & (uint8_t)128) != 0))
            nms_indices.emplace_back(idx);
    }
    delete mask;
    for (auto idx : nms_indices) {
        num_res.push_back({ id_list[idx],socre_list[idx],box_list[idx], key_point_list[idx]});
    }
    return num_res;
}


std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> nms_soft(std::vector<int>& id_list, std::vector<float>& socre_list, std::vector<cv::Rect2f>& box_list, std::vector<std::vector<std::vector<float>>>& key_point_list, float IOU, int max_nms = 3000) {
	std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> filter_res;
	std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> nms_res;
	auto bbox_num = id_list.size();
	for (size_t i = 0; i < bbox_num; i++)
	{
		filter_res.push_back({ id_list[i],socre_list[i],box_list[i], key_point_list[i] });
	}

	std::stable_sort(filter_res.begin(), filter_res.end(),
		[](const std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>& tuple1, const std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>& tuple2) {
			return std::get<1>(tuple1) > std::get<1>(tuple2);
		}
	);

	int idx = 0;
	for (auto res : filter_res) {
		bool keep = true;
		for (int k = 0; k < nms_res.size() && keep; ++k) {
			if (std::get<0>(res) == std::get<0>(nms_res[k])) {
				if (1.f - jaccardDistance(std::get<2>(res), std::get<2>(nms_res[k])) > IOU) {
					keep = false;
				}
			}

		}
		if (keep == true)
			nms_res.emplace_back(res);
		if (idx > max_nms) {
			break;
		}
		idx++;
	}
	return nms_res;
};


std::vector<std::vector<float>> coordTrans(std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>>& nms_res, PicPre& img, bool check_border = true) {
    std::vector<std::vector<float>> output_data;
    int left_pad = img.getPad().first;
    int top_pad = img.getPad().second;
    float ratio = img.getRatio().first;
    //std::cout <<"ratio: " << ratio << std::endl;
    for (auto&& res : nms_res) {
        float class_id = std::get<0>(res);
        float score = std::get<1>(res);
        auto box = std::get<2>(res);
        float x1 = (box.tl().x - left_pad) / ratio;
        float y1 = (box.tl().y - top_pad) / ratio;
        float x2 = (box.br().x - left_pad) / ratio;
        float y2 = (box.br().y - top_pad) / ratio;
        if (check_border) {
            x1 = checkBorder(x1, 0.f, (float)img.src_img.cols);
            y1 = checkBorder(y1, 0.f, (float)img.src_img.rows);
            x2 = checkBorder(x2, 0.f, (float)img.src_img.cols);
            y2 = checkBorder(y2, 0.f, (float)img.src_img.rows);
        }
        float w = x2 - x1;
        float h = y2 - y1;
        //bbox：左上角点和wh
        std::vector<float> one_obj_res;
        one_obj_res = { class_id, x1, y1, w, h, score };
        for (auto i : std::get<3>(res)) {
            auto kpx = (i[0] - left_pad) / ratio;
            auto kpy = (i[1] - top_pad) / ratio;
            auto kpprob = i[2];
            one_obj_res.push_back(kpx);
            one_obj_res.push_back(kpy);
            one_obj_res.push_back(kpprob);
        }
        output_data.emplace_back(one_obj_res);
    }
    return output_data;
}


const int kCocoSkeleton[19][2] = { {16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12},
                                   {7, 13}, {6, 7}, {6, 8}, {7, 9}, {8, 10}, {9, 11}, {2, 3},
                                   {1, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7} };

const int kColorMap_YOLOV7_POSE[][3] = { {255, 128, 0}, {255, 153, 51}, {255, 178, 102},
                           {230, 230, 0}, {255, 153, 255}, {153, 204, 255},
                           {255, 102, 255}, {255, 51, 255}, {102, 178, 255},
                           {51, 153, 255}, {255, 153, 153}, {255, 102, 102},
                           {255, 51, 51}, {153, 255, 153}, {102, 255, 102},
                           {51, 255, 51}, {0, 255, 0}, {0, 0, 255},
                           {255, 0, 0},  {255, 255, 255} };

const int kLimbColorIndex[] = { 9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16 };    // 16 limps 
const int kKptColorIndex[] = { 16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9 };             // 17 key points

void visualize(std::vector<std::vector<float>>& output_res, const cv::Mat& img, const std::string resRoot, const std::string name, const std::vector<std::string>& names,int NOC) {
    std::default_random_engine e;
    std::uniform_int_distribution<unsigned> u(10, 200);
    for (auto res : output_res) {
        int class_id = (int)res[0];
        float x1 = res[1];
        float y1 = res[2];
        float w = res[3];
        float h = res[4];
        float score = res[5];
        cv::Scalar color_ = cv::Scalar(u(e), u(e), u(e));
        cv::rectangle(img, cv::Rect2f(x1, y1, w, h), color_, 2);
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << score;
        std::string s = std::to_string(class_id) + "_" + "obj" + " " + ss.str();
        auto s_size = cv::getTextSize(s, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, 0);
        cv::rectangle(img, cv::Point2f(x1 - 1, y1 - s_size.height - 7), cv::Point2f(x1 + s_size.width, y1 - 2), color_, -1);
        cv::putText(img, s, cv::Point2f(x1, y1 - 2), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 0.2);
    }

    NOC = 1;
    for (auto res : output_res) {
        std::vector < std::vector < float >> oneobj_kpt;
        for (size_t i = 5 + NOC; i < res.size(); i += 3)
        {
            std::vector < float > one_point;
            one_point = { res[i],res[i + 1], res[i + 2] };
            oneobj_kpt.push_back(one_point);
        }

        int k = 0;
        for (auto one_kpt : oneobj_kpt) {
            if (one_kpt[2] > 0.5) {
                cv::Scalar color = cv::Scalar(kColorMap_YOLOV7_POSE[kKptColorIndex[k]][0], kColorMap_YOLOV7_POSE[kKptColorIndex[k]][1], kColorMap_YOLOV7_POSE[kKptColorIndex[k]][2]);
                cv::circle(img, { (int)one_kpt[0], (int)one_kpt[1] }, 4, color, -1);
                k++;
            }
        }

        int ske = 0;
        for (auto sk_pair : kCocoSkeleton) {
            int kp1 = sk_pair[0] - 1;
            int kp2 = sk_pair[1] - 1;
            if (((oneobj_kpt[kp1][2]) > 0.5) && ((oneobj_kpt[kp2][2]) > 0.5)) {
                cv::Scalar color = cv::Scalar(kColorMap_YOLOV7_POSE[kLimbColorIndex[ske]][0], kColorMap_YOLOV7_POSE[kLimbColorIndex[ske]][1], kColorMap_YOLOV7_POSE[kLimbColorIndex[ske]][2]);
                cv::line(img,
                    { int(oneobj_kpt[kp1][0]), int(oneobj_kpt[kp1][1]) },
                    { int(oneobj_kpt[kp2][0]), int(oneobj_kpt[kp2][1]) },
                    color, 2);
            }
            ske++;

        }
    }
    #ifdef _WIN32
        cv::imshow("results", img);
        cv::waitKey(0);
    #elif __linux__
        std::string save_path = resRoot + '/' + name;
        std::regex rgx("\\.(?!.*\\.)"); // 匹配最后一个点号（.）之前的位置，且该点号后面没有其他点号
        std::string RES_PATH = std::regex_replace(save_path, rgx, "_result.");
        cv::imwrite(RES_PATH, img);
    #endif
}



using namespace icraft::xrt;
struct Grid {
    uint16_t location_x = 0;
    uint16_t location_y = 0;
    uint16_t anchor_index = 0;
};


std::vector<int> get_stride(NetInfo& netinfo) {
    std::vector<int> stride;
    for (auto i : netinfo.o_cubic) {
        stride.emplace_back(netinfo.i_cubic[0].h / i.h);
    }
    return stride;
};

template <typename T>
Grid get_grid(int bits, T* tensor_data, int base_addr, int anchor_length) {
    Grid grid;
    uint16_t anchor_index;
    uint16_t location_y;
    uint16_t location_x;
    if (bits == 8)
    {
        anchor_index = (((uint16_t)tensor_data[base_addr + anchor_length - 1]) << 8) + (uint8_t)tensor_data[base_addr + anchor_length - 2];
        location_y = (((uint16_t)tensor_data[base_addr + anchor_length - 3]) << 8) + (uint8_t)tensor_data[base_addr + anchor_length - 4];
        location_x = (((uint16_t)tensor_data[base_addr + anchor_length - 5]) << 8) + (uint8_t)tensor_data[base_addr + anchor_length - 6];

    }
    else if (bits == 16)
    {
        anchor_index = (uint16_t)tensor_data[base_addr + anchor_length - 1];
        location_y = (uint16_t)tensor_data[base_addr + anchor_length - 2];
        location_x = (uint16_t)tensor_data[base_addr + anchor_length - 3];
    }
    grid.location_x = location_x;
    grid.location_y = location_y;
    grid.anchor_index = anchor_index;
    return grid;
}

template <typename T>
void get_cls_bbox(std::vector<int>& id_list, std::vector<float>& socre_list, std::vector<cv::Rect2f>& box_list, std::vector<std::vector<std::vector<float>>>& key_point_list, T* tensor_data, int base_addr,
    Grid& grid, std::vector<float>& norm, int stride,
    int N_CLASS,int NOP, float THR_F, std::vector<int>& real_out_channles, int bbox_info_channel, int kptdata_ptr_start) {
    
    
    T* class_ptr_start = tensor_data + base_addr;
    T* max_prob_ptr = std::max_element(class_ptr_start, class_ptr_start + N_CLASS);
    int max_index = std::distance(class_ptr_start, max_prob_ptr);
    auto _prob_ = sigmoid(*max_prob_ptr * norm[0]);
    auto realscore = _prob_;

    if (realscore > THR_F) {
        //getBbox
        std::vector<float> ltrb = dfl(tensor_data,
            norm[1], base_addr + real_out_channles[0], bbox_info_channel);
        float x1 = grid.location_x + 0.5 - ltrb[0];
        float y1 = grid.location_y + 0.5 - ltrb[1];
        float x2 = grid.location_x + 0.5 + ltrb[2];
        float y2 = grid.location_y + 0.5 + ltrb[3];

        float x = ((x2 + x1) / 2.f) * stride;
        float y = ((y2 + y1) / 2.f) * stride;
        float w = (x2 - x1) * stride;
        float h = (y2 - y1) * stride;
        std::vector<float> xywh = { x,y,w,h };


        // key points 
        auto kpt_ptr_start = tensor_data + base_addr + kptdata_ptr_start;
        std::vector<std::vector<float>> one_obj_kpt;
        for (int k = 0; k < NOP; ++k) {
            std::vector<float> one_kpt;
            float ori_x = kpt_ptr_start[k * 3] * norm[2];
            float ori_y = kpt_ptr_start[k * 3 + 1] * norm[2];
            float kpt_score = kpt_ptr_start[k * 3 + 2] * norm[2];
            float kpt_x = (ori_x * 2. + grid.location_x) * stride;
            float kpt_y = (ori_y * 2. + grid.location_y) * stride;
            float kpt_prob = sigmoid(kpt_score);
            one_kpt = { kpt_x, kpt_y, kpt_prob };
            one_obj_kpt.emplace_back(one_kpt);
        }

        key_point_list.emplace_back(one_obj_kpt);


        id_list.emplace_back(max_index);
        socre_list.emplace_back(realscore);
        box_list.emplace_back(cv::Rect2f((xywh[0] - xywh[2] / 2),
            (xywh[1] - xywh[3] / 2), xywh[2], xywh[3]));
    }
}


void post_detpost_hard(const std::vector<Tensor>& output_tensors, PicPre& img, NetInfo& netinfo,
    float conf, float iou_thresh, bool fpga_nms, int N_CLASS,int NOP, 
    std::vector<std::vector<std::vector<float>>> &ANCHORS,std::vector<std::string>& LABELS, 
    bool & show, bool & save , std::string &resRoot, std::string & name,icraft::xrt::Device device,const std::string& runBackend, std::vector<int>& real_out_channles, int bbox_info_channel) {

    std::vector<int> id_list;
    std::vector<float> socre_list;
    std::vector<cv::Rect2f> box_list;
    std::vector<std::vector<std::vector<float>>> key_point_list;

    std::vector<int> ori_out_channles = { N_CLASS,bbox_info_channel,NOP * 3 };
    int parts = ori_out_channles.size();
    std::vector<std::vector<float>> _norm = set_norm_by_head(output_tensors.size(), parts, netinfo.o_scale);
    std::vector<int> _stride = get_stride(netinfo);
    
    for (size_t i = 0; i < output_tensors.size(); i++) {
 
        auto host_tensor = output_tensors[i].to(HostDevice::MemRegion());
        int output_tensors_bits = output_tensors[i].dtype()->element_dtype.getStorageType().bits();
        int obj_num = output_tensors[i].dtype()->shape[2];
        int anchor_length = output_tensors[i].dtype()->shape[3];
        if (output_tensors_bits == 16) {
            auto tensor_data = (int16_t*)host_tensor.data().cptr();
            int kptdata_ptr_start = ceil((N_CLASS) / float(32)) * 32 + bbox_info_channel;

            for (size_t obj = 0; obj < obj_num; obj++) {
                int base_addr = obj * anchor_length; 
                Grid grid = get_grid(output_tensors_bits, tensor_data, base_addr, anchor_length);
                get_cls_bbox(id_list, socre_list, box_list, key_point_list, tensor_data, base_addr, grid, _norm[i], _stride[i], N_CLASS, NOP, conf, real_out_channles, bbox_info_channel, kptdata_ptr_start);
            }
        }
        else {
            auto tensor_data = (int8_t*)host_tensor.data().cptr();
            int kptdata_ptr_start = ceil((N_CLASS) / float(64)) * 64 + bbox_info_channel;

            for (size_t obj = 0; obj < obj_num; obj++) {
                int base_addr = obj * anchor_length; 
                Grid grid = get_grid(output_tensors_bits, tensor_data, base_addr, anchor_length);
                get_cls_bbox(id_list, socre_list, box_list, key_point_list, tensor_data, base_addr,grid, _norm[i], _stride[i], N_CLASS, NOP, conf, real_out_channles, bbox_info_channel, kptdata_ptr_start);

            }
        }
    }
    std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> nms_res;
    //std::cout << "number of results before nms = " << id_list.size() << '\n';
    //auto post_start = std::chrono::system_clock::now();
    if (fpga_nms&& runBackend.compare("buyi") == 0) {
        nms_res = nms_hard(box_list, socre_list, id_list, key_point_list, conf, iou_thresh, device);
    }
    else {
        nms_res = nms_soft(id_list, socre_list, box_list, key_point_list, iou_thresh);   // 后处理 之 NMS

    }
    //auto post_end = std::chrono::system_clock::now();
    //auto post_time = std::chrono::duration_cast<std::chrono::microseconds>(post_end - post_start);
    //std::cout << "nms_time = " << double(post_time.count() / 1000.f) << '\n';
    //std::cout << "number of results after nms = " << nms_res.size() << '\n';
    std::vector<std::vector<float>> output_res = coordTrans(nms_res, img);

    #ifdef _WIN32
    if (show) {
        visualize(output_res, img.ori_img, resRoot, name, LABELS, N_CLASS);
    }
    #endif
    if (save) {
        #ifdef _WIN32
            saveRes(output_res, resRoot, name);
        #elif __linux__
            visualize(output_res, img.ori_img, resRoot, name, LABELS, N_CLASS);
        #endif
    }


}

void post_detpost_soft(const std::vector<Tensor>& output_tensors, PicPre& img, NetInfo& netinfo,
    float conf, float iou_thresh, bool fpga_nms, int N_CLASS, int NOP,
    std::vector<std::vector<std::vector<float>>>& ANCHORS, std::vector<std::string>& LABELS,
    bool& show, bool& save, std::string& resRoot, std::string& name, icraft::xrt::Device device, const std::string& runBackend, int bbox_info_channel) {

    std::vector<int> id_list;
    std::vector<float> socre_list;
    std::vector<cv::Rect2f> box_list;
    std::vector<std::vector<std::vector<float>>> key_point_list;
    std::vector<int> stride = get_stride(netinfo);

    for (size_t yolo = 0; yolo < output_tensors.size(); yolo = yolo + 3) {

        int _H = output_tensors[yolo].dtype()->shape[1];
        int _W = output_tensors[yolo].dtype()->shape[2];
        auto host_tensor_class = output_tensors[yolo + 0].to(HostDevice::MemRegion());
        auto host_tensor_box = output_tensors[yolo + 1].to(HostDevice::MemRegion());
        auto host_tensor_pose = output_tensors[yolo + 2].to(HostDevice::MemRegion());
        auto tensor_data_class = (float*)host_tensor_class.data().cptr();
        auto tensor_data_box = (float*)host_tensor_box.data().cptr();
        auto tensor_data_pose = (float*)host_tensor_pose.data().cptr();

        for (size_t h = 0; h < _H; h++) {
            for (size_t w = 0; w < _W; w++) {

                auto one_head_stride = stride[yolo];
                auto classPtr = tensor_data_class +
                    h * _W * N_CLASS +
                    w * N_CLASS;
                auto boxPtr = tensor_data_box +
                    h * _W * 64 +
                    w * 64;
                auto posPtr = tensor_data_pose +
                    h * _W * 51 +
                    w * 51;

                for (size_t cls_idx = 0; cls_idx < N_CLASS; cls_idx++) {
                    auto _prob_ = sigmoid(*(classPtr + cls_idx));
                    auto realscore = _prob_;
                    if (realscore > conf) {
                        //getBbox
                        std::vector<float> ltrb = dfl(boxPtr, 1.0, 0, 64);
                        float x1 = w + 0.5 - ltrb[0];
                        float y1 = h + 0.5 - ltrb[1];
                        float x2 = w + 0.5 + ltrb[2];
                        float y2 = h + 0.5 + ltrb[3];

                        float x_ = ((x2 + x1) / 2.f) * one_head_stride;
                        float y_ = ((y2 + y1) / 2.f) * one_head_stride;
                        float w_ = (x2 - x1) * one_head_stride;
                        float h_ = (y2 - y1) * one_head_stride;
                        std::vector<float> xywh = { x_,y_,w_,h_ };

                        // key points
                        std::vector<std::vector<float>> one_obj_kpt;
                        for (size_t k = 0; k < NOP; k++)
                        {
                            std::vector<float> one_kpt;
                            float ori_x = posPtr[k * 3];
                            float ori_y = posPtr[k * 3 + 1];
                            float kpt_score = posPtr[k * 3 + 2];
                            float kpt_x = (ori_x * 2. + w) * one_head_stride;
                            float kpt_y = (ori_y * 2. + h) * one_head_stride;
                            float kpt_prob = sigmoid(kpt_score);
                            one_kpt = { kpt_x, kpt_y, kpt_prob };
                            one_obj_kpt.emplace_back(one_kpt);
                        }

                        id_list.emplace_back(cls_idx);
                        socre_list.emplace_back(realscore);
                        box_list.emplace_back(cv::Rect2f((xywh[0] - xywh[2] / 2),
                            (xywh[1] - xywh[3] / 2), xywh[2], xywh[3]));
                        key_point_list.emplace_back(one_obj_kpt);
                    }
                }
            }

        }
    }
    std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> nms_res;
    //std::cout << "number of results before nms = " << id_list.size() << '\n';
    //auto post_start = std::chrono::system_clock::now();
    if (fpga_nms && runBackend.compare("buyi") == 0) {
        nms_res = nms_hard(box_list, socre_list, id_list, key_point_list, conf, iou_thresh, device);
    }
    else {
        nms_res = nms_soft(id_list, socre_list, box_list, key_point_list, iou_thresh);   // 后处理 之 NMS

    }
    //auto post_end = std::chrono::system_clock::now();
    //auto post_time = std::chrono::duration_cast<std::chrono::microseconds>(post_end - post_start);
    //std::cout << "nms_time = " << double(post_time.count() / 1000.f) << '\n';
    //std::cout << "number of results after nms = " << nms_res.size() << '\n';
    std::vector<std::vector<float>> output_res = coordTrans(nms_res, img);

    #ifdef _WIN32
        if (show) {
            visualize(output_res, img.ori_img, resRoot, name, LABELS, N_CLASS);
        }
    #endif
        if (save) {
    #ifdef _WIN32
            saveRes(output_res, resRoot, name);
    #elif __linux__
            visualize(output_res, img.ori_img, resRoot, name, LABELS, N_CLASS);
    #endif
        }
}

// smooth
const float ALPHA = 0.5f;
const float SMOOTH_IOU = 0.80f;
const float KPT_ALPHA = 0.40f;
const float KPT_SMOOTH_IOU = 0.50f;

using YoloPostResult = std::tuple<std::vector<int>, std::vector<float>, std::vector<cv::Rect2f>, std::vector<std::vector<std::vector<float>>>  >; // box_list, id_list, score_list, key_point_list

YoloPostResult post_detpost_plin(const std::vector<Tensor>& output_tensors, YoloPostResult& last_frame_result,NetInfo& netinfo,
  float conf, float iou_thresh, bool MULTILABEL, bool fpga_nms, int N_CLASS, int NOP,
  std::vector<std::vector<std::vector<float>>>& ANCHORS, icraft::xrt::Device device, std::vector<int>& real_out_channles, int bbox_info_channel) {

    std::vector<int> id_list;
    std::vector<float> socre_list;
    std::vector<cv::Rect2f> box_list;
    std::vector<std::vector<std::vector<float>>> key_point_list;

    std::vector<int> ori_out_channles = { N_CLASS,bbox_info_channel,NOP * 3 };
    int parts = ori_out_channles.size();
    std::vector<std::vector<float>> _norm = set_norm_by_head(output_tensors.size(), parts, netinfo.o_scale);
    std::vector<int> _stride = get_stride(netinfo);
    // std::vector<int> _stride =
    //     set_stride_by_head(output_tensors.size(), parts, stride);

    for (size_t i = 0; i < output_tensors.size(); i++) {

        auto host_tensor = output_tensors[i].to(HostDevice::MemRegion());
        int output_tensors_bits = output_tensors[i].dtype()->element_dtype.getStorageType().bits();
        int obj_num = output_tensors[i].dtype()->shape[2];
        int anchor_length = output_tensors[i].dtype()->shape[3];
        if (output_tensors_bits == 16) {
            auto tensor_data = (int16_t*)host_tensor.data().cptr();
            int kptdata_ptr_start = ceil((N_CLASS) / float(32)) * 32 + bbox_info_channel;

            for (size_t obj = 0; obj < obj_num; obj++) {
                int base_addr = obj * anchor_length;
                Grid grid = get_grid(output_tensors_bits, tensor_data, base_addr, anchor_length);
                get_cls_bbox(id_list, socre_list, box_list, key_point_list, tensor_data, base_addr, grid, _norm[i], _stride[i], N_CLASS, NOP, conf, real_out_channles, bbox_info_channel, kptdata_ptr_start);
            }
        }
        else {
            auto tensor_data = (int8_t*)host_tensor.data().cptr();
            int kptdata_ptr_start = ceil((N_CLASS) / float(64)) * 64 + bbox_info_channel;

            for (size_t obj = 0; obj < obj_num; obj++) {
                int base_addr = obj * anchor_length;
                Grid grid = get_grid(output_tensors_bits, tensor_data, base_addr, anchor_length);
                get_cls_bbox(id_list, socre_list, box_list, key_point_list, tensor_data, base_addr, grid, _norm[i], _stride[i], N_CLASS, NOP, conf, real_out_channles, bbox_info_channel, kptdata_ptr_start);

            }
        }
    }
    std::vector<std::tuple<int, float, cv::Rect2f, std::vector<std::vector<float>>>> nms_res;
    //std::cout << "number of results before nms = " << id_list.size() << '\n';
    //auto post_start = std::chrono::system_clock::now();
    if (fpga_nms) {
        nms_res = nms_hard(box_list, socre_list, id_list, key_point_list, conf, iou_thresh, device);
    }
    else {
        nms_res = nms_soft(id_list, socre_list, box_list, key_point_list, iou_thresh);   // 后处理 之 NMS

    }



    // Smooth matched detections only; face also uses this decoder with its own history.
    auto id_list_last_frame = std::get<0>(last_frame_result);
    auto score_list_last_frame = std::get<1>(last_frame_result);
    auto box_list_last_frame = std::get<2>(last_frame_result);
    const auto &key_point_list_last_frame = std::get<3>(last_frame_result);

    for (auto &idx_score_bbox : nms_res) {
        size_t best_prev = box_list_last_frame.size();
        float best_iou = 0.0f;
        for (size_t i = 0; i < box_list_last_frame.size(); ++i) {
            if (i >= id_list_last_frame.size() || std::get<0>(idx_score_bbox) != id_list_last_frame[i])
                continue;
            const float match_iou = 1.0f - cv::jaccardDistance(std::get<2>(idx_score_bbox), box_list_last_frame[i]);
            if (match_iou > best_iou) {
                best_iou = match_iou;
                best_prev = i;
            }
        }
        if (best_prev >= box_list_last_frame.size() || best_iou <= KPT_SMOOTH_IOU)
            continue;

        if (best_iou > SMOOTH_IOU && best_prev < score_list_last_frame.size()) {
            auto &box = std::get<2>(idx_score_bbox);
            const auto &previous_box = box_list_last_frame[best_prev];
            box.x = previous_box.x * ALPHA + box.x * (1.0f - ALPHA);
            box.y = previous_box.y * ALPHA + box.y * (1.0f - ALPHA);
            box.width = previous_box.width * ALPHA + box.width * (1.0f - ALPHA);
            box.height = previous_box.height * ALPHA + box.height * (1.0f - ALPHA);
            std::get<1>(idx_score_bbox) = score_list_last_frame[best_prev] * ALPHA +
                                          std::get<1>(idx_score_bbox) * (1.0f - ALPHA);
        }

        auto &current_kpts = std::get<3>(idx_score_bbox);
        if (best_prev < key_point_list_last_frame.size() &&
            current_kpts.size() == key_point_list_last_frame[best_prev].size()) {
            const auto &previous_kpts = key_point_list_last_frame[best_prev];
            for (size_t k = 0; k < current_kpts.size(); ++k) {
                if (current_kpts[k].size() < 3 || previous_kpts[k].size() < 3)
                    continue;
                for (size_t axis = 0; axis < 3; ++axis)
                    current_kpts[k][axis] = previous_kpts[k][axis] * (1.0f - KPT_ALPHA) +
                                            current_kpts[k][axis] * KPT_ALPHA;
            }
        }
    }
    std::vector<int> id_list_ret;   //id_list_ret.reserve(nms_indices.size());
    std::vector<float> score_list_ret; //score_list_ret.reserve(nms_indices.size());
    std::vector<cv::Rect2f> box_list_ret; //box_list_ret.reserve(nms_indices.size());
    std::vector<std::vector<std::vector<float>>> key_point_list_ret; //key_point_lis.reserve(nms_indices.size());
    for (auto  idx_score_bbox : nms_res) {
        // store 
        id_list_ret.emplace_back(std::get<0>(idx_score_bbox));
        score_list_ret.emplace_back(std::get<1>(idx_score_bbox));
        box_list_ret.emplace_back(std::get<2>(idx_score_bbox));
        key_point_list_ret.emplace_back(std::get<3>(idx_score_bbox));
    }
    return YoloPostResult { id_list_ret, score_list_ret, box_list_ret, key_point_list_ret };

    }
