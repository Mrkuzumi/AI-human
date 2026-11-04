#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace frame_udp
{
constexpr std::size_t HEADER_BYTES = 28;
constexpr std::size_t MAX_PAYLOAD_BYTES = 1200;

class Sender
{
public:
    Sender(const std::string &host, uint16_t port, int width, int height, int quality)
        : width_(width), height_(height), quality_(quality)
    {
        socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0)
        {
            last_error_ = std::strerror(errno);
            return;
        }
        target_.sin_family = AF_INET;
        target_.sin_port = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &target_.sin_addr) != 1)
        {
            last_error_ = "frame UDP host must be a numeric IPv4 address";
            ::close(socket_);
            socket_ = -1;
        }
        packet_.reserve(HEADER_BYTES + MAX_PAYLOAD_BYTES);
    }

    ~Sender()
    {
        if (socket_ >= 0)
            ::close(socket_);
    }

    Sender(const Sender &) = delete;
    Sender &operator=(const Sender &) = delete;

    bool ready() const { return socket_ >= 0; }
    const std::string &lastError() const { return last_error_; }

    bool sendFrame(uint32_t sequence, uint64_t capture_time_ns, uint8_t source_id,
                   const cv::Mat &rgb565)
    {
        if (!ready() || rgb565.empty() || rgb565.type() != CV_8UC2)
        {
            last_error_ = "frame must be a non-empty RGB565 image";
            return false;
        }

        cv::Mat bgr;
        cv::cvtColor(rgb565, bgr, cv::COLOR_BGR5652BGR);
        if (bgr.cols != width_ || bgr.rows != height_)
            cv::resize(bgr, bgr, cv::Size(width_, height_), 0.0, 0.0, cv::INTER_AREA);

        jpeg_.clear();
        if (!cv::imencode(".jpg", bgr, jpeg_, {cv::IMWRITE_JPEG_QUALITY, quality_}))
        {
            last_error_ = "JPEG encoding failed";
            return false;
        }
        const std::size_t part_count =
            (jpeg_.size() + MAX_PAYLOAD_BYTES - 1) / MAX_PAYLOAD_BYTES;
        if (jpeg_.empty() || jpeg_.size() > 0xFFFFFFFFU || part_count > 0xFFFFU)
        {
            last_error_ = "encoded frame is too large";
            return false;
        }

        for (std::size_t part = 0; part < part_count; ++part)
        {
            const std::size_t offset = part * MAX_PAYLOAD_BYTES;
            const std::size_t bytes = std::min(MAX_PAYLOAD_BYTES, jpeg_.size() - offset);
            packet_.clear();
            packet_.insert(packet_.end(), {'D', 'M', 'I', '1'});
            appendU8(1);
            appendU8(source_id);
            appendU16(static_cast<uint16_t>(HEADER_BYTES));
            appendU32(sequence);
            appendU32(static_cast<uint32_t>(jpeg_.size()));
            appendU16(static_cast<uint16_t>(part));
            appendU16(static_cast<uint16_t>(part_count));
            appendU64(capture_time_ns);
            packet_.insert(packet_.end(), jpeg_.begin() + offset, jpeg_.begin() + offset + bytes);
            const auto sent = ::sendto(socket_, packet_.data(), packet_.size(), MSG_DONTWAIT,
                                       reinterpret_cast<const sockaddr *>(&target_), sizeof(target_));
            if (sent != static_cast<ssize_t>(packet_.size()))
            {
                last_error_ = std::strerror(errno);
                return false;
            }
        }
        last_error_.clear();
        return true;
    }

private:
    void appendU8(uint8_t value) { packet_.push_back(value); }
    void appendU16(uint16_t value)
    {
        appendU8(static_cast<uint8_t>(value));
        appendU8(static_cast<uint8_t>(value >> 8));
    }
    void appendU32(uint32_t value)
    {
        for (int shift = 0; shift < 32; shift += 8)
            appendU8(static_cast<uint8_t>(value >> shift));
    }
    void appendU64(uint64_t value)
    {
        for (int shift = 0; shift < 64; shift += 8)
            appendU8(static_cast<uint8_t>(value >> shift));
    }

    int socket_ = -1;
    sockaddr_in target_{};
    int width_ = 640;
    int height_ = 360;
    int quality_ = 55;
    std::vector<uchar> jpeg_;
    std::vector<uint8_t> packet_;
    std::string last_error_;
};
} // namespace frame_udp
