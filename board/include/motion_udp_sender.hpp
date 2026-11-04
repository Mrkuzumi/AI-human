#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zlib.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace motion_udp
{
constexpr std::size_t POSE_COUNT = 17;
constexpr std::size_t FACE_COUNT = 51;

struct Detection
{
    float score = 0.0f;
    std::array<float, 4> box{};
    std::vector<std::array<float, 3>> keypoints;
};

class Sender
{
public:
    Sender(const std::string &host, uint16_t port)
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
            last_error_ = "motion UDP host must be a numeric IPv4 address";
            ::close(socket_);
            socket_ = -1;
        }
        packet_.reserve(904);
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

    bool sendFrame(uint32_t sequence,
                   uint64_t capture_time_ns,
                   uint64_t emit_time_ns,
                   uint16_t frame_width,
                   uint16_t frame_height,
                   uint8_t source_id,
                   const Detection *body,
                   const Detection *face,
                   uint32_t track_id = 1)
    {
        if (!ready())
            return false;
        if ((body && body->keypoints.size() != POSE_COUNT) ||
            (face && face->keypoints.size() != FACE_COUNT))
        {
            last_error_ = "invalid motion keypoint count";
            return false;
        }

        constexpr uint8_t FLAG_BODY = 1U << 0;
        constexpr uint8_t FLAG_FACE = 1U << 1;
        constexpr uint8_t FLAG_NORMALIZED = 1U << 2;
        constexpr uint16_t HEADER_BYTES = 44;
        constexpr uint32_t DETECTION_BYTES = 20;
        constexpr uint32_t POINT_BYTES = 12;
        constexpr uint32_t CRC_BYTES = 4;

        const uint8_t flags = FLAG_NORMALIZED |
                              (body ? FLAG_BODY : 0) |
                              (face ? FLAG_FACE : 0);
        const uint32_t packet_bytes = HEADER_BYTES + CRC_BYTES +
                                      (body ? DETECTION_BYTES + POSE_COUNT * POINT_BYTES : 0) +
                                      (face ? DETECTION_BYTES + FACE_COUNT * POINT_BYTES : 0);

        packet_.clear();
        packet_.insert(packet_.end(), {'D', 'M', 'V', '2'});
        appendU8(2);
        appendU8(flags);
        appendU16(HEADER_BYTES);
        appendU32(packet_bytes);
        appendU32(sequence);
        appendU64(capture_time_ns);
        appendU64(emit_time_ns);
        appendU16(frame_width);
        appendU16(frame_height);
        appendU8(body ? static_cast<uint8_t>(POSE_COUNT) : 0);
        appendU8(face ? static_cast<uint8_t>(FACE_COUNT) : 0);
        appendU8(source_id);
        appendU8(0);
        appendU32(track_id);

        if (body)
            appendDetection(*body);
        if (face)
            appendDetection(*face);

        const uint32_t crc = static_cast<uint32_t>(
            ::crc32(0L, reinterpret_cast<const Bytef *>(packet_.data()), packet_.size()));
        appendU32(crc);
        if (packet_.size() != packet_bytes)
        {
            last_error_ = "motion packet size mismatch";
            return false;
        }

        const auto sent = ::sendto(socket_, packet_.data(), packet_.size(), MSG_DONTWAIT,
                                   reinterpret_cast<const sockaddr *>(&target_), sizeof(target_));
        if (sent != static_cast<ssize_t>(packet_.size()))
        {
            last_error_ = std::strerror(errno);
            return false;
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

    void appendFloat(float value)
    {
        static_assert(sizeof(float) == sizeof(uint32_t), "DMV2 requires 32-bit float");
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        appendU32(bits);
    }

    void appendDetection(const Detection &detection)
    {
        appendFloat(detection.score);
        for (float value : detection.box)
            appendFloat(value);
        for (const auto &point : detection.keypoints)
            for (float value : point)
                appendFloat(value);
    }

    int socket_ = -1;
    sockaddr_in target_{};
    std::vector<uint8_t> packet_;
    std::string last_error_;
};
} // namespace motion_udp

