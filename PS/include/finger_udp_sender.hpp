#pragma once
#include "motion_udp_sender.hpp"
#include <algorithm>

// Separate FNG1 packets preserve the existing DMV1 body/face receiver.
namespace finger_udp {
constexpr std::size_t POINT_COUNT = 21;
constexpr std::size_t MAX_HANDS = 2;
using Detection = motion_udp::Detection;

class Sender {
public:
    Sender(const std::string &host, uint16_t port) {
        socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0) { last_error_ = std::strerror(errno); return; }
        target_.sin_family = AF_INET;
        target_.sin_port = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &target_.sin_addr) != 1) {
            last_error_ = "finger UDP host must be a numeric IPv4 address";
            ::close(socket_); socket_ = -1;
        }
        packet_.reserve(588); // 40 + 2 * (20 + 21 * 12) + 4, fits one UDP datagram.
    }
    ~Sender() { if (socket_ >= 0) ::close(socket_); }
    Sender(const Sender &) = delete;
    Sender &operator=(const Sender &) = delete;
    bool ready() const { return socket_ >= 0; }
    const std::string &lastError() const { return last_error_; }

    bool sendFrame(uint32_t sequence, uint64_t capture_ns, uint64_t emit_ns,
                   uint16_t width, uint16_t height, uint8_t source_id,
                   const std::vector<Detection> &hands) {
        if (!ready()) return false;
        if (hands.size() > MAX_HANDS || std::any_of(hands.begin(), hands.end(),
            [](const Detection &d) { return d.keypoints.size() != POINT_COUNT; })) {
            last_error_ = "invalid finger hand count or keypoint count"; return false;
        }
        const uint32_t bytes = 44 + hands.size() * 272;
        packet_.clear();
        packet_.insert(packet_.end(), {'F', 'N', 'G', '1'});
        appendInt(1, 1); appendInt(4, 1); // version, normalized coordinates flag
        appendInt(40, 2); appendInt(bytes, 4); appendInt(sequence, 4);
        appendInt(capture_ns, 8); appendInt(emit_ns, 8);
        appendInt(width, 2); appendInt(height, 2);
        appendInt(hands.size(), 1); appendInt(POINT_COUNT, 1);
        appendInt(source_id, 1); appendInt(0, 1);
        for (const auto &hand : hands) {
            appendFloat(hand.score);
            for (float v : hand.box) appendFloat(v);
            for (const auto &point : hand.keypoints)
                for (float v : point) appendFloat(v);
        }
        appendInt(::crc32(0L, reinterpret_cast<const Bytef *>(packet_.data()), packet_.size()), 4);
        if (packet_.size() != bytes) { last_error_ = "finger packet size mismatch"; return false; }
        const auto sent = ::sendto(socket_, packet_.data(), packet_.size(), MSG_DONTWAIT,
                                  reinterpret_cast<const sockaddr *>(&target_), sizeof(target_));
        if (sent != static_cast<ssize_t>(packet_.size())) {
            last_error_ = std::strerror(errno); return false;
        }
        last_error_.clear(); return true;
    }
private:
    void appendInt(uint64_t value, int bytes) {
        for (int i = 0; i < bytes; ++i) packet_.push_back(static_cast<uint8_t>(value >> (i * 8)));
    }
    void appendFloat(float value) {
        static_assert(sizeof(float) == 4, "FNG1 requires float32");
        uint32_t bits; std::memcpy(&bits, &value, sizeof(bits)); appendInt(bits, 4);
    }
    int socket_ = -1;
    sockaddr_in target_{};
    std::vector<uint8_t> packet_;
    std::string last_error_;
};
} // namespace finger_udp
