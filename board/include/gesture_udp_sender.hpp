#pragma once

// GST1: board-side gesture signal channel.
//
// The board evaluates gestures on its PS side and publishes the *current state*
// of every gesture it knows about once per frame.  The PC bridge receives this
// on its own port and turns it into the OSC messages Unity already listens to,
// which keeps mode gating (follow ignores gestures) and event logging in one
// place.
//
// Layout (little endian):
//   0   4  magic "GST1"
//   4   1  version
//   5   1  flags                     (reserved, 0)
//   6   2  header_bytes = 48
//   8   4  packet_bytes
//  12   4  sequence
//  16   8  capture_time_ns
//  24   8  emit_time_ns
//  32   2  frame_width
//  34   2  frame_height
//  36   1  source_id
//  37   1  reserved
//  38   2  count
//  40   4  reserved
//  44   4  reserved
//  48   N * 12-byte records: { id u16, active u8, flags u8, progress f32, metric f32 }
//   +4  CRC32 over everything before it
//
// Adding a gesture never changes the framing: reserve an id in this header, add
// the detector on the board, and map the id to a name in the PC receiver.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "motion_udp_sender.hpp" // socket + zlib + stdint plumbing

namespace gesture_udp
{
// Gesture ids are the wire contract with the PC (see pc/reference/gesture_packet.py
// GESTURE_IDS, which must carry the same names).
constexpr uint16_t ID_HEAD_PAT = 0;   // touch_head
constexpr uint16_t ID_TOUCH_FACE = 1; // touch_face
constexpr uint16_t ID_HUG = 2;        // hug

constexpr uint8_t FLAG_METRIC_VALID = 1U << 0;

struct Record
{
    uint16_t id = 0;
    uint8_t active = 0;
    uint8_t flags = 0;
    float progress = 0.0f;
    float metric = 0.0f;
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
            last_error_ = "gesture UDP host must be a numeric IPv4 address";
            ::close(socket_);
            socket_ = -1;
        }
        packet_.reserve(64 + 32 * 12);
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
                   const std::vector<Record> &records)
    {
        if (!ready())
            return false;
        if (records.size() > 32)
        {
            last_error_ = "gesture record count exceeds protocol limit";
            return false;
        }

        constexpr uint16_t HEADER_BYTES = 48;
        constexpr uint32_t RECORD_BYTES = 12;
        constexpr uint32_t CRC_BYTES = 4;
        const uint32_t packet_bytes =
            HEADER_BYTES + CRC_BYTES + static_cast<uint32_t>(records.size()) * RECORD_BYTES;

        packet_.clear();
        packet_.insert(packet_.end(), {'G', 'S', 'T', '1'});
        appendU8(1);  // version
        appendU8(0);  // flags
        appendU16(HEADER_BYTES);
        appendU32(packet_bytes);
        appendU32(sequence);
        appendU64(capture_time_ns);
        appendU64(emit_time_ns);
        appendU16(frame_width);
        appendU16(frame_height);
        appendU8(source_id);
        appendU8(0);
        appendU16(static_cast<uint16_t>(records.size()));
        appendU32(0);
        appendU32(0);

        for (const auto &record : records)
        {
            appendU16(record.id);
            appendU8(record.active);
            appendU8(record.flags);
            appendFloat(record.progress);
            appendFloat(record.metric);
        }

        const uint32_t crc = static_cast<uint32_t>(
            ::crc32(0L, reinterpret_cast<const Bytef *>(packet_.data()), packet_.size()));
        appendU32(crc);
        if (packet_.size() != packet_bytes)
        {
            last_error_ = "gesture packet size mismatch";
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
        static_assert(sizeof(float) == sizeof(uint32_t), "GST1 requires 32-bit float");
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        appendU32(bits);
    }

    int socket_ = -1;
    sockaddr_in target_{};
    std::vector<uint8_t> packet_;
    std::string last_error_;
};
} // namespace gesture_udp
