#include "motion_udp_sender.hpp"

#include <arpa/inet.h>
#include <cassert>
#include <cstdint>
#include <sys/socket.h>
#include <unistd.h>

int main()
{
    const int receiver = ::socket(AF_INET, SOCK_DGRAM, 0);
    assert(receiver >= 0);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(39541);
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(::bind(receiver, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) == 0);

    motion_udp::Detection body;
    body.score = 0.9f;
    body.box = {0.1f, 0.2f, 0.3f, 0.4f};
    body.keypoints.resize(motion_udp::POSE_COUNT, {0.5f, 0.5f, 0.9f});
    motion_udp::Detection face = body;
    face.keypoints.resize(motion_udp::FACE_COUNT, {0.5f, 0.5f, 0.9f});

    motion_udp::Sender sender("127.0.0.1", 39541);
    assert(sender.ready());
    assert(sender.sendFrame(7, 100, 200, 1920, 1080, 0, &body, &face, 2));

    uint8_t packet[904]{};
    const auto bytes = ::recv(receiver, packet, sizeof(packet), 0);
    assert(bytes == 904);
    assert(packet[0] == 'D' && packet[1] == 'M' && packet[2] == 'V' && packet[3] == '2');
    assert(packet[36] == motion_udp::POSE_COUNT && packet[37] == motion_udp::FACE_COUNT);
    assert(packet[40] == 2 && packet[41] == 0 && packet[42] == 0 && packet[43] == 0);

    const uint32_t expected_crc = static_cast<uint32_t>(
        ::crc32(0L, reinterpret_cast<const Bytef *>(packet), sizeof(packet) - 4));
    const uint32_t packet_crc = static_cast<uint32_t>(packet[900]) |
                                (static_cast<uint32_t>(packet[901]) << 8) |
                                (static_cast<uint32_t>(packet[902]) << 16) |
                                (static_cast<uint32_t>(packet[903]) << 24);
    assert(packet_crc == expected_crc);
    ::close(receiver);
    return 0;
}
