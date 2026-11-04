#include "finger_udp_sender.hpp"
#include <cstdlib>
#include <iostream>
#include <iomanip>

uint16_t listenLocal(int &fd) {
    fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (fd < 0 || ::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) std::exit(2);
    socklen_t bytes = sizeof(addr);
    if (::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &bytes) != 0) std::exit(2);
    timeval timeout{3, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return ntohs(addr.sin_port);
}

void printPacket(int fd, const char *label) {
    uint8_t data[2048];
    const auto size = ::recv(fd, data, sizeof(data), 0);
    if (size < 0) std::exit(7);
    std::cout << label << " ";
    for (int i = 0; i < size; ++i)
        std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(data[i]);
    std::cout << std::dec << "\n";
}

int main() {
    int finger_fd, motion_fd;
    finger_udp::Sender sender("127.0.0.1", listenLocal(finger_fd));
    motion_udp::Sender motion("127.0.0.1", listenLocal(motion_fd));
    finger_udp::Detection hand;
    hand.score = 0.75f;
    hand.box = {0.1f, 0.2f, 0.3f, 0.4f};
    for (int k = 0; k < 21; ++k) hand.keypoints.push_back({k / 20.0f, 0.5f, 0.9f});
    for (uint32_t count = 0; count <= 2; ++count) {
        if (!sender.sendFrame(count, 123456789, 123456999, 1920, 1080, 0,
                              std::vector<finger_udp::Detection>(count, hand))) return 3;
        if (!motion.sendFrame(count, 123456789, 123456999, 1920, 1080, 0, nullptr, nullptr)) return 4;
        printPacket(finger_fd, "FNG1");
        printPacket(motion_fd, "DMV1");
    }
    hand.keypoints.pop_back();
    if (sender.sendFrame(3, 0, 0, 1920, 1080, 0, {hand})) return 5;
    hand.keypoints.push_back({0.1f, 0.2f, 0.3f});
    if (sender.sendFrame(3, 0, 0, 1920, 1080, 0, {hand, hand, hand})) return 6;
    ::close(finger_fd);
    ::close(motion_fd);
    std::cout << "C++ FNG1/DMV1 sender passed\n";
}
