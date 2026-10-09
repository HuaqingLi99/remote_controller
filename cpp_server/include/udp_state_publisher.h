#pragma once

#include <cstdint>
#include <netinet/in.h>
#include <string>

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>


#pragma pack(push, 1)
/**
 * Fixed-size UDP state packet.
 *
 * Layout:
 * - q[7]: joint position in rad
 * - dq[7]: joint velocity in rad/s
 * - K_F_ext_hat[6]: estimated external wrench from Franka state
 * - arm_state: arm_state enum value
 * - gripper_state: gripper_state enum value
 */
struct robot_state_udp {
    double q[7];
    double dq[7];
    double K_F_ext_hat[6];
    double tau_ext[7];
    int32_t arm_state;
    int32_t gripper_state;
};
#pragma pack(pop)
// 168 + 7*8 = 224 bytes
static_assert(sizeof(robot_state_udp) == 224, "robot_state_udp wire size changed"); 
/**
 * Best-effort UDP publisher for robot_state_udp snapshots.
 *
 * This is used only for state feedback. Commands still go through XML-RPC.
 */
class UdpStatePublisher {
public:
    /**
     * Create a UDP sender for one target IP and port.
     */
    UdpStatePublisher(const std::string& ip, int port);
    ~UdpStatePublisher();

    /**
     * Send one robot_state_udp snapshot.
     */
    void publish(const robot_state_udp& state) const;

private:
    int sockfd_;
    struct sockaddr_in server_addr_;
    std::string ip_;
    int port_;
};
