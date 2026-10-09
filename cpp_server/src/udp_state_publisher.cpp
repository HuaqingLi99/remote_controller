#include "udp_state_publisher.h"

/**
 * Small UDP sender for robot_state_udp snapshots.
 *
 * The server uses this only for best-effort state streaming to the Python
 * client. Motion commands and gripper commands are still handled through
 * XML-RPC; UDP packets are fixed-size snapshots, not command messages.
 */

UdpStatePublisher:: UdpStatePublisher(const std::string& ip, int port)
        : sockfd_(-1), ip_(ip), port_(port) {
        if(port_ <= 0 || port_ > 65535) {
            throw std::runtime_error("Invalid port number");
        }
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        std::memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(port_);


        //convert IP address from string to binary form
        int ret = inet_pton(AF_INET, ip_.c_str(), &server_addr_.sin_addr);
        if (ret <= 0) {
            close(sockfd_);
            sockfd_ = -1;
            throw std::runtime_error("Invalid IP address");
        }
    }

UdpStatePublisher::~UdpStatePublisher() {
        if (sockfd_ >= 0) {
            close(sockfd_);
            sockfd_ = -1;
        }
    }
void UdpStatePublisher::publish(const robot_state_udp& state) const {
        // send the state as udp packet to the server_addr_
        ssize_t sent_bytes = sendto(
            sockfd_,
            &state,
            sizeof(state),
            0,
            reinterpret_cast<const struct sockaddr*>(&server_addr_),//turn server_addr_ to sockaddr pointer
            sizeof(server_addr_));

        if (sent_bytes < 0) {
            std::cerr << "Failed to send UDP packet" << std::endl;
        }else if (sent_bytes != static_cast<ssize_t>(sizeof(state)))

            std::cerr << "Partial UDP packet sent" << std::endl;
        }
    
