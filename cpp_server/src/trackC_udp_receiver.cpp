#include "trackC_udp_receiver.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;

#pragma pack(push, 1)
struct TrackCPacket {
    std::uint64_t seq;
    double T_ref[16];
};
#pragma pack(pop)

static_assert(sizeof(TrackCPacket) == 136, "TrackCPacket wire size must be 136 bytes");

std::uint64_t nowNs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()
        ).count()
    );
}

bool allFinite(const TrackCPacket& packet) {
    for (size_t i = 0; i < 16; ++i) {
        if (!std::isfinite(packet.T_ref[i])) {
            return false;
        }
    }
    return true;
}

}  // namespace

TrackCUdpReceiver::TrackCUdpReceiver(
    int command_port,
    double stream_hz,
    const std::array<double, 16>& initial_T_ref)
    : stream_hz_(stream_hz),
      initial_T_ref_(initial_T_ref) {

    if (!std::isfinite(stream_hz_) || stream_hz_ <= 0.0) {
        throw std::runtime_error("trackC stream_hz must be positive.");
    }

    writeTarget(0, initial_T_ref_);
    sockfd_ = createSocket(command_port);
}

TrackCUdpReceiver::~TrackCUdpReceiver() {
    stop();
}

int TrackCUdpReceiver::createSocket(int command_port) {
    if (command_port <= 0 || command_port > 65535) {
        throw std::runtime_error("trackC command_port must be in range 1..65535.");
    }

    int sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        throw std::runtime_error(std::string("trackC socket() failed: ") + std::strerror(errno));
    }

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;

    if (::setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        ::close(sockfd);
        throw std::runtime_error(
            std::string("trackC setsockopt(SO_RCVTIMEO) failed: ") + std::strerror(errno)
        );
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<std::uint16_t>(command_port));

    if (::bind(sockfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(sockfd);
        throw std::runtime_error(std::string("trackC bind() failed: ") + std::strerror(errno));
    }

    return sockfd;
}

void TrackCUdpReceiver::start() {
    bool expected = false;
    if (!started_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        throw std::runtime_error("trackC UDP receiver already started.");
    }

    stop_requested_.store(false, std::memory_order_release);
    receiver_thread_ = std::thread(&TrackCUdpReceiver::run, this);
}

void TrackCUdpReceiver::stop() {
    stop_requested_.store(true, std::memory_order_release);

    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }

    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = -1;
    }
}

void TrackCUdpReceiver::setFault(const std::string& reason) {
    bool expected = false;
    if (fault_requested_.compare_exchange_strong(
            expected,
            true,
            std::memory_order_acq_rel)) {
        std::lock_guard<std::mutex> lock(fault_mutex_);
        fault_reason_ = reason;
    }

    stop_requested_.store(true, std::memory_order_release);
}

bool TrackCUdpReceiver::hasFault() const {
    return fault_requested_.load(std::memory_order_acquire);
}

std::string TrackCUdpReceiver::faultReason() const {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    return fault_reason_;
}

void TrackCUdpReceiver::writeTarget(
    std::uint64_t seq,
    const std::array<double, 16>& T_ref) {

    write_generation_.fetch_add(1, std::memory_order_acq_rel);

    const std::uint64_t count =
        write_count_.load(std::memory_order_relaxed);

    AtomicTargetSlot& slot = ring_buffer_[count % kBufferSize];
    for (size_t i = 0; i < 16; ++i) {
        slot.T_ref[i].store(T_ref[i], std::memory_order_relaxed);
    }
    slot.seq.store(seq, std::memory_order_relaxed);
    slot.receive_time_ns.store(nowNs(), std::memory_order_relaxed);
    slot.has_target.store(true, std::memory_order_release);

    write_count_.store(count + 1, std::memory_order_release);
    write_generation_.fetch_add(1, std::memory_order_release);
}

bool TrackCUdpReceiver::getTarget(
    TrackCTargetSnapshot& out,
    std::size_t history_offset) const {

    for (int attempt = 0; attempt < 2; ++attempt) {
        const std::uint64_t gen_before =
            write_generation_.load(std::memory_order_acquire);

        if ((gen_before & 1U) != 0U) {
            continue;
        }

        const std::uint64_t count =
            write_count_.load(std::memory_order_acquire);

        if (count == 0 || history_offset >= count || history_offset >= kBufferSize) {
            return false;
        }

        const std::uint64_t newest_index = count - 1;
        const std::uint64_t target_index = newest_index - history_offset;

        const AtomicTargetSlot& slot = ring_buffer_[target_index % kBufferSize];

        TrackCTargetSnapshot candidate{};
        for (size_t i = 0; i < 16; ++i) {
            candidate.T_ref[i] = slot.T_ref[i].load(std::memory_order_relaxed);
        }
        candidate.seq = slot.seq.load(std::memory_order_relaxed);
        candidate.receive_time_ns =
            slot.receive_time_ns.load(std::memory_order_relaxed);
        candidate.has_target =
            slot.has_target.load(std::memory_order_acquire);

        const std::uint64_t gen_after =
            write_generation_.load(std::memory_order_acquire);

        if (gen_before == gen_after && (gen_after & 1U) == 0U) {
            if (!candidate.has_target) {
                return false;
            }

            out = candidate;
            return true;
        }
    }

    return false;
}

void TrackCUdpReceiver::run() {
    std::uint64_t previous_seq = 0;

    while (!stop_requested_.load(std::memory_order_acquire)) {
        TrackCPacket packet{};

        const ssize_t nbytes = ::recvfrom(
            sockfd_,
            &packet,
            sizeof(packet),
            0,
            nullptr,
            nullptr
        );

        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }

            setFault(std::string("trackC UDP recvfrom failed: ") + std::strerror(errno));
            break;
        }

        if (nbytes != static_cast<ssize_t>(sizeof(TrackCPacket))) {
            continue;
        }

        if (!allFinite(packet)) {
            setFault("trackC UDP rejected packet: T_ref contains non-finite value.");
            break;
        }

        if (packet.seq <= previous_seq) {
            continue;
        }

        std::array<double, 16> T_ref{};
        for (size_t i = 0; i < 16; ++i) {
            T_ref[i] = packet.T_ref[i];
        }

        writeTarget(packet.seq, T_ref);
        previous_seq = packet.seq;
    }
}
