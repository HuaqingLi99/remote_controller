#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

struct TrackJTargetSnapshot {
    std::array<double, 7> q_ref{};
    std::uint64_t seq{0};
    std::uint64_t receive_time_ns{0};
    bool has_target{false};
};

class TrackJUdpReceiver {
public:
    TrackJUdpReceiver(
        int command_port,
        double stream_hz,
        const std::array<double, 7>& initial_q_ref);

    ~TrackJUdpReceiver();

    TrackJUdpReceiver(const TrackJUdpReceiver&) = delete;
    TrackJUdpReceiver& operator=(const TrackJUdpReceiver&) = delete;

    void start();
    void stop();

    bool getTarget(
        TrackJTargetSnapshot& out,
        std::size_t history_offset = 0) const;

    bool getLatestTarget(TrackJTargetSnapshot& out) const {
        return getTarget(out, 0);
    }

    bool hasFault() const;
    std::string faultReason() const;

private:
    static constexpr std::size_t kBufferSize = 16;

    struct AtomicTargetSlot {
        AtomicTargetSlot() : seq(0), receive_time_ns(0), has_target(false) {
            for (auto& value : q_ref) {
                value.store(0.0, std::memory_order_relaxed);
            }
        }

        std::array<std::atomic<double>, 7> q_ref;
        std::atomic<std::uint64_t> seq;
        std::atomic<std::uint64_t> receive_time_ns;
        std::atomic<bool> has_target;
    };

    void run();
    void setFault(const std::string& reason);
    void writeTarget(std::uint64_t seq, const std::array<double, 7>& q_ref);
    int createSocket(int command_port);

    int sockfd_{-1};
    double stream_hz_{0.0};
    std::array<double, 7> initial_q_ref_{};

    std::thread receiver_thread_;
    std::atomic<bool> started_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> fault_requested_{false};

    mutable std::mutex fault_mutex_;
    std::string fault_reason_;

    std::array<AtomicTargetSlot, kBufferSize> ring_buffer_{};
    std::atomic<std::uint64_t> write_count_{0};
    std::atomic<std::uint64_t> write_generation_{0};
};
