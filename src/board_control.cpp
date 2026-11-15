#include "board_control.hpp"

#include <nlohmann/json.hpp>

#include <sys/socket.h>
#include <sys/un.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class ControlSession final : public MotionTools {
public:
    ControlSession(std::string socket_path, bool standup_only, bool walk_after_standup)
        : socket_path_(std::move(socket_path)), standup_only_(standup_only), walk_after_standup_(walk_after_standup) {
        if (!standup_only_ && !isatty(STDIN_FILENO)) {
            throw std::runtime_error("Run voice.sh control in an interactive SSH terminal");
        }
    }

    ~ControlSession() override {
        if (socket_ >= 0) close(socket_);
    }

    bool standup_only() const override { return standup_only_; }
    bool walk_after_standup() const override { return walk_after_standup_; }

    void update_cloud() override {
        connect_control();
        if (socket_ >= 0 && Clock::now() >= next_heartbeat_) {
            if (send({{"action", "heartbeat"}})) next_heartbeat_ = Clock::now() + 70ms;
        }
        receive();
        if (!standup_only_) read_confirmation();
        if (exiting_) throw std::runtime_error("The confirmation terminal was closed");
    }

    bool request(std::string_view direction) override {
        if (socket_ < 0 || current_) return false;
        current_ = ++next_request_;
        direction_ = direction;
        result_.reset();
        return send({{"action", "request"}, {"request", current_}, {"direction", direction}});
    }

    std::optional<std::string> take_result() override {
        return std::exchange(result_, std::nullopt);
    }

    void cancel_pending() override {
        if (current_ && !sent_confirmation_) {
            send({{"action", "cancel"}, {"request", current_}});
            clear_request();
        }
    }

private:
    void clear_request() {
        current_ = 0;
        direction_.clear();
        token_.clear();
        sent_confirmation_ = false;
        line_.clear();
    }

    void close_socket() {
        if (socket_ >= 0) close(std::exchange(socket_, -1));
        if (current_) result_ = "connection_lost";
        clear_request();
        next_connect_ = Clock::now() + 250ms;
    }

    bool send(const Json& message) {
        if (socket_ < 0) return false;
        const auto wire = message.dump();
        if (::send(socket_, wire.data(), wire.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(wire.size())) return true;
        close_socket();
        return false;
    }

    void connect_control() {
        if (socket_ >= 0 || Clock::now() < next_connect_) return;
        sockaddr_un peer{};
        peer.sun_family = AF_UNIX;
        if (socket_path_.size() >= sizeof(peer.sun_path)) throw std::runtime_error("Control socket path too long");
        std::memcpy(peer.sun_path, socket_path_.c_str(), socket_path_.size() + 1);
        socket_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0 || connect(socket_, reinterpret_cast<const sockaddr*>(&peer), sizeof(peer)) < 0) {
            close_socket();
            return;
        }
        ucred owner{};
        socklen_t length = sizeof(owner);
        if (getsockopt(socket_, SOL_SOCKET, SO_PEERCRED, &owner, &length) || owner.uid != 0) {
            close_socket();
            return;
        }
        next_heartbeat_ = Clock::now();
    }

    void receive() {
        for (unsigned i = 0; i < 8 && socket_ >= 0; ++i) {
            std::array<char, 513> data{};
            const auto size = recv(socket_, data.data(), data.size(), 0);
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
            if (size <= 0 || size > 512) {
                close_socket();
                return;
            }
            const auto message = Json::parse(data.data(), data.data() + size, nullptr, false);
            if (!message.is_object()) throw std::runtime_error("Invalid local control response");
            const auto event = message.value("event", "");
            if (!current_ || message.value("request", std::uint64_t{}) != current_) continue;
            if (event == "confirm" && !standup_only_ && !sent_confirmation_) {
                const auto direction = message.value("direction", "");
                token_ = message.value("token", "");
                if (token_.empty() || direction != direction_) {
                    throw std::runtime_error("Invalid local action confirmation");
                }
                const auto* label = direction == "forward" ? "前进" :
                                    direction == "left" ? "左转" : "右转";
                if (tcflush(STDIN_FILENO, TCIFLUSH) != 0) {
                    throw std::runtime_error("Could not clear buffered terminal input");
                }
                line_.clear();
                std::cout << "待确认 " << label << "（0.4 秒）。10 秒内输入 确认 并回车。\n" << std::flush;
            } else if (event == "result") {
                result_ = message.value("status", "unknown");
                std::cout << "动作结果 " << *result_ << '\n' << std::flush;
                clear_request();
            }
        }
    }

    void read_confirmation() {
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        if (poll(&descriptor, 1, 0) <= 0) return;
        std::array<char, 4096> data{};
        const auto count = read(STDIN_FILENO, data.data(), data.size());
        if (count < 0 && errno == EINTR) return;
        if (count <= 0) {
            exiting_ = true;
            return;
        }
        if (token_.empty()) return;
        line_.append(data.data(), static_cast<std::size_t>(count));
        const auto newline = line_.find('\n');
        if (newline == std::string::npos) {
            if (line_.size() > 64) line_ = "invalid";
            return;
        }
        auto value = line_.substr(0, newline);
        line_.clear();
        if (!value.empty() && value.back() == '\r') value.pop_back();
        if (value == "确认") {
            sent_confirmation_ = true;
            send({{"action", "confirm"}, {"request", current_}, {"token", std::exchange(token_, std::string{})}});
        }
    }

    std::string socket_path_;
    bool standup_only_;
    bool walk_after_standup_;
    std::uint64_t current_ = 0, next_request_ = 0;
    Clock::time_point next_heartbeat_{}, next_connect_{};
    std::optional<std::string> result_;
    std::string direction_, token_, line_;
    bool exiting_ = false, sent_confirmation_ = false;
    int socket_ = -1;
};
}

std::unique_ptr<MotionTools> make_board_control(const std::string& socket_path, bool standup_only,
                                                bool walk_after_standup) {
    return std::make_unique<ControlSession>(socket_path, standup_only, walk_after_standup);
}
