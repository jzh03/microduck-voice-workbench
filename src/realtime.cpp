#include "realtime.hpp"
#include "audio.hpp"
#include "motion_tools.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unordered_set>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#endif

namespace {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr std::string_view base64 =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
constexpr std::string_view model = "qwen3.8-omni-flash-realtime";
constexpr std::size_t max_event_bytes = 2 * 1024 * 1024;

std::string encode_pcm(const Samples& samples) {
    const auto bytes = std::as_bytes(std::span(samples));
    std::string result;
    result.reserve((bytes.size() + 2) / 3 * 4);
    for (std::size_t index = 0; index < bytes.size(); index += 3) {
        const bool second = index + 1 < bytes.size();
        const bool third = index + 2 < bytes.size();
        const unsigned a = std::to_integer<unsigned>(bytes[index]);
        const unsigned b = second ? std::to_integer<unsigned>(bytes[index + 1]) : 0;
        const unsigned c = third ? std::to_integer<unsigned>(bytes[index + 2]) : 0;
        result += base64[a >> 2];
        result += base64[(a << 4 | b >> 4) & 63];
        result += second ? base64[(b << 2 | c >> 6) & 63] : '=';
        result += third ? base64[c & 63] : '=';
    }
    return result;
}

unsigned base64_digit(char digit) {
    const auto value = base64.find(digit);
    if (value == std::string_view::npos) throw std::runtime_error("Invalid base64 audio");
    return static_cast<unsigned>(value);
}

Samples decode_pcm(std::string_view text) {
    if (text.size() % 4 != 0) throw std::runtime_error("Invalid base64 audio length");
    std::vector<unsigned char> bytes;
    bytes.reserve(text.size() / 4 * 3);
    for (std::size_t index = 0; index < text.size(); index += 4) {
        const bool third = text[index + 2] != '=';
        const bool fourth = text[index + 3] != '=';
        if ((!third && fourth) || ((!third || !fourth) && index + 4 != text.size())) {
            throw std::runtime_error("Invalid base64 audio padding");
        }
        const auto a = base64_digit(text[index]);
        const auto b = base64_digit(text[index + 1]);
        const auto c = third ? base64_digit(text[index + 2]) : 0;
        const auto d = fourth ? base64_digit(text[index + 3]) : 0;
        bytes.push_back(static_cast<unsigned char>(a << 2 | b >> 4));
        if (third) bytes.push_back(static_cast<unsigned char>(b << 4 | c >> 2));
        if (fourth) bytes.push_back(static_cast<unsigned char>(c << 6 | d));
    }
    if (bytes.size() % 2 != 0) throw std::runtime_error("Incomplete PCM sample");
    Samples samples(bytes.size() / 2);
    if (!bytes.empty()) std::memcpy(samples.data(), bytes.data(), bytes.size());
    return samples;
}

void check(CURLcode result, std::string_view operation) {
    if (result != CURLE_OK) {
        throw std::runtime_error(std::string(operation) + ": " + curl_easy_strerror(result));
    }
}

class CurlRuntime {
public:
    CurlRuntime() { check(curl_global_init(CURL_GLOBAL_DEFAULT), "Initialize libcurl"); }
    ~CurlRuntime() { curl_global_cleanup(); }
};

std::string model_url(const std::string& url) {
    const std::unique_ptr<CURLU, decltype(&curl_url_cleanup)> parsed(curl_url(), curl_url_cleanup);
    if (!parsed || curl_url_set(parsed.get(), CURLUPART_URL, url.c_str(), CURLU_DISALLOW_USER) != CURLUE_OK) {
        throw std::runtime_error("Invalid Realtime WebSocket URL");
    }
    char* raw_query = nullptr;
    const auto query_status = curl_url_get(parsed.get(), CURLUPART_QUERY, &raw_query, 0);
    const std::unique_ptr<char, decltype(&curl_free)> existing_query(raw_query, curl_free);
    if (query_status != CURLUE_OK && query_status != CURLUE_NO_QUERY) {
        throw std::runtime_error("Invalid Realtime WebSocket query");
    }
    std::string query = "model=" + std::string(model);
    std::string_view rest = existing_query ? existing_query.get() : "";
    while (!rest.empty()) {
        const auto delimiter = rest.find('&');
        const auto field = rest.substr(0, delimiter);
        if (!field.empty() && field.substr(0, field.find('=')) != "model") {
            query += '&';
            query += field;
        }
        if (delimiter == std::string_view::npos) break;
        rest.remove_prefix(delimiter + 1);
    }
    char* raw_url = nullptr;
    if (curl_url_set(parsed.get(), CURLUPART_QUERY, query.c_str(), 0) != CURLUE_OK ||
        curl_url_get(parsed.get(), CURLUPART_URL, &raw_url, 0) != CURLUE_OK) {
        throw std::runtime_error("Could not set the Realtime model in the WebSocket URL");
    }
    const std::unique_ptr<char, decltype(&curl_free)> resolved(raw_url, curl_free);
    return resolved.get();
}

class WebSocket {
public:
    WebSocket(const std::string& url, const std::string& key) {
        if (!handle_) throw std::runtime_error("Could not allocate WebSocket");
        if (!key.empty()) {
            if (key.find_first_of("\r\n") != std::string::npos) {
                throw std::runtime_error("Invalid API key");
            }
            headers_.reset(curl_slist_append(nullptr, ("Authorization: Bearer " + key).c_str()));
            if (!headers_) throw std::runtime_error("Could not allocate WebSocket headers");
            set(CURLOPT_HTTPHEADER, headers_.get());
#ifdef _WIN32
            set(CURLOPT_SSL_OPTIONS, static_cast<long>(CURLSSLOPT_NATIVE_CA));
#endif
        } else {
            set(CURLOPT_NOPROXY, "*");
        }
        set(CURLOPT_PROTOCOLS_STR, key.empty() ? "ws" : "wss");
        set(CURLOPT_DISALLOW_USERNAME_IN_URL, 1L);
        const auto resolved_url = model_url(url);
        set(CURLOPT_URL, resolved_url.c_str());
        set(CURLOPT_CONNECT_ONLY, 2L);
        set(CURLOPT_CONNECTTIMEOUT_MS, 8000L);
        set(CURLOPT_TIMEOUT_MS, 12000L);
        set(CURLOPT_NOSIGNAL, 1L);
        const auto result = curl_easy_perform(handle_.get());
        if (result != CURLE_OK) {
            long status = 0;
            curl_easy_getinfo(handle_.get(), CURLINFO_RESPONSE_CODE, &status);
            check(result, "WebSocket handshake (HTTP " + std::to_string(status) + ")");
        }
    }

    void send(const Json& event, std::chrono::milliseconds timeout = 3s) {
        const auto text = event.dump();
        const auto deadline = Clock::now() + timeout;
        std::size_t offset = 0;
        while (offset < text.size()) {
            std::size_t written = 0;
            const auto result = curl_ws_send(handle_.get(), text.data() + offset,
                                             text.size() - offset, &written, 0, CURLWS_TEXT);
            if (result != CURLE_AGAIN) check(result, "Send WebSocket event");
            offset += written;
            if (offset == text.size()) return;
            if (Clock::now() >= deadline) throw std::runtime_error("WebSocket send timed out");
            if (written == 0) std::this_thread::sleep_for(2ms);
        }
    }

    std::optional<Json> receive() {
        std::array<char, 16384> buffer{};
        for (;;) {
            const curl_ws_frame* frame = nullptr;
            std::size_t size = 0;
            const auto result = curl_ws_recv(handle_.get(), buffer.data(), buffer.size(), &size, &frame);
            if (result == CURLE_AGAIN) return std::nullopt;
            check(result, "Receive WebSocket event");
            if (frame->flags & CURLWS_CLOSE) throw std::runtime_error("WebSocket was closed");
            if (frame->flags & (CURLWS_PING | CURLWS_PONG)) continue;
            if (!(frame->flags & CURLWS_TEXT)) throw std::runtime_error("Expected a JSON text frame");
            if (message_.size() + size > max_event_bytes) {
                throw std::runtime_error("WebSocket event is too large");
            }
            message_.append(buffer.data(), size);
            if (frame->bytesleft == 0 && !(frame->flags & CURLWS_CONT)) {
                return Json::parse(std::exchange(message_, std::string{}));
            }
        }
    }

    void close() {
        const std::array<unsigned char, 2> normal_close{0x03, 0xe8};
        std::size_t sent = 0;
        curl_ws_send(handle_.get(), normal_close.data(), normal_close.size(), &sent, 0, CURLWS_CLOSE);
    }

private:
    template<class T>
    void set(CURLoption option, T value) {
        check(curl_easy_setopt(handle_.get(), option, value), "Configure WebSocket");
    }

    CurlRuntime runtime_;
    std::unique_ptr<curl_slist, decltype(&curl_slist_free_all)> headers_{nullptr, curl_slist_free_all};
    std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> handle_{curl_easy_init(), curl_easy_cleanup};
    std::string message_;
};

constexpr std::string_view vad_mode = "server_vad";

Json session_update(bool duplex, const MotionTools* motion) {
    Json detection = {{"type", vad_mode}, {"silence_duration_ms", vad_silence_ms}};
    if (duplex) detection["threshold"] = 0.50;
    Json event = {
        {"type", "session.update"},
        {"session", {
            {"modalities", {"text", "audio"}},
            {"voice", "Tina"},
            {"turn_detection", std::move(detection)}
        }}
    };
    auto& session = event["session"];
    session["instructions"] = std::string("你是 Microduck，用自然简短的语音回应用户，并参考本会话前几轮对话。") +
        "你无法通过语音读取机器人状态，不要猜测状态。";
    if (motion && motion->standup_only()) {
        session["instructions"] = session["instructions"].get<std::string>() +
            "用户明确要求你从地面起身时，调用 stand_up 一次。板端收到指令一秒后尝试起身。"
            "刚结束一次起身后，用户说再试一次表示重新申请 stand_up。"
            "请等待工具结果再报告动作状态。standing_balanced 表示板端已进入零速站立平衡。"
            "handoff_blocked 表示策略接管被拦截。"
            "standup_simulated 表示只完成了模拟，没有驱动舵机。";
        session["tools"] = Json::array({{
            {"type", "function"}, {"name", "stand_up"},
            {"description", "仅在用户明确发出地面起身指令时请求起身。"},
            {"parameters", {{"type", "object"}, {"properties", Json::object()}, {"additionalProperties", false}}}
        }});
        if (motion->walk_after_standup()) {
            session["instructions"] = session["instructions"].get<std::string>() +
                "收到 ready_to_walk 或 standup_simulated 后等待用户明确要求前进，才调用 step_forward。"
                "前进指令延迟一秒执行，以最高 0.5 米每秒行走最多 5 秒，最后一秒逐步减到零。"
                "forward_step_completed 表示前进指令已归零，机器人继续保持平衡。"
                "forward_step_simulated 表示仅完成模拟。";
            session["tools"].push_back({
                {"type", "function"}, {"name", "step_forward"},
                {"description", "在小鸭子已站稳且用户明确说往前走时，申请一次预设前进。"},
                {"parameters", {{"type", "object"}, {"properties", Json::object()}, {"additionalProperties", false}}}
            });
        }
    } else if (motion) {
        session["instructions"] = session["instructions"].get<std::string>() +
            "用户明确要求前进、左转或右转时，可申请一次预设短动作。确认由用户在终端输入。"
            "不要代替用户确认，不要连续申请。只有收到 completed 才可说明控制端已结束本次动作。"
            "连接断开或结果未知时，如实说明未能确认。";
        session["tools"] = Json::array({{
            {"type", "function"}, {"name", "move_short"},
            {"description", "申请一次预设短动作，等待操作者在终端确认。"},
            {"parameters", {{"type", "object"}, {"additionalProperties", false},
                {"required", {"direction"}}, {"properties", {{"direction", {
                    {"type", "string"}, {"enum", {"forward", "left", "right"}}
                }}}}}}
        }});
    }
    return event;
}

const Json& member(const Json& object, const char* name) {
    static const Json missing;
    const auto it = object.find(name);
    return it == object.end() ? missing : *it;
}

std::string string_member(const Json& object, const char* name) {
    const auto& value = member(object, name);
    return value.is_string() ? value.get<std::string>() : std::string{};
}

void print_transcript(const std::string& transcript) {
    const auto line = "Microduck: " + transcript;
#ifdef _WIN32
    // The service sends UTF-8. Use Unicode directly for an attached console,
    // and keep the UTF-8 byte stream when stdout is redirected.
    const auto output = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (GetConsoleMode(output, &mode)) {
        const auto bytes = static_cast<int>(line.size());
        const int length = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                                               line.data(), bytes, nullptr, 0);
        if (length > 0) {
            std::wstring wide(length, L'\0');
            if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, line.data(), bytes,
                                    wide.data(), length) == length) {
                wide += L"\r\n";
                std::cout.flush();
                DWORD written = 0;
                if (WriteConsoleW(output, wide.data(), static_cast<DWORD>(wide.size()),
                                  &written, nullptr)) return;
            }
        }
    }
#endif
    std::cout << line << '\n' << std::flush;
}

void verify_pcm(const Json& session, const char* direction, const char* legacy_name,
                unsigned expected_rate) {
    const auto& audio = member(session, "audio");
    const auto& format = member(member(audio, direction), "format");
    const auto& type = member(format, "type");
    const auto& rate = member(format, "sample_rate");
    // Qwen 3.8 uses 16 kHz input and 24 kHz output PCM by default.
    const auto& legacy_type = member(session, legacy_name);
    if ((!format.is_null() && !format.is_object()) ||
        (!type.is_null() ? type != "pcm" : (!legacy_type.is_null() && legacy_type != "pcm")) ||
        (!rate.is_null() && (!rate.is_number_integer() || rate != expected_rate))) {
        throw std::runtime_error("session.updated returned incompatible " + std::string(direction) +
                                 " audio, expected PCM at " + std::to_string(expected_rate) + " Hz");
    }
}

void verify_session(const Json& session) {
    if (!session.is_object()) throw std::runtime_error("session.updated did not include a session");
    const auto reported_model = string_member(session, "model");
    if (!reported_model.empty() && reported_model != model &&
        !reported_model.starts_with(std::string(model) + "-")) {
        throw std::runtime_error("session.updated returned a different model than requested");
    }
    verify_pcm(session, "input", "input_audio_format", input_rate);
    verify_pcm(session, "output", "output_audio_format", output_rate);
    const auto& detection = member(session, "turn_detection");
    if (session.contains("turn_detection") &&
        (!detection.is_object() ||
         (detection.contains("type") && member(detection, "type") != vad_mode))) {
        throw std::runtime_error("session.updated did not confirm turn_detection.type=" +
                                 std::string(vad_mode));
    }
}

enum class Stage { connecting, configuring, listening, waiting, responding, draining, finished };

std::string event_error(const Json& event, const Json& error, std::string_view api_key,
                        std::string_view stage) {
    std::string result = "DashScope " + std::string(stage);
    const auto add = [&result](std::string_view label, const std::string& value) {
        if (!value.empty()) result += " " + std::string(label) + "=" + value;
    };
    add("event", string_member(error, "event"));
    auto code = string_member(error, "code");
    if (code.empty()) code = string_member(error, "error_code");
    add("code", code);
    add("param", string_member(error, "param"));
    add("event_id", string_member(event, "event_id"));
    auto request = string_member(event, "request_id");
    if (request.empty()) request = string_member(error, "request_id");
    if (request.empty()) request = string_member(error, "task_id");
    add("request_id", request);
    auto message = string_member(error, "message");
    if (message.empty()) message = string_member(error, "error_message");
    if (message.empty()) message = string_member(event, "error");
    add("message", message);
    if (!api_key.empty()) {
        for (std::size_t pos = 0; (pos = result.find(api_key, pos)) != std::string::npos;) {
            result.replace(pos, api_key.size(), "[redacted]");
            pos += std::string_view("[redacted]").size();
        }
    }
    for (auto& character : result) {
        const auto byte = static_cast<unsigned char>(character);
        if (byte < 32 || byte == 127) character = ' ';
    }
    if (result.size() > 512) {
        std::size_t end = 512;
        while ((static_cast<unsigned char>(result[end]) & 0xc0) == 0x80) --end;
        result.resize(end);
        result += "...";
    }
    return result;
}

class Conversation {
public:
    Conversation(WebSocket& socket, Audio& audio, std::string_view api_key, MotionTools* motion)
        : socket_(socket), audio_(audio), api_key_(api_key), duplex_(audio.full_duplex()),
          motion_(motion) {}

    void handle(const Json& event) {
        const auto type = string_member(event, "type");
        if (type == "error") {
            const auto& error = member(event, "error");
            if (duplex_ && cancel_race(event, error)) return;
            if (expired_tool_receipt(error)) return;
            throw std::runtime_error(event_error(event, error.is_object() ? error : event,
                                                 api_key_, stage_name()));
        }
        if (type.empty()) {
            const auto& header = member(event, "header");
            const auto& error = member(event, "error");
            if (header.is_object()) {
                throw std::runtime_error(event_error(event, header, api_key_, stage_name()) +
                                         " (DashScope gateway envelope; check the Realtime URL and model)");
            }
            if (!error.is_null() || event.contains("code") || event.contains("error_code") ||
                event.contains("message") || event.contains("error_message")) {
                throw std::runtime_error(event_error(event, error.is_object() ? error : event,
                                                     api_key_, stage_name()));
            }
            throw std::runtime_error(std::string(stage_name()) +
                                     ": expected a Realtime event with a string 'type'");
        }
        if (stage_ == Stage::connecting && type == "session.created") {
            socket_.send(session_update(duplex_, motion_));
            stage_ = Stage::configuring;
            deadline_ = Clock::now() + 10s;
        } else if (stage_ == Stage::configuring && type == "session.updated") {
            verify_session(member(event, "session"));
            next_turn();
        } else if (duplex_) {
            handle_duplex(type, event);
        } else if (stage_ == Stage::listening && type == "input_audio_buffer.speech_started") {
            deadline_ = Clock::now() + 65s;
        } else if (stage_ == Stage::listening &&
                   (type == "input_audio_buffer.speech_stopped" ||
                    type == "input_audio_buffer.committed")) {
            wait_for_reply();
        } else if ((stage_ == Stage::waiting || stage_ == Stage::listening) &&
                   type == "response.created") {
            if (stage_ == Stage::listening) wait_for_reply();
            response_id_ = event.at("response").at("id").get<std::string>();
            has_audio_ = false;
            stage_ = Stage::responding;
        } else if (stage_ == Stage::responding) {
            handle_reply(type, event);
        }
    }

    void tick() {
        const auto now = Clock::now();
        if (deadline_ && now >= *deadline_) {
            throw std::runtime_error("Timed out waiting for a Realtime event or playback");
        }
        if (duplex_) audio_.pump_playback();
        if (stage_ == Stage::listening ||
            (duplex_ && stage_ >= Stage::waiting && stage_ <= Stage::draining)) {
            auto samples = audio_.capture();
            if (!samples.empty()) {
                if (standup_input_paused()) std::fill(samples.begin(), samples.end(), 0);
                socket_.send({{"type", "input_audio_buffer.append"}, {"audio", encode_pcm(samples)}},
                             duplex_ ? 250ms : 3s);
            }
            if (audio_.input_ended() && !input_ended_) input_ended_ = now;
            if (input_ended_ && now - *input_ended_ > 10s) {
                throw std::runtime_error("Qwen did not detect a speech turn in the WAV");
            }
        }
        if (!duplex_) audio_.pump_playback();
        if (stage_ == Stage::draining && audio_.finish_playback()) {
            if (duplex_) listen();
            else next_turn();
        }
    }

    void poll_motion() {
        if (!motion_) return;
        if (const auto result = motion_->take_result(); result && !awaiting_call_.empty()) {
            send_tool_result(awaiting_call_, *result);
            awaiting_call_.clear();
            if (!speaking_ && stage_ == Stage::waiting && resume_after_tools_) {
                resume_after_tools_ = false;
                socket_.send({{"type", "response.create"}}, 250ms);
                deadline_ = Clock::now() + 30s;
            }
        }
    }

    bool ready_for_motion_input() const { return stage_ >= Stage::listening; }

    bool finished() const { return stage_ == Stage::finished; }

private:
    bool standup_input_paused() const {
        return motion_ && motion_->standup_only() &&
            (stage_ != Stage::listening || Clock::now() < listen_after_);
    }

    void send_tool_result(const std::string& id, std::string_view status) {
        if (tool_receipts_.size() == 32) tool_receipts_.erase(tool_receipts_.begin());
        tool_receipts_.insert(id);
        socket_.send({{"type", "conversation.item.create"}, {"item", {
            {"type", "function_call_output"}, {"call_id", id},
            {"output", Json{{"status", status}}.dump()}
        }}}, 250ms);
    }

    bool expired_tool_receipt(const Json& error) {
        constexpr std::string_view prefix = "Unknown function call id: ";
        const auto message = string_member(error, "message");
        if (!motion_ || !message.starts_with(prefix) ||
            (string_member(error, "param") != "call_id") ||
            !tool_receipts_.erase(message.substr(prefix.size()))) return false;
        std::cerr << "百炼已丢弃上一条动作回执，请以终端动作结果为准。\n" << std::flush;
        if (stage_ == Stage::waiting && awaiting_call_.empty()) listen();
        return true;
    }

    void finish_tools() {
        // Reserve a single proposal for this user turn. Any other call only receives a result.
        for (const auto& [id, direction] : pending_calls_) {
            if (direction.empty()) send_tool_result(id, "invalid_tool_or_arguments");
            else if (!response_allows_motion_) send_tool_result(id, "request_not_allowed_in_this_response");
            else {
                response_allows_motion_ = false;
                if (motion_->request(direction)) awaiting_call_ = id;
                else send_tool_result(id, "control_unavailable_or_busy");
            }
        }
        pending_calls_.clear();
        stage_ = Stage::waiting;
        if (motion_->standup_only() && !awaiting_call_.empty()) deadline_.reset();
        else deadline_ = Clock::now() + 30s;
        if (awaiting_call_.empty() && resume_after_tools_) {
            resume_after_tools_ = false;
            socket_.send({{"type", "response.create"}}, 250ms);
        } else if (awaiting_call_.empty()) {
            listen();
        }
    }

    bool cancel_race(const Json& event, const Json& error) const {
        if (cancel_events_.empty()) return false;
        const auto code = string_member(error, "code");
        if (code == "response_cancel_not_active" || code == "response_already_cancelled") return true;
        const auto id = string_member(error, "event_id");
        if (!cancel_events_.contains(id) &&
            !cancel_events_.contains(string_member(event, "event_id"))) {
            return false;
        }
        auto message = string_member(error, "message");
        for (auto& ch : message) {
            if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
        }
        return message.find("no active response") != std::string::npos ||
               message.find("no response to cancel") != std::string::npos ||
               message.find("already completed") != std::string::npos ||
               message.find("already cancelled") != std::string::npos ||
               message.find("already canceled") != std::string::npos;
    }

    void handle_duplex(const std::string& type, const Json& event) {
        if (type == "input_audio_buffer.speech_started") {
            if (standup_input_paused()) return;
            const auto item = string_member(event, "item_id");
            if (speaking_ || (!item.empty() && item == speech_item_)) return;
            speaking_ = true;
            speech_item_ = item;
            if (motion_) {
                motion_->cancel_pending();
                if (!awaiting_call_.empty()) send_tool_result(awaiting_call_, "user_interrupted");
                awaiting_call_.clear();
                pending_calls_.clear();
                can_request_motion_ = true;
                resume_after_tools_ = true;
            }
            if (stage_ == Stage::responding || stage_ == Stage::draining) {
                interrupt_reply(stage_ == Stage::responding);
            }
            stage_ = Stage::listening;
            deadline_ = Clock::now() + 65s;
        } else if (type == "input_audio_buffer.speech_stopped" ||
                   type == "input_audio_buffer.committed") {
            if (motion_ && motion_->standup_only() && !speaking_) return;
            const auto item = string_member(event, "item_id");
            if (stage_ != Stage::listening ||
                (speaking_ && !item.empty() && !speech_item_.empty() && item != speech_item_)) return;
            speaking_ = false;
            stage_ = Stage::waiting;
            deadline_ = Clock::now() + 30s;
        } else if (type == "response.created") {
            const auto id = string_member(member(event, "response"), "id");
            if (id.empty()) throw std::runtime_error("response.created is missing a response ID");
            if (!seen_responses_.insert(id).second) return;
            if (speaking_) {
                response_id_ = id;
                interrupt_reply(true);
                return;
            }
            if (!awaiting_call_.empty()) return;
            if (stage_ != Stage::waiting && stage_ != Stage::listening) return;
            response_id_ = id;
            response_allows_motion_ = std::exchange(can_request_motion_, false);
            has_audio_ = false;
            stage_ = Stage::responding;
            deadline_ = Clock::now() + 30s;
        } else if (stage_ == Stage::responding) {
            handle_reply(type, event);
        }
    }

    void interrupt_reply(bool generating) {
        audio_.interrupt_playback();
        response_id_.clear();
        std::cout << "Interrupted playback\n" << std::flush;
        if (generating) {
            const auto event_id = "microduck-cancel-" + std::to_string(cancel_events_.size() + 1);
            cancel_events_.insert(event_id);
            socket_.send({{"type", "response.cancel"}, {"event_id", event_id}}, 250ms);
        }
    }

    std::string_view stage_name() const {
        if (stage_ == Stage::connecting) return "while waiting for session.created";
        if (stage_ == Stage::configuring) return "while waiting for session.updated";
        return "during the conversation";
    }

    void next_turn() {
        if (audio_.start_turn()) {
            listen();
        } else {
            stage_ = Stage::finished;
        }
        deadline_.reset();
        input_ended_.reset();
    }

    void listen() {
        if (motion_ && motion_->standup_only() && stage_ != Stage::configuring) {
            listen_after_ = Clock::now() + 350ms;
        }
        stage_ = Stage::listening;
        deadline_.reset();
        std::cout << "Listening (" << ++turn_ << ")\n" << std::flush;
    }

    void wait_for_reply() {
        audio_.pause_capture();
        stage_ = Stage::waiting;
        deadline_ = Clock::now() + 30s;
    }

    void handle_reply(const std::string& type, const Json& event) {
        if (type == "response.audio.delta" && event.value("response_id", "") == response_id_) {
            auto samples = decode_pcm(event.at("delta").get_ref<const std::string&>());
            if (!samples.empty()) {
                has_audio_ = true;
                audio_.enqueue_playback(std::move(samples));
                deadline_ = Clock::now() + 30s;
            }
        } else if (type == "response.audio_transcript.done" &&
                   event.value("response_id", "") == response_id_) {
            print_transcript(event.value("transcript", ""));
        } else if (type == "response.done" && event.at("response").value("id", "") == response_id_) {
            const auto status = event.at("response").value("status", "");
            if (duplex_ && (status == "cancelled" || status == "canceled")) {
                pending_calls_.clear();
                interrupt_reply(false);
                listen();
                return;
            }
            if (motion_ && status == "completed" && !pending_calls_.empty()) {
                if (has_audio_) audio_.interrupt_playback();
                finish_tools();
                return;
            }
            if (status != "completed" || !has_audio_) {
                throw std::runtime_error("Realtime response did not complete with audio");
            }
            stage_ = Stage::draining;
            deadline_ = Clock::now() + 65s;
        } else if (motion_ && type == "response.function_call_arguments.done" &&
                   string_member(event, "response_id") == response_id_) {
            const auto id = string_member(event, "call_id");
            if (id.empty() || id.size() > 128 || !seen_calls_.insert(id).second) return;
            if (pending_calls_.size() >= 16) throw std::runtime_error("Too many Realtime tool calls");
            const auto raw = string_member(event, "arguments");
            const auto name = string_member(event, "name");
            std::string action;
            if (raw.size() <= 256) {
                const auto args = Json::parse(raw, nullptr, false);
                if (motion_->standup_only() && name == "stand_up" && args.is_object() && args.empty()) {
                    action = "standup";
                } else if (motion_->walk_after_standup() && name == "step_forward" && args.is_object() && args.empty()) {
                    action = "forward";
                } else if (!motion_->standup_only() && name == "move_short" && args.is_object() && args.size() == 1) {
                    const auto direction = string_member(args, "direction");
                    if (direction == "forward" || direction == "left" || direction == "right") action = direction;
                }
            }
            pending_calls_.emplace_back(id, std::move(action));
        }
    }

    WebSocket& socket_;
    Audio& audio_;
    std::string_view api_key_;
    bool duplex_;
    MotionTools* motion_;
    Stage stage_ = Stage::connecting;
    std::optional<Clock::time_point> deadline_ = Clock::now() + 10s;
    std::optional<Clock::time_point> input_ended_;
    Clock::time_point listen_after_{};
    std::string response_id_;
    std::string speech_item_;
    std::unordered_set<std::string> seen_responses_;
    std::unordered_set<std::string> cancel_events_;
    std::unordered_set<std::string> seen_calls_;
    std::unordered_set<std::string> tool_receipts_;
    std::vector<std::pair<std::string, std::string>> pending_calls_;
    std::string awaiting_call_;
    unsigned turn_ = 0;
    bool has_audio_ = false;
    bool speaking_ = false;
    bool can_request_motion_ = false;
    bool response_allows_motion_ = false;
    bool resume_after_tools_ = true;
};
}

void run_realtime(Audio& audio, const std::string& url, const std::string& api_key,
                  const volatile std::sig_atomic_t& interrupted, MotionTools* motion) {
    WebSocket socket(url, api_key);
    Conversation conversation(socket, audio, api_key, motion);
    while (!interrupted && !conversation.finished()) {
        for (unsigned count = 0; count < 16 && !interrupted && !conversation.finished(); ++count) {
            const auto event = socket.receive();
            if (!event) break;
            conversation.handle(*event);
        }
        if (interrupted || conversation.finished()) break;
        if (motion && conversation.ready_for_motion_input()) motion->update_cloud();
        conversation.poll_motion();
        conversation.tick();
        std::this_thread::sleep_for(2ms);
    }
    socket.close();
}
