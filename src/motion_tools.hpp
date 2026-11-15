#pragma once

#include <optional>
#include <string>
#include <string_view>

class MotionTools {
public:
    virtual ~MotionTools() = default;
    virtual void update_cloud() = 0;
    virtual bool standup_only() const { return false; }
    virtual bool walk_after_standup() const { return false; }
    virtual bool request(std::string_view direction) = 0;
    virtual std::optional<std::string> take_result() = 0;
    virtual void cancel_pending() = 0;
};
