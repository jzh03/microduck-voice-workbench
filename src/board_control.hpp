#pragma once

#include "motion_tools.hpp"

#include <memory>
#include <string>

std::unique_ptr<MotionTools> make_board_control(const std::string& socket_path, bool standup_only = false,
                                                bool walk_after_standup = false);
