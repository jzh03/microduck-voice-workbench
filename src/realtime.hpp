#pragma once

#include <csignal>
#include <string>

class Audio;
class MotionTools;

void run_realtime(Audio& audio, const std::string& url, const std::string& api_key,
                  const volatile std::sig_atomic_t& interrupted, MotionTools* motion = nullptr);
