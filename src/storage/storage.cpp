#include "storage.hpp"

#include <atomic>
#include <mutex>
#include <string>

std::atomic<Storage *> Storage::instance;
std::mutex             Storage::instance_mutex;
std::string            Storage::logger_tag;
