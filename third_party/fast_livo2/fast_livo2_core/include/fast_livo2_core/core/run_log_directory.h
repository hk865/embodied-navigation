#ifndef FAST_LIVO2_RUN_LOG_DIRECTORY_H
#define FAST_LIVO2_RUN_LOG_DIRECTORY_H

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <vector>

namespace fastlivo_diagnostics
{
// Resolve before any stream opens. A fixture without a run gets a unique
// temporary process directory, never the compiled source tree's shared Log.
inline std::filesystem::path directory()
{
  if (const char *run = std::getenv("DEMO_RUN_DIR"); run && *run)
    return std::filesystem::absolute(run).lexically_normal() / "fastlivo_debug";
  static const std::filesystem::path temporary = [] {
    const auto pattern = (std::filesystem::temp_directory_path() / "fastlivo2-debug-XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const char *created = ::mkdtemp(buffer.data());
    if (!created) throw std::runtime_error("Cannot create independent FAST-LIVO diagnostic directory");
    return std::filesystem::path(created) / "fastlivo_debug";
  }();
  return temporary;
}
}
#endif
