#pragma once
#include <atomic>

struct DrakeStats {
  std::atomic<uint64_t> guesses{0}, bcp_steps{0}, implications{0}, scc_runs{0};
};

extern thread_local DrakeStats g_drake_stats;
