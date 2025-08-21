#pragma once
#include <thread>
#include <future>
#include <vector>
#include <utility>
#include <atomic>

template <class StateT, class LiteralIdT, class AssertFn, class SearchFn>
void BranchParallelDepth1(LiteralIdT lit,
                          StateT* state,
                          AssertFn assert_fn,    
                          SearchFn search_fn,  
                          std::atomic<size_t>* solutions,
                          size_t limit) {
  // Left branch copy
  StateT left = *state;
  auto run_branch = [&](bool left_branch) {
    StateT local = left_branch ? left : *state;
    LiteralIdT choose = left_branch ? lit : (lit ^ 1u);
    if (assert_fn(&local, choose)) {
      search_fn(&local);
    }
  };

  std::thread t(run_branch, true);
  run_branch(false);
  t.join();
}
