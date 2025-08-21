#pragma once
#include <vector>
#include <cstdint>

// Minimal adapter: wraps a function `bool Assert(LiteralId, State*)` that already exists,
// but drives it with an explicit stack to avoid recursion blowups.

template <class StateT, class LiteralIdT, class AssertFn>
bool AssertIterative(LiteralIdT root_lit, StateT* st, AssertFn assert_recursive) {
  std::vector<LiteralIdT> stack;
  stack.push_back(root_lit);
  while (!stack.empty()) {
    auto lit = stack.back(); stack.pop_back();
    if (!assert_recursive(lit, st)) return false;
  }
  return true;
}
