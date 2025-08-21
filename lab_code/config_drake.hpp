#pragma once
#include <cstdint>  

struct DrakeConfig {
  bool use_soa = false;
  bool use_simd = false;
  bool parallel_depth1 = false;
  int  threads = 1;
};

// map bits from tdoku's `flags` into your config
inline DrakeConfig MakeConfig(uint32_t flags) {
  DrakeConfig c;
  c.use_soa          = (flags & (1u<<8)) != 0;
  c.parallel_depth1  = (flags & (1u<<9)) != 0;
  c.use_simd         = (flags & (1u<<10)) != 0;
  return c;
}
