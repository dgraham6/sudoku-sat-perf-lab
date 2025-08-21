#pragma once
#include <array>
#include <cstdint>
#include <vector>

using u32 = std::uint32_t;

template<int NLiterals>
struct AdjVector {

  const std::vector<std::vector<u32>>* clauses_to_literals = nullptr;            // [clause] -> [lits...]
  const std::array<std::vector<u32>, NLiterals>* literals_to_clauses = nullptr;  // [lit]    -> [clauses...]

  void build(const std::vector<std::vector<u32>>& CTL,
             const std::array<std::vector<u32>, NLiterals>& LTC) {
    clauses_to_literals = &CTL;
    literals_to_clauses = &LTC;
  }

  template<class F>
  inline void for_each_clause_of_not_literal(u32 literal, F&& f) const {
    const auto& vec = (*literals_to_clauses)[literal ^ 1u];
    for (u32 c : vec) f(c);
  }

  template<class F>
  inline void for_each_literal_in_clause(u32 cid, F&& f) const {
    const auto& lits = (*clauses_to_literals)[cid];
    for (u32 L : lits) f(L);
  }

  inline int clause_size(u32 cid) const {
    return static_cast<int>((*clauses_to_literals)[cid].size());
  }
};

template<int NLiterals>
struct AdjCSR {
  // clause -> literals (CSR)
  std::vector<u32>  cl_edges;  // concatenated literals
  std::vector<u32>  cl_off;    // size = num_clauses + 1

  std::vector<u32>  lit_edges; // concatenated clause ids
  std::array<u32, NLiterals + 1> lit_off{}; // offsets per literal

  void build(const std::vector<std::vector<u32>>& CTL,
             const std::array<std::vector<u32>, NLiterals>& LTC) {

    cl_off.resize(CTL.size() + 1);
    std::size_t acc = 0;
    for (std::size_t c = 0; c < CTL.size(); ++c) { cl_off[c] = static_cast<u32>(acc); acc += CTL[c].size(); }
    cl_off[CTL.size()] = static_cast<u32>(acc);
    cl_edges.reserve(acc);
    for (const auto& v : CTL) cl_edges.insert(cl_edges.end(), v.begin(), v.end());

    lit_off[0] = 0;
    for (u32 l = 0; l < static_cast<u32>(NLiterals); ++l)
      lit_off[l + 1] = lit_off[l] + static_cast<u32>(LTC[l].size());
    lit_edges.resize(lit_off[NLiterals]);

    std::vector<u32> cur(NLiterals);
    for (u32 l = 0; l < static_cast<u32>(NLiterals); ++l) cur[l] = lit_off[l];
    for (u32 l = 0; l < static_cast<u32>(NLiterals); ++l)
      for (u32 c : LTC[l]) lit_edges[cur[l]++] = c;
  }

  template<class F>
  inline void for_each_clause_of_not_literal(u32 literal, F&& f) const {
    u32 b = lit_off[(literal ^ 1u)], e = lit_off[(literal ^ 1u) + 1];
    for (u32 p = b; p < e; ++p) f(lit_edges[p]);
  }

  template<class F>
  inline void for_each_literal_in_clause(u32 cid, F&& f) const {
    u32 b = cl_off[cid], e = cl_off[cid + 1];
    for (u32 p = b; p < e; ++p) f(cl_edges[p]);
  }

  inline int clause_size(u32 cid) const {
    return static_cast<int>(cl_off[cid + 1] - cl_off[cid]);
  }
};
