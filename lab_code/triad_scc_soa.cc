#include "triad_scc_core.hpp"
#include "adjacency.hpp"

// Use the compact CSR adjacency
using Solver = SolverDpllTriadScc<AdjCSR<kNumLiterals>>;

extern "C" size_t DrakeSolverTriadScc_SOA(
    const char* input, size_t limit, uint32_t flags, char* solution, size_t* num_guesses) {
  Solver solver;
  return solver.SolveSudoku(input, limit, flags, solution, num_guesses);
}
