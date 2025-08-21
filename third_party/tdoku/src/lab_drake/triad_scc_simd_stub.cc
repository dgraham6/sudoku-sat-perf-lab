// Minimal stub so the registry can link on ARM.

#include <cstddef>
#include <cstdint>

extern "C" size_t DrakeSolverTriadScc_SOA(
    const char* input, size_t limit, uint32_t flags, char* solution, size_t* num_guesses);

extern "C" size_t DrakeSolverTriadScc_SIMD(
    const char* input, size_t limit, uint32_t flags, char* solution, size_t* num_guesses) {
  return DrakeSolverTriadScc_SOA(input, limit, flags, solution, num_guesses);
}
