#include <cstddef>
#include <cstdint>
#include <cstring>


#ifdef RUST_SUDOKU
struct count_and_guesses {
    size_t count;
    size_t guesses;
};

extern "C"
struct count_and_guesses rust_solve_sudoku(const char *input, size_t limit);

extern "C"
size_t OtherSolverRustSudoku(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                             char *solution, size_t *num_guesses) {
    auto result = rust_solve_sudoku(input, limit);
    *num_guesses = result.guesses;
    return result.count;
}
#endif


#ifdef SK_BFORCE2
#include "module_sk_bforce2/sk_t.h"
#include "module_sk_bforce2/Zhn.h"
#include "module_sk_bforce2/Zhn_cpp.h"

size_t SKBFORCE_guesses;

extern "C"
size_t OtherSolverSKBFORCE2(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                           char *solution, size_t *num_guesses) {
    SKBFORCE_guesses = 0;
    int result = zhou[0].CheckValidityQuick((char*)input);
    *num_guesses = SKBFORCE_guesses;
    return result;
}
#endif


#ifdef JCZSOLVE
#include "jczsolve/JCZSolve.h"
// apply patch JCZSolve.c.diff so JCZSolve uses this extern.
size_t JCZSolve_guesses;

extern "C"
size_t OtherSolverJCZSolve(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                           char *solution, size_t *num_guesses) {
    JCZSolve_guesses = 0;
    size_t count = JCZSolver(input, solution, limit);
    *num_guesses = JCZSolve_guesses;
    return count;
}
#endif


#ifdef JSOLVE
#include "jsolve/JSolve.h"
// apply patch JSolve.c.diff so JSolve uses this extern
size_t JSolve_guesses;

extern "C"
size_t OtherSolverJSolve(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                         char *solution, size_t *num_guesses) {
    JSolve_guesses = 0;
    int count = JSolve(input, solution, limit);
    *num_guesses = JSolve_guesses;
    return count;
}
#endif


#ifdef FSSS
extern "C"
int fsss_solve(const char* in, char* out, size_t *num_guesses, const int mode=0);

extern "C"
size_t OtherSolverFsss(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                       char *solution, size_t *num_guesses) {
    char buffer[81];
    for (int i = 0; i < 81; i++) {
        buffer[i] = input[i] == '.' ? 0 : (input[i] - '0');
    }
    int count = fsss_solve(buffer, solution, num_guesses, limit > 1 ? 1 : 0);
    return count;
}
#endif


#ifdef FSSS2
#include "module_fsss2/fsss2.h"
int nTrials;
bool do_locked_candidates;

extern "C"
size_t OtherSolverFsss2(const char *input, size_t limit, uint32_t configuration,
                        char *solution, size_t *num_guesses) {
    nTrials = 0;
    do_locked_candidates = configuration > 0;
    char zero_based_output[81];
    int count;

    bool pencilmark = input[81] >= '.';
    if (pencilmark) {
        pencilmarks pm;
        pm.initPencilmarks(input);
        if (limit == 1) {
            hasAnySolution has{};
            count = has.solve(pm);
        } else {
            getSingleSolution gss{};
            count = gss.solve(pm, zero_based_output);
        }
    } else {
        char zero_based_input[81];
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 9; j++) {
                char c = input[i * 9 + j];
                if (c >= '1' && c <= '9') {
                    zero_based_input[i * 9 + j] = c - '0';
                } else {
                    zero_based_input[i * 9 + j] = 0;
                }
            }
        }
        if (limit == 1) {
            hasAnySolution has{};
            count = has.solve(zero_based_input);
        } else {
            getSingleSolution gss{};
            count = gss.solve(zero_based_input, zero_based_output);
        }
    }
    *num_guesses = nTrials;
    return count;
}
#endif


#ifdef BB_SUDOKU
extern int Solver(char num_search, unsigned int use_methods, char ret_puzzle,
                  int initp, char* buffer);
extern bool InitTables();
extern int GCnt;

extern "C"
size_t OtherSolverBBSudoku(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                           char *solution, size_t *num_guesses) {
    static bool initialized = InitTables();
    GCnt = 0;
    unsigned methods = 0x03; /* guesses & locked candidates */
    memcpy(solution, input, 81); /* BB_SUDOKU uses a single in/out buffer */
    int count = Solver(limit, methods, 1 /* return solution */, 0, solution);
    *num_guesses = GCnt - 1;
    return count;
}
#endif


#ifdef FAST_SOLV_9R2
extern "C" int make_links(void);
extern "C" int fast_solv_9r2_solve(const char *buffer, int limit);
extern int fast_solv_92r_undone;

extern "C"
size_t OtherSolverFastSolv9r2(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                              char *solution, size_t *num_guesses) {
    static int initialized = make_links();
    size_t count = fast_solv_9r2_solve(input, limit);
    *num_guesses = fast_solv_92r_undone;
    return count;
}
#endif

