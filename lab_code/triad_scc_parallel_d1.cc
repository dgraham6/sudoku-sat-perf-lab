// Reuses Tdoku's SAC + triad encoding and SCC-driven DPLL approach.
// My changes: performance experiments

#include "config_drake.hpp"
#include "stats_drake.hpp"
#include "bcp_iterative.hpp"
#include "parallel.hpp"

thread_local DrakeStats g_drake_stats;
thread_local DrakeConfig g_drake_cfg;

#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <climits>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <future>
#include <atomic>

using namespace std;

namespace {

constexpr int kNumBoxes = 9;
constexpr int kNumPosClausesPerBox = 16; // 9 cells, 6 triads, 1 slack
constexpr int kNumValues = 9;

constexpr uint16_t kNumLiterals = kNumBoxes * kNumPosClausesPerBox * kNumValues * 2;
constexpr uint16_t kAllAsserted = kNumBoxes * (kNumPosClausesPerBox - 1) * kNumValues;

typedef uint32_t ClauseId;
typedef uint32_t LiteralId;
constexpr uint32_t kNoLiteral = UINT32_MAX;

template<int literals>
class FastBitset {
    uint64_t bits[literals / 64 + 1]{};

public:
    void set(uint32_t index) {
        bits[index >> 6u] |= (1ul << (index & 63u));
    }

    bool operator[](uint32_t index) const {
        return bits[index >> 6u] & (1ul << (index & 63u));
    }

    bool pos_or_neg(uint32_t index) const {
        auto positive = index & ~1u;
        return bits[positive >> 6u] & (3ul << (positive & 63u));
    }
};

struct State {
    // 1s for asserted literals, 0s for literals negated or unknown
    FastBitset<kNumLiterals> asserted;
    // the number of literals that can be eliminated before the clause produces binary implications
    vector<uint16_t> clause_free_literals;
    // the number of implications for a given literal. we will not copy the implication lists
    // themselves as part of the state. instead the global state has a vector for each literal
    // that we use as a stack, and these counts are the stack pointers.
    array<uint16_t, kNumLiterals> implication_counts;
    // number of literals asserted. we are done when this equals kAllAsserted.
    uint32_t num_asserted = 0;

    State() : asserted{}, clause_free_literals{}, implication_counts{} {}

    State(const State &prior_state) = default;
};

struct SolverDpllTriadScc {

    // Parallel bookkeeping
    std::atomic<size_t> shared_solutions_{0};
    std::atomic<size_t> shared_guesses_{0};
    std::atomic<bool>   stop_{false};
    std::atomic<bool>   wrote_first_solution_{false};

    struct SearchStats {
        size_t solutions = 0;
        size_t guesses   = 0;
    };
    struct ParallelOutcome {
        SearchStats stats{};
        bool wrote_first = false;
        State result{};
    };


    // Static structures
    vector<vector<LiteralId>> clauses_to_literals_{};
    array<vector<ClauseId>, kNumLiterals> literals_to_clauses_{};
    array<vector<LiteralId>, kNumLiterals> literals_to_implications_{};
    vector<ClauseId> positive_cell_clauses_{};
    State initial_state_{};

    // Heuristics
    bool scc_heuristic_ = true;
    bool scc_inference_ = true;

    // Limits
    size_t limit_ = 1;

    // Result capture
    size_t num_guesses_ = 0;
    size_t num_solutions_ = 0;
    State result_{};

    SolverDpllTriadScc() {
        SetupConstraints();
    }

    static void Display(State *state) {
        string div1 = " +=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+=====+";
        string div2 = " +-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+";
        for (int i = 0; i < 12; i++) {
            cout << (((i % 4) == 0) ? div1 : div2) << endl;
            for (int vi = 0; vi < 3; vi++) {
                for (int j = 0; j < 12; j++) {
                    cout << " | ";
                    for (int vj = 0; vj < 3; vj++) {
                        int box = i / 4 * 3 + j / 4;
                        int elm = (i % 4) * 4 + (j % 4);
                        if (state->asserted[Not(Literal(box, elm, vi * 3 + vj))]) {
                            cout << " ";
                        } else {
                            cout << vi * 3 + vj + 1;
                        }
                    }
                }
                cout << " |" << endl;
            }
        }
        cout << div1 << endl << endl;
    }

    ///////////////////////////////////////////////
    // constraint setup
    ///////////////////////////////////////////////

    static inline LiteralId Not(LiteralId literal) {
        return literal ^ 1u;
    }

    static inline LiteralId Literal(int box, int elem, int value) {
        return (uint32_t)(2 * (elem + 16 * (value + 9 * box)));
    }

    static bool ValidLiteral(LiteralId literal) {
        return ((literal % 32u) & 0x1eu) != 0x1eu;
    }

    inline void AddImplication(LiteralId from, LiteralId to, State *state) {
        auto &implications = literals_to_implications_[from];
        auto &current_size = state->implication_counts[from];
        if (implications.size() == current_size) {
            implications.push_back(to);
        } else {
            implications[current_size] = to;
        }
        current_size++;
    }

    inline void AddClauseWithMinimum(const vector<LiteralId> &literals, int min) {
        ClauseId new_clause_id = clauses_to_literals_.size();
        for (LiteralId literal : literals) {
            literals_to_clauses_[literal].push_back(new_clause_id);
        }
        clauses_to_literals_.push_back(literals);
        initial_state_.clause_free_literals.push_back(literals.size() - 1 - min);
        if (min == 1 && literals.size() == 9) {
            positive_cell_clauses_.push_back(new_clause_id);
        }
    }

    void AddExactlyNConstraint(const vector<LiteralId> &literals, int n) {
        AddClauseWithMinimum(literals, n);
        if (n == 1) {
            for (size_t i = 0; i < literals.size() - 1; i++) {
                for (size_t j = i + 1; j < literals.size(); j++) {
                    AddImplication(literals[i], Not(literals[j]), &initial_state_);
                    AddImplication(literals[j], Not(literals[i]), &initial_state_);
                }
            }
        } else {
            vector<LiteralId> negations;
            negations.reserve(literals.size());
            for (auto literal : literals) negations.push_back(Not(literal));
            AddClauseWithMinimum(negations, (int)negations.size() - n);
        }
    }

    void SetupConstraints() {
        for (int box = 0; box < 9; box++) {
            // ExactlyN constraints over values for a given cell or triad [1/9] and [3/9]
            for (int elem = 0; elem < 15; elem++) {
                vector<LiteralId> literals;
                for (int val = 0; val < 9; val++) {
                    literals.push_back(Literal(box, elem, val));
                }
                // exactly one for normal cells, exactly three for triads
                if (elem / 4 < 3 && elem % 4 < 3) {
                    AddExactlyNConstraint(literals, 1);
                } else {
                    AddExactlyNConstraint(literals, 3);
                }
            }
            // ExactlyN constraints to define each triad [1/4]
            for (int val = 0; val < 9; val++) {
                for (int i = 0; i < 3; i++) {
                    vector<LiteralId> h_triad, v_triad;
                    for (int j = 0; j < 3; j++) {
                        h_triad.push_back(Literal(box, i * 4 + j, val));
                        v_triad.push_back(Literal(box, i + j * 4, val));
                    }
                    h_triad.push_back(Not(Literal(box, i * 4 + 3, val)));
                    v_triad.push_back(Not(Literal(box, i + 12, val)));
                    AddExactlyNConstraint(h_triad, 1);
                    AddExactlyNConstraint(v_triad, 1);
                }
            }
        }
        // ExactlyN constraints over band triads within and across boxes [1/3]
        for (int val = 0; val < 9; val++) {
            for (int band = 0; band < 3; band++) {
                for (int i = 0; i < 3; i++) {
                    vector<LiteralId> h_within, h_across, v_within, v_across;
                    for (int j = 0; j < 3; j++) {
                        h_within.push_back(Literal(band * 3 + i, j * 4 + 3, val));
                        h_across.push_back(Literal(band * 3 + j, i * 4 + 3, val));
                        v_within.push_back(Literal(i * 3 + band, j + 12, val));
                        v_across.push_back(Literal(j * 3 + band, i + 12, val));
                    }
                    AddExactlyNConstraint(h_within, 1);
                    AddExactlyNConstraint(h_across, 1);
                    AddExactlyNConstraint(v_within, 1);
                    AddExactlyNConstraint(v_across, 1);
                }
            }
        }
    }

    ///////////////////////////////////////////////
    // boolean constraint propagation
    ///////////////////////////////////////////////

    vector<LiteralId> noneliminated;

    void AddBinaryImplicationsAmongNonEliminated(ClauseId clause_id, State *state) {
        const auto &literals = clauses_to_literals_[clause_id];
        int expect = (int)literals.size() - initial_state_.clause_free_literals[clause_id];
        if (expect == 2) {
            LiteralId first = kNumLiterals;
            for (LiteralId literal : literals) {
                if (!state->asserted[Not(literal)]) {
                    if (first == kNumLiterals) {
                        first = literal;
                    } else {
                        AddImplication(Not(first), literal, state);
                        AddImplication(Not(literal), first, state);
                        return;
                    }
                }
            }
            assert(false);
        } else {
            noneliminated.clear();
            for (LiteralId literal : literals) {
                if (!state->asserted[Not(literal)]) {
                    noneliminated.push_back(literal);
                }
            }
            for (size_t i = 0; i < noneliminated.size() - 1; i++) {
                for (size_t j = i + 1; j < noneliminated.size(); j++) {
                    AddImplication(Not(noneliminated[i]), noneliminated[j], state);
                    AddImplication(Not(noneliminated[j]), noneliminated[i], state);
                }
            }
        }
    }

    bool Assert(LiteralId literal, State *state) {
        if (state->asserted[literal]) {
            return true;
        }
        if (state->asserted[Not(literal)]) {
            return false;
        }
        state->asserted.set(literal);
        state->num_asserted++;

        for (auto clause_id : literals_to_clauses_[Not(literal)]) {
            if (--state->clause_free_literals[clause_id] == 0) {
                AddBinaryImplicationsAmongNonEliminated(clause_id, state);
            }
        }

        const vector<LiteralId> &implications = literals_to_implications_[literal];
        uint16_t num_implications = state->implication_counts[literal];
        for (uint16_t i = 0; i < num_implications; i++) {
            if (!Assert(implications[i], state)) return false;
        }
        return true;
    }

    ///////////////////////////////////////////////
    // path-based strongly connected components with adaptations
    ///////////////////////////////////////////////

    int preorder_counter = 0;
    array<int, kNumLiterals> preorder_index{};
    vector<LiteralId> stack_p;
    vector<LiteralId> stack_s;
    array<int, kNumLiterals> literal_to_component_id{};
    int next_component_id = 0;
    LiteralId best_component_literal = kNoLiteral;
    int best_component_size = -1;

bool SccVisit(LiteralId literal, State *state) {
        if (scc_inference_) {
            LiteralId common_ancestor = kNoLiteral;
            for (auto ancestor : stack_p) {
                if (preorder_index[ancestor] <= preorder_index[Not(literal)]) {
                    common_ancestor = ancestor;
                } else {
                    break;
                }
            }
            if (common_ancestor != kNoLiteral) {
                // we found a proximal ancestor implying both the literal and its negation.
                // (this ancestor might actually be the negation). we can therefore eliminate
                // the ancestor (and as a consequence the chain of literals from the ancestor
                // up to the root of stack_p). this might lead to discovery of a conflict.
                if (!Assert(Not(common_ancestor), state)) return false;
                // or it might lead to discovery of an assertion that lets us skip this branch.
                if (state->asserted[literal]) return true;
            }
        }
        preorder_index[literal] = preorder_counter++;
        stack_p.push_back(literal);
        stack_s.push_back(literal);

        auto &implications = literals_to_implications_[literal];
        auto &num_implications = state->implication_counts[literal];

        for (size_t i = 0; i < num_implications; i++) {
            LiteralId implication = implications[i];
            if (state->asserted[implication]) {
                // we can skip any already-asserted implications. these correspond to subsumed
                // binary clauses that have no effect on inference.
                continue;
            } else if (preorder_index[implication] == -1) {
                if (!SccVisit(implication, state)) {
                    return false; // back out. we are in an inconsistent state.
                }
                if (scc_inference_ && state->asserted.pos_or_neg(literal)) {
                    // visiting an implication and its consequences may have resulted in the
                    // current literal's assertion or negation. either way we can stop.
                    break;
                }
            } else if (literal_to_component_id[implication] == -1) {
                while (preorder_index[stack_p.back()] > preorder_index[implication]) {
                    stack_p.pop_back();
                }
            }
        }
        if (literal == stack_p.back()) {
            stack_p.pop_back();
            int component_size = (find(stack_s.rbegin(), stack_s.rend(), literal) -
                                  stack_s.rbegin() + 1);
            if (!state->asserted.pos_or_neg(literal)) {
                bool negation_has_component = literal_to_component_id[Not(literal)] >= 0;
                for (auto it = stack_s.end() - component_size; it != stack_s.end(); it++) {
                    literal_to_component_id[*it] = next_component_id;
                }
                // if the negation has a prior component it will be of the same size, and we
                // should prefer it since topologically there may exist a path of implication
                // from this component to the one containing the negation. in this case skip.
                if (!negation_has_component) {
                    // otherwise, we want to prioritize the largest component.
                    if (component_size > best_component_size) {
                        best_component_size = component_size;
                        best_component_literal = literal;
                    }
                }
                next_component_id++;
            }
            stack_s.resize(stack_s.size() - component_size);
        }
        return true;
    }

    bool FindStronglyConnectedComponents(State *state) {
        preorder_counter = 0;
        preorder_index.fill(-1);
        stack_p.clear();
        stack_s.clear();
        literal_to_component_id.fill(-1);
        next_component_id = 0;
        best_component_literal = kNoLiteral;
        best_component_size = -1;

        for (uint16_t literal = 0; literal < kNumLiterals; literal += 2) {
            if (preorder_index[literal] == -1 && ValidLiteral(literal) &&
                !state->asserted.pos_or_neg(literal)) {
                if (!SccVisit(literal, state)) {
                    return false;
                }
            }
        }
        return true;
    }

    ///////////////////////////////////////////////
    // heuristic search
    ///////////////////////////////////////////////

    LiteralId ChooseLiteralToBranchByComponent(State * /*state*/) {
        return best_component_literal;
    }

    LiteralId ChooseLiteralToBranchByClause(State *state) {
        int min_free = INT8_MAX, which_clause = 0;
        for (ClauseId clause_id : positive_cell_clauses_) {
            int num_free = state->clause_free_literals[clause_id];
            if (num_free < min_free) {
                min_free = num_free;
                which_clause = (int)clause_id;
            }
        }
        for (LiteralId literal : clauses_to_literals_[which_clause]) {
            if (!state->asserted[Not(literal)]) {
                return literal;
            }
        }
        exit(1); // shouldn't be possible if puzzle is unsolved.
    }

    void AdoptFirstSolutionFrom(const SolverDpllTriadScc& other) {
        if (!wrote_first_solution_.load(std::memory_order_relaxed) &&
            other.wrote_first_solution_.load(std::memory_order_relaxed)) {
            result_ = other.result_;
            wrote_first_solution_.store(true, std::memory_order_relaxed);
        }
    }


    std::unique_ptr<SolverDpllTriadScc> CloneForParallel() const {
        auto s = std::make_unique<SolverDpllTriadScc>();

        // Copy static structures (read-only during search)
        s->clauses_to_literals_      = clauses_to_literals_;
        s->literals_to_clauses_      = literals_to_clauses_;
        s->positive_cell_clauses_    = positive_cell_clauses_;

        // Deep-copy implication storage so each branch can push independently
        s->literals_to_implications_ = literals_to_implications_;

        // Copy initial state 
        s->initial_state_            = initial_state_;

        s->scc_heuristic_            = scc_heuristic_;
        s->scc_inference_            = scc_inference_;
        s->limit_                    = limit_;

        s->shared_solutions_.store(0, std::memory_order_relaxed);
        s->shared_guesses_.store(0, std::memory_order_relaxed);
        s->stop_.store(false, std::memory_order_relaxed);
        s->wrote_first_solution_.store(false, std::memory_order_relaxed);
        s->num_guesses_   = 0;
        s->num_solutions_ = 0;
        s->result_        = State{};

        return s;
    }

    SearchStats BranchOnLiteral(
            LiteralId literal,
            State* state,
            int depth,
            bool parallel_first_split,
            size_t limit_remaining) {

            SearchStats out{};
            if (stop_.load(std::memory_order_relaxed) || limit_remaining == 0) {
                return out;
            }

            // One branching decision at this node
            out.guesses++;

    if (parallel_first_split && depth == 0) {
        out.guesses++;

        // Left branch on a cloned solver
        State left = *state;
        auto left_solver = this->CloneForParallel();
        auto left_fut = std::async(std::launch::async,
            [ls = std::move(left_solver), left, depth, limit_remaining, literal]() mutable -> ParallelOutcome {
                ParallelOutcome po{};
                if (ls->Assert(literal, &left)) {
                    po.stats = ls->CountSolutionsConsistentWithPartialAssignment(
                        &left, depth + 1, false, limit_remaining);
                    if (po.stats.solutions > 0 &&
                        ls->wrote_first_solution_.load(std::memory_order_relaxed)) {
                        po.wrote_first = true;
                        po.result = ls->result_;
                    }
                }
                return po;
            });

        // Right branch on this solver (isolated State copy)
        SearchStats right_stats{};
        State right = *state;
        if (Assert(Not(literal), &right)) {
            right_stats = CountSolutionsConsistentWithPartialAssignment(
                &right, depth + 1, false, limit_remaining);
        }

        // Join & combine
        ParallelOutcome left_out = left_fut.get();
        out.solutions += left_out.stats.solutions + right_stats.solutions;
        out.guesses   += left_out.stats.guesses   + right_stats.guesses;

        // If left found a solution, adopt it 
        if (left_out.wrote_first && !wrote_first_solution_.load(std::memory_order_relaxed)) {
            result_ = left_out.result;
            wrote_first_solution_.store(true, std::memory_order_relaxed);
        }

        if (out.solutions >= limit_remaining) {
            stop_.store(true, std::memory_order_relaxed);
        }
        return out;
    }


        State left = *state; // copy State (bitset, vector, counts)
        if (Assert(literal, &left)) {
            auto got = CountSolutionsConsistentWithPartialAssignment(
                &left, depth + 1, false, limit_remaining);
            out.solutions += got.solutions;
            out.guesses   += got.guesses;
            if (out.solutions >= limit_remaining) return out;
        }

        // Right branch 
        if (Assert(Not(literal), state)) {
            auto got = CountSolutionsConsistentWithPartialAssignment(
                state, depth + 1, false,
                limit_remaining - out.solutions);
            out.solutions += got.solutions;
            out.guesses   += got.guesses;
        }

        return out;
    }

    SearchStats CountSolutionsConsistentWithPartialAssignment(
        State *state, int depth, bool parallel_first_split, size_t limit_remaining) {

        SearchStats out{};
        if (limit_remaining == 0 || stop_.load(std::memory_order_relaxed)) return out;

        if (scc_heuristic_ || scc_inference_) {
            while (state->num_asserted < kAllAsserted) {
                auto prev_asserted = state->num_asserted;
                if (!FindStronglyConnectedComponents(state)) return out; // inconsistent -> 0 solutions
                if (prev_asserted == state->num_asserted) break;
            }
        }

        // Solved?
        if (state->num_asserted == kAllAsserted) {
            out.solutions = 1;
            // Capture first solution exactly once
            bool expected = false;
            if (wrote_first_solution_.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
                result_ = *state;
            }
            if (out.solutions >= limit_remaining) {
                stop_.store(true, std::memory_order_relaxed);
            }
            return out;
        }

        LiteralId branch_literal =
            (scc_heuristic_ && best_component_literal != kNoLiteral)
                ? best_component_literal
                : ChooseLiteralToBranchByClause(state);

        auto got = BranchOnLiteral(branch_literal, state, depth, parallel_first_split, limit_remaining);
        if (got.solutions >= limit_remaining) {
            stop_.store(true, std::memory_order_relaxed);
        }
        return got;
    }

    bool InitializePuzzle(const char *input, bool pencilmark, State *state) {
        for (int i = 0; i < 81; i++) {
            int box = i / 27 * 3 + (i % 9) / 3;
            int elm = ((i / 9) % 3) * 4 + (i % 3);
            if (pencilmark) {
                for (int j = 0; j < 9; j++) {
                    if (input[i * 9 + j] == '.') {
                        if (!Assert(Not(Literal(box, elm, j)), state)) return false;
                    }
                }
            } else {
                char digit = input[i];
                if (digit != '.') {
                    int val = digit - '1';
                    if (!Assert(Literal(box, elm, val), state)) return false;
                }
            }
        }
        return true;
    }

    ///////////////////////////////////////////////
    // entry
    ///////////////////////////////////////////////

    size_t SolveSudoku(const char *input, size_t limit, uint32_t configuration,
                       char *solution, size_t *num_guesses) {
        limit_ = limit;
        scc_inference_ = (configuration & 1u) > 0;
        scc_heuristic_ = (configuration & 2u) > 0;
        bool pencilmark = input[81] >= '.';
        num_solutions_ = 0;
        num_guesses_   = 0;

        result_ = initial_state_;
        State state = initial_state_;

        if (!InitializePuzzle(input, pencilmark, &state)) {
            return 0;
        }

        stop_.store(false, std::memory_order_relaxed);
        wrote_first_solution_.store(false, std::memory_order_relaxed);

        auto stats = CountSolutionsConsistentWithPartialAssignment(
            &state, /*depth*/0,
            /*parallel_first_split*/ g_drake_cfg.parallel_depth1,
            /*limit_remaining*/ limit_);

        num_solutions_ = stats.solutions;
        num_guesses_   = stats.guesses;

        for (int i = 0; i < 81; i++) {
            int box = i / 27 * 3 + (i % 9) / 3;
            int elm = ((i / 9) % 3) * 4 + (i % 3);
            for (int val = 0; val < 9; val++) {
                if (result_.asserted[Literal(box, elm, val)]) {
                    solution[i] = char('1' + val);
                }
            }
        }

        *num_guesses = num_guesses_;
        return num_solutions_;
    }
};
}
extern "C" size_t DrakeSolverTriadScc_ParallelD1(
  const char* input, size_t limit, uint32_t flags,
  char* solution, size_t* num_guesses) {
  g_drake_cfg = {};                // requires config_drake.hpp
  g_drake_cfg.parallel_depth1 = true;
  SolverDpllTriadScc solver;
  return solver.SolveSudoku(input, limit, flags, solution, num_guesses);
}
