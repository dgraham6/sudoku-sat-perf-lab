# Sudoku SAT Perf Lab

## Why this project

Performance-oriented Sudoku solvers are a deep rabbit hole, and at the bottom of it sits Thomas Dillon's **[Tdoku](https://github.com/t-dillon/tdoku)**. It's a masterclass in cache-friendly data layouts, bit tricks, and SAT/DPLL engineering. Reading through his code is what got me studying SAT encodings, unit propagation, DPLL search, SCC analysis, and SIMD in the first place.

This repo is where I tinker with that stuff. I re-implement pieces of Tdoku's SCC-driven DPLL approach, try out different memory layouts and parallel strategies, and benchmark them against the Tdoku reference to figure out what actually moves the needle.

Short version: I went looking for faster solvers. Some ideas helped, plenty didn't.

## What's here

Two experimental solvers:

* `lab_code/triad_scc_soa` uses a struct-of-arrays layout with cache-first tweaks.
* `lab_code/triad_scc_parallel_d1` fans out on the first guess across threads (depth-1 splitting).

Plus reproducible benchmarks on the same datasets Tdoku uses. The main takeaway: parallelism rarely pays off here, and micro-optimizations dominate.

## Table of Contents

* [Why this project](#why-this-project)
* [What's here](#whats-here)
* [Background](#background)
* [Build & run](#build--run)
* [Analysis of results](#analysis-of-results)
* [Design & implementation notes](#design--implementation-notes)
* [Environment](#environment)

## Background

**SAT & DPLL, briefly.**
Sudoku can be encoded as SAT: literals represent "(box, element, value, polarity)," and constraints like exactly-one rules and triads become clauses. DPLL combines unit propagation (BCP) with branching (guesses) and backtracking. SCC analysis helps pick impactful literals and spot equivalences in the mostly-binary implication graph.

**Why it can be so fast.**
Most puzzles collapse under unit propagation and short implication chains. A few consequences fall out of that:

* The bottleneck is in tight, branch-sensitive inner loops.
* Memory layout and bit operations usually matter more than fancy heuristics.
* SIMD can help, but only if the data is arranged to feed the vector units efficiently.

## Build & run

```bash
# Build the vendored tdoku harness together with the lab solvers.
cmake -S third_party/tdoku -B third_party/tdoku/build -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/tdoku/build -j

# Correctness: solve + count solutions on the bundled test set.
./third_party/tdoku/build/run_tests

# Benchmark the lab solvers vs the stock tdoku SCC reference (writes a CSV to results/).
scripts/bench_tdoku_vs_drake.sh
```

The lab solvers live in `lab_code/` (single source of truth). They're compiled
straight from that directory and registered in tdoku's benchmark/test harness via
`third_party/tdoku/src/all_solvers.h`, all with SCC inference + heuristic enabled.

## Analysis of results

> Numbers below are from `puzzles1_unbiased` (AppleClang, `-O3 -march=native`, ARM,
> single thread, 2000 puzzles), reproducible via `scripts/bench_tdoku_vs_drake.sh`.
> All solvers run with SCC inference + heuristic on, so the lab and tdoku solvers
> produce identical guess counts (~0.51 guesses/puzzle, ~62% solved with no
> guesses). The differences below are purely in execution speed.

**SoA beats Parallel D1.**
`lab_code/triad_scc_soa` (~2.3k puzzles/sec) runs about 35% faster than the depth-1
parallel version (~1.7k puzzles/sec). Memory layout wins and thread overhead loses,
especially since most puzzles need few or no guesses, so there's little search to
parallelize.

**Tdoku is about 6x faster than my SoA.**
The gap comes from micro-architectural wins:

* hyper-tight propagation loops,
* better branch predictability,
* fewer cache misses with carefully packed state,
* SIMD where it truly pays for itself.

**Why Parallel D1 underperforms.**

* Branching helps only on puzzles that force deep search, and most don't.
* Thread startup, scheduling, and sync dominate the actual work.
* Copying or sharing solver state adds cache overhead.
* Net: parallel fan-out only makes sense with a work-stealing pool and coarse tasks. A single depth-1 split is too fine-grained.

## Design & implementation notes

### Borrowed ideas

* **Triad encoding**: pack constraints so the row/col/box interplay is cheap to reason about.
* **SCC-driven branching**: operate over the binary implication graph to pick impactful literals.
* **Clause bookkeeping**: static `clauses_to_literals_`, `literals_to_clauses_`, plus dynamic implication counts.

### My experiments

1. **Struct-of-Arrays (`lab_code/triad_scc_soa`)**

   * Goal: stride linearly through memory and separate independent fields.
   * Result: fewer cache misses and better prefetch, so clear gains over my baselines.

2. **Parallel guess fan-out (`lab_code/triad_scc_parallel_d1`)**

   * Strategy: when a cell has 2-3 candidates, split across threads at depth-1.
   * Guardrails: cap thread count, recycle a small pool.
   * Result: overhead outweighed the benefit on unbiased datasets. Parallelism here needs coarser splitting and proper work-stealing.

## Environment

* Platform: Apple Silicon (ARM64). The x86 SIMD solver from tdoku is skipped on
  ARM; `triad_scc_simd_stub.cc` forwards the SIMD entry point to the SoA solver.
* Compiler: AppleClang (tested with 17.x and 21.x)
* Flags: `-O3 -march=native`
