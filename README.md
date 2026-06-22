# Sudoku SAT Perf Lab

## Why this project

Performance-oriented Sudoku solvers are a deep rabbit hole — and at the bottom sits Thomas Dillon’s **[Tdoku](https://github.com/t-dillon/tdoku)**, a masterclass in cache-friendly data layouts, bit tricks, and SAT/DPLL engineering. Reading his code sent me down a path of studying SAT encodings, unit propagation, DPLL search, SCC analysis, and SIMD.

This repo is my **performance lab**: a place to re-implement pieces of Tdoku’s SCC-driven DPLL approach, experiment with memory layouts and parallel strategies, and benchmark them against the Tdoku reference to see what really matters.

> TL;DR: I tried to find faster solutions. Some ideas helped, some didn’t.

---

## Highlights

* **Two experimental solvers**

  * `lab_code/triad_scc_soa` — Struct-of-Arrays layout and cache-first tweaks.
  * `lab_code/triad_scc_parallel_d1` — “Fan-out on guess” thread parallelism (depth-1 splitting).
* **Reproducible benchmarks** on the same datasets Tdoku uses.
* **Key lessons**: parallelism helps rarely, while micro-optimizations dominate.

---

## Table of Contents

* [Why this project](#why-this-project)
* [Highlights](#highlights)
* [Background](#background)
* [Build & run](#build--run)
* [Analysis of results](#analysis-of-results)
* [Design & Implementation Notes](#design--implementation-notes)
* [Environment](#environment)

---

## Background

**SAT & DPLL (in short).**
Sudoku can be encoded as SAT: literals represent “(box, element, value, polarity).” Constraints like exactly-one rules and triads become clauses. **DPLL** combines unit propagation (BCP) with branching (guesses) and backtracking. **SCC analysis** helps choose impactful literals and identify equivalences in the mostly-binary implication graph.

**Why it can be so fast.**
Most puzzles collapse under unit propagation and short implication chains. That means:

* The bottleneck is in *tight, branch-sensitive inner loops*.
* **Memory layout and bit operations** usually matter more than new heuristics.
* SIMD can help, but only if the data is arranged to feed the vector units efficiently.

---

## Build & run

```bash
# Build the vendored tdoku harness together with the Drake lab solvers.
cmake -S third_party/tdoku -B third_party/tdoku/build -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/tdoku/build -j

# Correctness: solve + count solutions on the bundled test set.
./third_party/tdoku/build/run_tests

# Benchmark drake vs the stock tdoku SCC reference (writes a CSV to results/).
scripts/bench_tdoku_vs_drake.sh
```

The Drake solvers live in `lab_code/` (single source of truth). They are compiled
straight from that directory and registered in tdoku's benchmark/test harness via
`third_party/tdoku/src/all_solvers.h`, all with SCC inference + heuristic enabled.

## Analysis of results

> Numbers below are from `puzzles1_unbiased` (AppleClang, `-O3 -march=native`, ARM,
> single thread, 2000 puzzles), reproducible via `scripts/bench_tdoku_vs_drake.sh`.
> All solvers run with SCC inference + heuristic on, so the Drake and tdoku solvers
> produce **identical** guess counts (~0.51 guesses/puzzle, ~62% solved with no
> guesses) — the differences below are purely in execution speed.

**SoA beats Parallel D1.**
`lab_code/triad_scc_soa` (~2.3k puzzles/sec) runs \~35% faster than the depth-1
parallel version (~1.7k puzzles/sec). Memory layout wins; thread overhead loses —
especially since most puzzles need few or no guesses, so there is little search to
parallelize.

**Tdoku is \~6× faster than my SoA.**
The gap comes from micro-architectural wins:

* hyper-tight propagation loops,
* better branch predictability,
* fewer cache misses with carefully packed state,
* SIMD where it truly pays its cost.

**Why Parallel D1 underperforms.**

* Branching helps only on puzzles that force deep search; most don’t.
* Thread startup, scheduling, and sync dominate the actual work.
* Copying or sharing solver state adds cache overhead.
* Net: parallel fan-out only makes sense with a **work-stealing pool** and coarse tasks; a single depth-1 split is too fine-grained.

---

## Design & Implementation Notes

### Borrowed ideas

* **Triad encoding**: pack constraints so row/col/box interplay is cheap to reason about.
* **SCC-driven branching**: operate over the binary implication graph to pick impactful literals.
* **Clause bookkeeping**: static `clauses_to_literals_`, `literals_to_clauses_`, dynamic implication counts.

### My experiments

1. **Struct-of-Arrays (`lab_code/triad_scc_soa`)**

   * Goal: stride linearly through memory and separate independent fields.
   * Result: fewer cache misses and better prefetch → clear gains over my baselines.

2. **Parallel guess fan-out (`lab_code/triad_scc_parallel_d1`)**

   * Strategy: when a cell has 2–3 candidates, split across threads at depth-1.
   * Guardrails: cap thread count, recycle a small pool.
   * Result: overhead outweighed benefit on unbiased datasets. parallelism needs more coarse-grained splitting and proper work-stealing.

---

## Environment

* Platform: Apple Silicon (ARM64). The x86 SIMD solver from tdoku is skipped on
  ARM; `triad_scc_simd_stub.cc` forwards the SIMD entry point to the SoA solver.
* Compiler: AppleClang (tested with 17.x and 21.x)
* Flags: `-O3 -march=native`

---
