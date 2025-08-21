# Sudoku SAT Perf Lab

## Why this project

Performance-oriented Sudoku solvers are a deep rabbit hole — and at the bottom sits Thomas Dillon’s **[Tdoku](https://github.com/t-dillon/tdoku)**, a masterclass in cache-friendly data layouts, bit tricks, and SAT/DPLL engineering. Reading his code sent me down a path of studying SAT encodings, unit propagation, DPLL search, SCC analysis, and SIMD.

This repo is my **performance lab**: a place to re-implement pieces of Tdoku’s SCC-driven DPLL approach, experiment with memory layouts and parallel strategies, and benchmark them against the Tdoku reference to see what really matters.

> TL;DR: I tried to find faster solutions. Some ideas helped, some didn’t.

---

## Highlights

* **Two experimental solvers**

  * `drake/triad_scc_soa` — Struct-of-Arrays layout and cache-first tweaks.
  * `drake/triad_scc_parallel_d1` — “Fan-out on guess” thread parallelism (depth-1 splitting).
* **Reproducible benchmarks** on the same datasets Tdoku uses.
* **Key lessons**: parallelism helps rarely, while micro-optimizations dominate.

---

## Table of Contents

* [Why this project](#why-this-project)
* [Highlights](#highlights)
* [Background](#background)
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

## Analysis of results

**SoA beats Parallel D1.**
`drake/triad_scc_soa` runs \~55% faster than the depth-1 parallel version. Memory layout wins; thread overhead loses — especially since \~48% of puzzles need **no guesses at all**.

**Tdoku is \~7× faster than my SoA.**
The gap comes from micro-architectural wins:

* hyper-tight propagation loops,
* better branch predictability,
* fewer cache misses with carefully packed state,
* SIMD where it truly pays its cost.

**Why Parallel D1 underperforms.**

* Branching helps only on puzzles that force deep search; half don’t.
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

1. **Struct-of-Arrays (`drake/triad_scc_soa`)**

   * Goal: stride linearly through memory and separate independent fields.
   * Result: fewer cache misses and better prefetch → clear gains over my baselines.

2. **Parallel guess fan-out (`drake/triad_scc_parallel_d1`)**

   * Strategy: when a cell has 2–3 candidates, split across threads at depth-1.
   * Guardrails: cap thread count, recycle a small pool.
   * Result: overhead outweighed benefit on unbiased datasets. parallelism needs more coarse-grained splitting and proper work-stealing.

---

## Environment

* Compiler: AppleClang 17.0.0.17000013
* Flags: `-O3 -march=native`

---
