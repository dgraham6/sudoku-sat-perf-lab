#!/usr/bin/env bash
set -euo pipefail

# Benchmark the Drake lab solvers against the stock tdoku SCC reference.
#
# repo layout:
#   sudoku-sat-lab/
#     lab_code/                 # Drake solver sources (single source of truth)
#     results/                  # benchmark CSVs land here
#     scripts/                  # this script
#     third_party/tdoku/        # vendored tdoku + benchmark harness

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TDOKU_DIR="$ROOT_DIR/third_party/tdoku"
BUILD_DIR="$TDOKU_DIR/build"
DATA_DIR="$TDOKU_DIR/data"
RESULTS_DIR="$ROOT_DIR/results"

mkdir -p "$RESULTS_DIR"

if [[ ! -x "$BUILD_DIR/run_benchmark" ]]; then
  cmake -S "$TDOKU_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$BUILD_DIR" -j
fi

RUN="$BUILD_DIR/run_benchmark"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$RESULTS_DIR/tdoku_vs_drake_$STAMP.csv"

echo "compiler,compiler_version,flags,dataset,solver,puzzles_per_sec,usec_per_puzzle,percent_no_guess,guesses_per_puzzle" > "$OUT"

# run_benchmark flags (see run_benchmark.cc):
#   -n  test set size            -w  warmup seconds
#   -t  target test seconds      -r  randomly permute puzzles [0|1]
#   -c  emit CSV instead of table [0|1]   (NOTE: this is NOT a solver config flag)
# The SCC inference/heuristic configuration is fixed per-solver in all_solvers.h
# (all of the solvers below run with SCC inference + heuristic enabled).
N=2000   # number of puzzles
W=2      # warmup seconds
T=2      # test seconds
R=0      # do not permute (stable, reproducible ordering)

SOLVERS="tdoku/triad_scc,drake/triad_scc_soa,drake/triad_scc_parallel_d1"

# --- unbiased set ---
"$RUN" "$DATA_DIR/puzzles1_unbiased" -s "$SOLVERS" -n "$N" -w "$W" -t "$T" -r "$R" -c 1 >> "$OUT"

# --- 17-clue hard set ---
"$RUN" "$DATA_DIR/puzzles2_17_clue" -s "$SOLVERS" -n "$N" -w "$W" -t "$T" -r "$R" -c 1 >> "$OUT"

echo "Wrote $OUT"
