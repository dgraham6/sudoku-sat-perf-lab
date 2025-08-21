set -euo pipefail

# repo layout:
#   sudoku-sat-lab/
#     results/
#     scripts/
#     third_party/tdoku/

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TDOKU_DIR="$ROOT_DIR/third_party/tdoku"
BUILD_DIR="$TDOKU_DIR/build"
DATA_DIR="$TDOKU_DIR/data"
RESULTS_DIR="$ROOT_DIR/results"

mkdir -p "$RESULTS_DIR"

if [[ ! -x "$BUILD_DIR/run_benchmark" ]]; then
  mkdir -p "$BUILD_DIR"
  cmake -S "$TDOKU_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$BUILD_DIR" -j
fi

RUN="$BUILD_DIR/run_benchmark"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$RESULTS_DIR/tdoku_vs_drake_$STAMP.csv"

echo "compiler,compiler_version,flags,dataset,solver,puzzles_per_sec,usec_per_puzzle,percent_no_guess,guesses_per_puzzle" > "$OUT"

N=2000       # number of puzzles
W=2          # warmup batches
T=1          # harness threads (keep at 1 to measure in-solver effects)
R=0          # random seed
C=1          # configuration flags passed to solvers (keep 1)

# --- unbiased set ---
"$RUN" "$DATA_DIR/puzzles1_unbiased" \
  -s tdoku/triad_scc,drake/triad_scc_parallel_d1,drake/triad_scc_soa \
  -n "$N" -w "$W" -t "$T" -r "$R" -c "$C" >> "$OUT"

# --- 17-clue hard set ---
"$RUN" "$DATA_DIR/puzzles2_17_clue" \
  -s tdoku/triad_scc,drake/triad_scc_parallel_d1,drake/triad_scc_soa \
  -n "$N" -w "$W" -t "$T" -r "$R" -c "$C" >> "$OUT"

echo "Wrote $OUT"
