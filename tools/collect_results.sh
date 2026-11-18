#!/bin/bash
# Run ON THE CLUSTER (login node) to build the data archive for Zenodo / GitHub Releases.
# Usage: VLMNAV_BASE=/path/to/project [OUT=/path/to/data_release] bash tools/collect_results.sh [--no-verification-frames]
# Collects the result files of the series reported in the paper:
#   results/exp_test2_r*_{v2,v1abl,sweeponly,standoffonly}.csv   V2 series, ablation arms, confirmation (worlds 32-36, 41-45, 51-55)
#   results/exp_test_r*_mainA.csv                                 V1 series (worlds 11-15)
#   results/exp_test_f*_*stress.csv                               stress series (full catalogue)
#   results/ctrl_*.csv                                            controller tests with the oracle goal
#   results/verif_*.csv, results/xverif_*.csv                     verification study (three models)
#   vlm_nav/worlds/*.json                                         world manifests
#   verif_*/                                                      frames of the verification study (unless --no-verification-frames)
# Does NOT collect: files with "smoke" or "dbg" in the name, Slurm logs, frames of the navigation runs, containers, model weights.
# Absolute paths of the cluster inside text files are replaced by <VLMNAV_BASE>.
set -eu
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
OUT=${OUT:-$PWD/data_release}
rm -rf "$OUT"; mkdir -p "$OUT/results" "$OUT/worlds"
cd "$BASE"
for f in results/exp_test2_r*.csv results/exp_test_r*_mainA.csv results/exp_test_f*stress.csv results/ctrl_*.csv results/verif_*.csv results/xverif_*.csv; do
  [ -f "$f" ] || continue
  case "$f" in *smoke*|*dbg*) continue;; esac
  case "$f" in results/exp_test2_r*) case "$f" in *_v2.csv|*_v1abl.csv|*_sweeponly.csv|*_standoffonly.csv) ;; *) continue;; esac;; esac
  cp "$f" "$OUT/results/"
done
cp vlm_nav/worlds/*.json "$OUT/worlds/"
if [ "${1:-}" != "--no-verification-frames" ]; then
  mkdir -p "$OUT/verif_frames"
  for d in verif_*/; do [ -d "$d" ] && cp -r "$d" "$OUT/verif_frames/"; done
fi
# remove the cluster's absolute paths from text data files
n=0
for f in $(grep -rIl --include="*.json" --include="*.csv" "$BASE" "$OUT" 2>/dev/null || true); do
  sed -i "s|$BASE|<VLMNAV_BASE>|g" "$f"; n=$((n+1)); echo "path rewritten in ${f#$OUT/}"
done
echo "files with rewritten paths: $n"
# index of the archive
{
  echo "# Contents of the data archive"; echo
  echo "| Series (file pattern) | files |"; echo "|---|---|"
  for p in "exp_test2_r*_v2.csv" "exp_test2_r*_v1abl.csv" "exp_test2_r*_sweeponly.csv" "exp_test2_r*_standoffonly.csv" "exp_test_r*_mainA.csv" "exp_test_f*stress.csv" "ctrl_*.csv" "verif_*.csv" "xverif_*.csv"; do
    echo "| \`$p\` | $(ls "$OUT"/results/$p 2>/dev/null | wc -l) |"
  done
  echo "| world manifests \`worlds/*.json\` | $(ls "$OUT"/worlds/*.json | wc -l) |"
  [ -d "$OUT/verif_frames" ] && echo "| verification frame folders \`verif_frames/verif_*\` | $(ls -d "$OUT"/verif_frames/verif_*/ | wc -l) |"
} > "$OUT/CONTENTS.md"
( cd "$OUT" && find . -type f ! -name SHA256SUMS.txt | sort | xargs sha256sum > SHA256SUMS.txt && echo "files: $(wc -l < SHA256SUMS.txt)" && du -sh . )
cat "$OUT/CONTENTS.md"
tar czf "$OUT.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")"
ls -la "$OUT.tar.gz"
echo "Check for personal data before uploading:  grep -rIl \"\$USER\" $OUT | head"
