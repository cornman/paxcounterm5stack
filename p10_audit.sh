#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Power of Ten — mechanical audit for the two scriptable rules.
#   Rule 4: functions <= 60 lines
#   Rule 5: assertion density >= 2 c_assert per function (average)
#
# This is a heuristic scanner (brace-depth based), good enough to give a number
# to track. Struct-inline methods in headers are not individually measured and
# are reviewed by hand. Rules 1-3, 6-10 are not mechanically checked here — see
# POWER_OF_TEN.md and the compiler/analyzer flags in run_tests.sh.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail
cd "$(dirname "$0")"

FILES=$(ls ./*.cpp ./*.ino ./*.h 2>/dev/null)
LIMIT=60

awk -v limit="$LIMIT" '
FNR==1 { file=FILENAME }
{
  line=$0
  # Candidate function start: a definition line at column 0 containing "(".
  if (depth==0 && start==0 && line ~ /^[A-Za-z_]/ && line ~ /\(/ &&
      line !~ /^(if|for|while|switch|return|else)/)
    start=FNR
  o=gsub(/{/,"{"); c=gsub(/}/,"}")
  prev=depth; depth+=o-c
  if (prev==0 && depth>0 && start>0) fstart=start
  if (prev>0 && depth<=0) {
    if (fstart>0) {
      len=FNR-fstart+1; funcs++; total+=len
      if (len>max) { max=len; maxwhere=file ":" fstart }
      if (len>limit) printf "  RULE 4  %s:%d  %d lines (> %d)\n", file, fstart, len, limit
    }
    fstart=0; start=0
  }
  if (depth<=0) start=0
}
END {
  printf "\nRule 4 (function length):\n"
  printf "  functions measured : %d\n", funcs
  printf "  longest            : %d lines  (%s)\n", max, maxwhere
  if (max<=limit) printf "  status             : PASS (all <= %d)\n", limit
  else            printf "  status             : review flagged functions above\n"
}
' $FILES

echo ""
echo "Rule 5 (assertion density):"
ASSERTS=$(grep -oE 'c_assert\(' $FILES | wc -l | tr -d ' ')
# Approximate function count: measured functions from awk pass above is exact for
# .cpp/.ino; recompute here across the same files for the ratio.
FUNCS=$(awk '
{
  line=$0
  if (depth==0 && start==0 && line ~ /^[A-Za-z_]/ && line ~ /\(/ &&
      line !~ /^(if|for|while|switch|return|else)/) start=FNR
  o=gsub(/{/,"{"); c=gsub(/}/,"}"); prev=depth; depth+=o-c
  if (prev==0 && depth>0 && start>0) fstart=start
  if (prev>0 && depth<=0) { if (fstart>0) funcs++; fstart=0; start=0 }
  if (depth<=0) start=0
}
END { print funcs }
' $FILES)
printf "  c_assert calls     : %s\n" "$ASSERTS"
printf "  functions          : %s\n" "$FUNCS"
if [ "$FUNCS" -gt 0 ]; then
  awk -v a="$ASSERTS" -v f="$FUNCS" 'BEGIN{
    d=a/f; printf "  density            : %.2f per function\n", d
    printf "  status             : %s (floor 2.0)\n", (d>=2.0 ? "PASS" : "below floor")
  }'
fi
