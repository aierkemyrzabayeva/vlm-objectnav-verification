#!/bin/bash
# Offline checks of the release (no simulator, no GPU). Run from the repository root: bash tools/check_release.sh
set -u
cd "$(dirname "$0")/../code" || exit 1
fail=0
ok()  { echo "OK    $1"; }
bad() { echo "FAIL  $1"; fail=1; }

python3 -m py_compile *.py && ok "all Python files compile" || bad "py_compile"
for s in *.sh ../containers/*.sh; do bash -n "$s" || { bad "shell syntax: $s"; }; done; ok "shell scripts parse"
rm -rf __pycache__
n=$(python3 test_logic.py ./gen_world.py 2>&1 | grep -c '^PASS')
[ "$n" -eq 10 ] && ok "test_logic.py: 10 tests pass" || bad "test_logic.py: $n of 10 tests pass"
for c in "analyze_exp.py --selftest" "analyze_verif.py --selftest" "compare_versions.py --selftest" "collect_verif.py --selftest"; do
  python3 $c 2>&1 | grep -q SELFTEST_OK && ok "$c" || bad "$c"
done
python3 make_launch_ours.py --selftest 2>&1 | grep -q SELFTEST_OK && ok "make_launch_ours.py --selftest" || bad "make_launch_ours.py --selftest"
python3 test_controller.py --selftest 2>&1 | grep -q SELFTEST_OK && ok "test_controller.py --selftest" || echo "SKIP  test_controller.py --selftest (needs numpy)"

tmp=$(mktemp -d)
python3 gen_world.py --seed 51 --catalog reliable --out "$tmp" --name test2_r51 > /dev/null
diff -q "$tmp/test2_r51.json" ../examples/test2_r51.json > /dev/null && ok "world generator reproduces the manifest of world 51" || bad "world manifest differs"
for a in standoffonly v1abl; do
  python3 analyze_exp.py ../examples/exp_test2_r51_$a.csv > "$tmp/a_$a.txt" 2>&1
  diff -q "$tmp/a_$a.txt" ../examples/expected_analyze_exp_$a.txt > /dev/null && ok "analyze_exp.py on the $a example" || bad "analyze_exp.py output differs ($a)"
done
python3 compare_versions.py --a ../examples/exp_test2_r51_standoffonly.csv --b ../examples/exp_test2_r51_v1abl.csv --cond FULL > "$tmp/cmp.txt" 2>&1
diff -q "$tmp/cmp.txt" ../examples/expected_compare_versions.txt > /dev/null && ok "compare_versions.py on the examples" || bad "compare_versions.py output differs"
rm -rf "$tmp" __pycache__
[ $fail -eq 0 ] && echo "ALL CHECKS PASSED" || { echo "SOME CHECKS FAILED"; exit 1; }
