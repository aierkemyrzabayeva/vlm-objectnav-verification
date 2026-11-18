#!/usr/bin/env python3
"""
make_launch_ours.py - create ours/multi_robot_scenario.launch from the original system's TD3/assets/multi_robot_scenario.launch.

The original file is never modified and is not part of this repository. Two minimal patches are applied; each must match exactly once,
otherwise nothing is written.

  1. the world file comes from the environment variable WORLD_FILE (set by the jobs) instead of the fixed TD3_signs2.world
  2. the rviz node is removed (the jobs run headless; rviz needs a display)

Usage:  python3 make_launch_ours.py ../TD3/assets/multi_robot_scenario.launch multi_robot_scenario.launch
        python3 make_launch_ours.py --selftest
"""
import re
import sys

WORLD_OLD = '<arg name="world_name" default="TD3_signs2.world"/>'
WORLD_NEW = '<arg name="world_name" default="$(env WORLD_FILE)"/>'
RVIZ = re.compile(r'^[ \t]*<node pkg="rviz" type="rviz" name="rviz"[^\n]*/>[ \t]*\n?', re.M)


def patch(text):
    if text.count(WORLD_OLD) != 1:
        raise SystemExit(f"patch 1 matches {text.count(WORLD_OLD)} times (expected 1); nothing written")
    text = text.replace(WORLD_OLD, WORLD_NEW)
    text, n = RVIZ.subn("", text)
    if n != 1:
        raise SystemExit(f"patch 2 matches {n} times (expected 1); nothing written")
    return text


def selftest():
    toy = ('<launch>\n<arg name="world_name" default="TD3_signs2.world"/>\n<node pkg="gazebo_ros" type="gzserver" name="gz"/>\n'
           '  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find x)/launch/y.rviz" />\n</launch>\n')
    out = patch(toy)
    assert '$(env WORLD_FILE)' in out and 'rviz' not in out and 'gzserver' in out and out.count("\n") == toy.count("\n") - 1
    for bad in ("<launch></launch>", toy + toy):
        try:
            patch(bad)
        except SystemExit:
            continue
        raise AssertionError("a wrong file was accepted")
    print("SELFTEST_OK")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        selftest()
    else:
        src = sys.argv[1] if len(sys.argv) > 1 else "../TD3/assets/multi_robot_scenario.launch"
        dst = sys.argv[2] if len(sys.argv) > 2 else "multi_robot_scenario.launch"
        open(dst, "w").write(patch(open(src).read()))
        print("written", dst)
