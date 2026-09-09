#!/usr/bin/env bash
# ============================================================================
# setup_ego_workspace.sh — restore the pre-built EGO-Planner workspace
#
# The ego-planner submodule is distributed as a *pre-built* catkin workspace
# without source. Four things are missing or wrong on a fresh clone:
#
#   1. devel/.catkin points at the original build machine's path
#   2. devel/share/*/package.xml are absent  -> rospack finds nothing
#   3. *.msg definitions are absent (only compiled headers exist) -> rosmsg fails
#   4. devel/lib/map_tools/*.py wrapper scripts hardcode the build machine's path
#
# This script regenerates all of it from artifacts that ARE present (the
# compiled C++ headers and the shared libraries). Idempotent — safe to re-run.
#
# Usage:
#   ./scripts/setup_ego_workspace.sh [path/to/ego-planner/planner]
#   (default: <repo>/ego-planner/planner)
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS="${1:-$REPO_ROOT/ego-planner/planner}"

if [ ! -d "$WS/devel" ]; then
    echo "ERROR: $WS/devel not found." >&2
    echo "Run 'git submodule update --init --recursive' first." >&2
    exit 1
fi

echo "==> Restoring EGO-Planner workspace at: $WS"

# ---------------------------------------------------------------------------
# 1. devel/.catkin must point at this workspace's source dir
# ---------------------------------------------------------------------------
echo "$WS/src" > "$WS/devel/.catkin"
echo "  [1/4] devel/.catkin -> $WS/src"

# ---------------------------------------------------------------------------
# 2. package.xml for every package (devel/share + src, for rospack discovery)
#    Dependencies derived from `ldd` on the compiled binaries.
# ---------------------------------------------------------------------------
python3 - "$WS" <<'PYEOF'
import os, sys, textwrap

ws = sys.argv[1]
share = os.path.join(ws, 'devel', 'share')
src = os.path.join(ws, 'src')

# package -> dependencies (space separated; verified via ldd on the binaries)
DEPS = {
    'ego_planner':          'roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf message_generation message_runtime quadrotor_msgs traj_utils bspline_opt path_searching plan_env pose_utils uav_utils',
    'waypoint_generator':   'roscpp rospy std_msgs geometry_msgs nav_msgs tf',
    'traj_utils':           'roscpp std_msgs geometry_msgs nav_msgs bspline_opt plan_env',
    'bspline_opt':          'roscpp std_msgs geometry_msgs eigen plan_env traj_utils',
    'path_searching':       'roscpp std_msgs geometry_msgs nav_msgs plan_env',
    'plan_env':             'roscpp std_msgs geometry_msgs sensor_msgs pcl_ros',
    'quadrotor_msgs':       'roscpp std_msgs geometry_msgs nav_msgs message_generation message_runtime',
    'pose_utils':           'roscpp std_msgs geometry_msgs nav_msgs',
    'uav_utils':            'roscpp std_msgs geometry_msgs',
    'local_sensing_node':   'roscpp std_msgs sensor_msgs geometry_msgs pcl_ros cv_bridge',
    'map_generator':        'roscpp std_msgs sensor_msgs pcl_ros',
    'odom_visualization':   'roscpp std_msgs nav_msgs tf',
    'so3_control':          'roscpp std_msgs geometry_msgs',
    'so3_quadrotor_simulator': 'roscpp std_msgs geometry_msgs',
    'multi_map_server':     'roscpp std_msgs nav_msgs',
    'mockamap':             'roscpp std_msgs sensor_msgs',
    'rviz_plugins':         'roscpp rviz std_msgs geometry_msgs',
    'cmake_utils':          '',
    'map_tools':            'rospy nav_msgs geometry_msgs sensor_msgs tf tf2_ros map_server rviz',
}

TMPL = textwrap.dedent('''\
    <?xml version="1.0"?>
    <package format="2">
      <name>{name}</name>
      <version>0.0.0</version>
      <description>The {name} package (pre-built, from ZJU-FAST-Lab ego-planner)</description>
      <maintainer email="todo@todo">EGO-Planner</maintainer>
      <license>MIT</license>
      <buildtool_depend>catkin</buildtool_depend>
    {deps}  <export>
      </export>
    </package>
    ''')

count = 0
for pkg, deps in DEPS.items():
    dstr = ''.join(f'  <depend>{d}</depend>\n' for d in deps.split())
    xml = TMPL.format(name=pkg, deps=dstr)
    # devel/share copy is always (re)written — it is the one rospack needs.
    sdir = os.path.join(share, pkg)
    os.makedirs(sdir, exist_ok=True)
    with open(os.path.join(sdir, 'package.xml'), 'w') as f:
        f.write(xml)
    # src copy: never clobber a real, version-controlled package.xml
    # (map_tools ships one with the full dependency set).
    sfile = os.path.join(src, pkg, 'package.xml')
    if not os.path.exists(sfile):
        os.makedirs(os.path.dirname(sfile), exist_ok=True)
        with open(sfile, 'w') as f:
            f.write(xml)
    count += 1
print(f"  [2/4] wrote package.xml for {count} packages (devel/share + src)")
PYEOF

# ---------------------------------------------------------------------------
# 3. Reconstruct *.msg from the compiled C++ Definition structs
# ---------------------------------------------------------------------------
python3 - "$WS" <<'PYEOF'
import os, re, sys, glob

ws = sys.argv[1]
inc = os.path.join(ws, 'devel', 'include')

def extract(path):
    src = open(path).read()
    # Target the Definition struct — MD5Sum/DataType also define value().
    m = re.search(r'struct Definition<\s*::[\w:]+_<ContainerAllocator>\s*>(.*?)\n\};', src, re.S)
    if not m:
        return None
    vm = re.search(r'static const char\* value\(\)\s*\{(.*?)\n  \}', m.group(1), re.S)
    if not vm:
        return None
    lits = re.findall(r'"((?:[^"\\]|\\.)*)"', vm.group(1))
    if not lits:
        return None
    text = ''.join(lits).encode().decode('unicode_escape')
    # Drop the expanded dependency sections (==== MSG: ...)
    text = text.split('\n' + '=' * 80 + '\n')[0]
    return text.rstrip() + '\n'

count = 0
for hdr in sorted(glob.glob(os.path.join(inc, '*', '*.h'))):
    pkg = os.path.basename(os.path.dirname(hdr))
    name = os.path.basename(hdr)[:-2]
    defn = extract(hdr)
    if not defn:
        continue
    outdir = os.path.join(ws, 'src', pkg, 'msg')
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, name + '.msg'), 'w') as f:
        f.write(defn)
    count += 1
print(f"  [3/4] reconstructed {count} .msg definitions from devel/include headers")
PYEOF

# ---------------------------------------------------------------------------
# 4. Repoint the catkin-generated wrapper scripts at the local source
# ---------------------------------------------------------------------------
python3 - "$WS" <<'PYEOF'
import os, sys, glob

ws = sys.argv[1]
WRAPPER = '''#!/usr/bin/python3
# -*- coding: utf-8 -*-
# generated from catkin/cmake/template/script.py.in (path repaired by setup_ego_workspace.sh)
python_script = {src!r}
with open(python_script, 'r') as fh:
    context = {{
        '__builtins__': __builtins__,
        '__doc__': None,
        '__file__': python_script,
        '__name__': __name__,
        '__package__': None,
    }}
    exec(compile(fh.read(), python_script, 'exec'), context)
'''

count = 0
for wrap in glob.glob(os.path.join(ws, 'devel', 'lib', '*', '*.py')):
    txt = open(wrap).read()
    if 'generated from catkin' not in txt[:200]:
        continue
    pkg = os.path.basename(os.path.dirname(wrap))
    name = os.path.basename(wrap)
    src = os.path.join(ws, 'src', pkg, 'scripts', name)
    if not os.path.exists(src):
        continue
    if src in txt:
        continue
    with open(wrap, 'w') as f:
        f.write(WRAPPER.format(src=src))
    os.chmod(wrap, 0o755)
    count += 1
print(f"  [4/4] repaired {count} wrapper script(s) under devel/lib/")
PYEOF

echo ""
echo "Done. Now source the combined environment:"
echo "  source $REPO_ROOT/setup_ego_vio.sh"
