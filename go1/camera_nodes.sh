#!/usr/bin/env bash
# ============================================================================
# camera_nodes.sh -- stop / restart the Go1's autostart camera nodes
#
# WHY
# ---
# The Jetsons autostart camera nodes that hold /dev/video*, so UnitreeCameraSDK
# cannot open the device until they are stopped. The hard requirement from the
# user: **kill processes only** -- do not delete files, do not disable autostart,
# and be able to bring them back without rebooting.
#
# WHAT IT DOES
# ------------
#   record    capture pid, full argv, cwd and parent of every matching process
#   kill      stop the recorded pids (SIGTERM, then SIGKILL), verifying first
#             that each pid still runs the command we recorded
#   restore   relaunch exactly the recorded command lines, detached
#   status    what matches right now
#
# The record file is the single source of truth for restore. Nothing is ever
# written outside it: no unit files, no autostart entries, no file deletions.
#
# RUN IT ON THE JETSON it applies to, over ssh. It is plain bash + coreutils.
#
# USAGE
# -----
#   ./camera_nodes.sh status
#   ./camera_nodes.sh record  ~/go1_cam_backup.json
#   ./camera_nodes.sh kill    ~/go1_cam_backup.json
#   ./camera_nodes.sh restore ~/go1_cam_backup.json
#
#   # run it against a Jetson from the VM, in one go:
#   ssh unitree@192.168.123.13 'bash -s' < camera_nodes.sh record /tmp/cam.json
#
# DEFAULT MATCHERS (override with GO1_CAM_MATCH, space separated)
#   point_cloud_node camerarosnode live_human_pose mqttControlNode
# ============================================================================
set -uo pipefail

MATCHERS="${GO1_CAM_MATCH:-point_cloud_node camerarosnode live_human_pose mqttControlNode}"
DEFAULT_FILE="${HOME}/go1_cam_backup.json"
LOGDIR="${GO1_CAM_LOGDIR:-${HOME}/go1_cam_logs}"

# ---------------------------------------------------------------------------
die() { echo "error: $*" >&2; exit 1; }

# List pids whose argv matches any matcher. Excludes this script and its own
# process group so a matcher that happens to appear in our argv cannot make us
# kill ourselves.
matching_pids() {
    local pid
    for pid in $(ls /proc | grep -E '^[0-9]+$'); do
        [ "$pid" = "$$" ] && continue
        [ -r "/proc/$pid/cmdline" ] || continue
        local cmd
        cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null) || continue
        [ -z "${cmd// /}" ] && continue          # kernel thread or zombie
        case "$cmd" in *camera_nodes.sh*) continue ;; esac
        local m
        for m in $MATCHERS; do
            case "$cmd" in *"$m"*) echo "$pid"; break ;; esac
        done
    done
}

# JSON-escape a string (handles backslash and double quote; argv can contain
# both, and this is the only place we need escaping).
json_escape() {
    local s=$1
    s=${s//\\/\\\\}
    s=${s//\"/\\\"}
    printf '%s' "$s"
}

cmdline_of() { tr '\0' '\n' < "/proc/$1/cmdline" 2>/dev/null; }

# ---------------------------------------------------------------------------
cmd_status() {
    local pid cmd n=0
    printf '%-8s %-8s %-6s %s\n' PID PPID USER CMD
    for pid in $(matching_pids); do
        cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline")
        printf '%-8s %-8s %-6s %s\n' "$pid" \
            "$(awk '{print $4}' /proc/$pid/stat 2>/dev/null)" \
            "$(stat -c %U /proc/$pid 2>/dev/null)" "$cmd"
        n=$((n+1))
    done
    echo
    if [ "$n" -eq 0 ]; then
        echo "no matching processes."
        echo "if that is unexpected, check the matchers: GO1_CAM_MATCH='...' $0 status"
    else
        echo "$n matching process(es)."
    fi
}

cmd_record() {
    local file="${1:-$DEFAULT_FILE}"
    [ -e "$file" ] && die "$file exists -- move it aside rather than overwriting a previous record"

    local n=0
    {
        printf '{\n'
        printf '  "recorded_at": "%s",\n' "$(date -Is)"
        printf '  "hostname": "%s",\n' "$(hostname)"
        printf '  "matchers": "%s",\n' "$(json_escape "$MATCHERS")"
        printf '  "processes": [\n'
        for pid in $(matching_pids); do
            local cwd exe ppid argv=()
            cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null || echo "")
            exe=$(readlink "/proc/$pid/exe" 2>/dev/null || echo "")
            ppid=$(awk '{print $4}' /proc/$pid/stat 2>/dev/null || echo "")
            while IFS= read -r a; do argv+=("$a"); done < <(cmdline_of "$pid")

            [ "$n" -gt 0 ] && printf ',\n'
            printf '    {\n'
            printf '      "pid": %s,\n' "$pid"
            printf '      "ppid": %s,\n' "${ppid:-0}"
            printf '      "exe": "%s",\n' "$(json_escape "$exe")"
            printf '      "cwd": "%s",\n' "$(json_escape "$cwd")"
            printf '      "argv": ['
            local i
            for i in "${!argv[@]}"; do
                [ "$i" -gt 0 ] && printf ', '
                printf '"%s"' "$(json_escape "${argv[$i]}")"
            done
            printf ']\n'
            printf '    }'
            n=$((n+1))
        done
        printf '\n  ]\n'
        printf '}\n'
    } > "$file"

    echo "recorded $n process(es) to $file"
    [ "$n" -eq 0 ] && echo "warning: nothing matched -- the file records an empty set"
    return 0
}

cmd_kill() {
    local file="${1:-$DEFAULT_FILE}"
    [ -r "$file" ] || die "$file not readable -- run 'record' first"

    # pull pid + argv first element out of the record without a json parser
    local pids
    pids=$(grep -oE '"pid": *[0-9]+' "$file" | grep -oE '[0-9]+')
    [ -z "$pids" ] && { echo "nothing recorded in $file"; return 0; }

    local pid alive=()
    for pid in $pids; do
        if [ ! -d "/proc/$pid" ]; then
            echo "  pid $pid already gone"
            continue
        fi
        # PID-reuse guard: only kill if the live argv still matches the record's
        # first two tokens for that pid.
        local want have
        want=$(python3 - "$file" "$pid" <<'PY' 2>/dev/null
import json,sys
try:
    d=json.load(open(sys.argv[1]))
except Exception:
    sys.exit(0)
for p in d.get("processes",[]):
    if str(p.get("pid"))==sys.argv[2]:
        print(" ".join(p.get("argv",[])[:2]))
PY
)
        have=$(tr '\0' ' ' < "/proc/$pid/cmdline" | cut -c1-${#want})
        if [ -n "$want" ] && [ "$have" != "$want" ]; then
            echo "  pid $pid: argv no longer matches the record, skipping"
            echo "           recorded: $want"
            echo "           now     : $(tr '\0' ' ' < /proc/$pid/cmdline | cut -c1-60)"
            continue
        fi
        echo "  SIGTERM -> $pid  $(tr '\0' ' ' < /proc/$pid/cmdline | cut -c1-60)"
        kill -TERM "$pid" 2>/dev/null
        alive+=("$pid")
    done

    # give them a moment, then escalate only what is still up
    sleep 3
    for pid in "${alive[@]:-}"; do
        [ -d "/proc/$pid" ] || continue
        echo "  SIGKILL -> $pid (ignored SIGTERM)"
        kill -KILL "$pid" 2>/dev/null
    done
    sleep 1

    echo
    echo "after kill:"
    cmd_status
}

cmd_restore() {
    local file="${1:-$DEFAULT_FILE}"
    [ -r "$file" ] || die "$file not readable -- run 'record' first"
    mkdir -p "$LOGDIR"

    # All of it in one python pass: the record is JSON, so parse it as JSON
    # rather than trying to peel fields out with grep.
    python3 - "$file" "$LOGDIR" <<'PY'
import json, os, subprocess, sys
rec, logdir = sys.argv[1], sys.argv[2]
d = json.load(open(rec))
procs = d.get("processes", [])
if not procs:
    print("  the record contains no processes -- nothing to restore")
    sys.exit(0)

n = 0
for p in procs:
    argv = p.get("argv") or []
    if not argv:
        print("  skipping entry %s: empty argv" % p.get("pid"))
        continue
    cwd = p.get("cwd") or os.path.expanduser("~")
    if not os.path.isdir(cwd):
        print("  %s: cwd %s no longer exists, using %s" % (argv[0], cwd, os.path.expanduser("~")))
        cwd = os.path.expanduser("~")
    if not (os.path.isfile(argv[0]) and os.access(argv[0], os.X_OK)):
        print("  SKIP %s: not executable (was it on a path that has changed?)" % argv[0])
        continue
    log = open(os.path.join(logdir, "restore_%s.log" % p.get("pid", n)), "ab")
    # start_new_session detaches it, so it survives the ssh session that ran us
    subprocess.Popen(argv, cwd=cwd, stdout=log, stderr=log,
                     stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
    print("  relaunched: %s   (cwd %s)" % (" ".join(argv), cwd))
    n += 1
print()
print("relaunched %d process(es); logs in %s" % (n, logdir))
PY

    echo "verify with: $0 status"
}

# ---------------------------------------------------------------------------
case "${1:-}" in
    status)  cmd_status ;;
    record)  shift; cmd_record "${1:-}" ;;
    kill)    shift; cmd_kill   "${1:-}" ;;
    restore) shift; cmd_restore "${1:-}" ;;
    *)
        sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
