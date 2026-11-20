#!/bin/bash
# 停掉 start_domain_bridge.sh 起的全部管道进程。
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for f in "$HERE"/logs/pipe_*.pid; do
    [ -f "$f" ] || continue
    pid="$(cat "$f")"
    if kill -0 "$pid" 2>/dev/null; then
        kill "$pid" && echo "[stop] $(basename "$f" .pid) pid=$pid"
    fi
    rm -f "$f"
done
