#!/usr/bin/env bash

set -euo pipefail

launch_log="$(mktemp)"
status_log="$(mktemp)"
launch_pid=""
status_pid=""
first_task_id="conflict_a_$$"
second_task_id="conflict_b_$$"

cleanup() {
  if [[ -n "${status_pid}" ]] && kill -0 "${status_pid}" 2>/dev/null; then
    kill "${status_pid}" 2>/dev/null || true
    wait "${status_pid}" 2>/dev/null || true
  fi
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  rm -f "${launch_log}" "${status_log}"
}
trap cleanup EXIT

wait_for_service() {
  local service_name="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      return 0
    fi
    sleep 1
  done

  return 1
}

wait_for_log() {
  local pattern="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if grep -Fq "${pattern}" "${launch_log}"; then
      return 0
    fi
    sleep 1
  done

  return 1
}

task_has_status() {
  python3 - "$status_log" "$1" "$2" <<'PY'
from pathlib import Path
import sys
path, task_id, status = sys.argv[1:]
text = Path(path).read_text() if Path(path).exists() else ''
blocks = [b for b in text.split('---') if b.strip()]
for block in blocks:
    if f'task_id: {task_id}' in block and f'status: {status}' in block:
        raise SystemExit(0)
raise SystemExit(1)
PY
}

wait_for_task_status() {
  local task_id="$1"
  local status="$2"
  local timeout_seconds="$3"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if task_has_status "$task_id" "$status"; then
      return 0
    fi
    sleep 1
  done

  return 1
}

print_failure_context() {
  echo "Conflict reservation check failed. Recent launch output:" >&2
  tail -n 120 "${launch_log}" >&2 || true
  if [[ -f "${status_log}" ]]; then
    echo "Captured task status output:" >&2
    cat "${status_log}" >&2 || true
  fi
}

echo "Launching demo stack for conflict reservation check..."
ros2 launch fleet_bringup demo.launch.py >"${launch_log}" 2>&1 &
launch_pid=$!

if ! wait_for_service "/submit_task" 20; then
  print_failure_context
  echo "submit_task service did not become available" >&2
  exit 1
fi

echo "Subscribing for task lifecycle status reporting..."
timeout 60s ros2 topic echo /task_statuses >"${status_log}" 2>&1 &
status_pid=$!
sleep 1

echo "Submitting first task ${first_task_id}..."
ros2 run fleet_bringup submit_demo_task.sh --task-id "${first_task_id}"

for status in queued assigned executing; do
  if ! wait_for_task_status "${first_task_id}" "${status}" 20; then
    print_failure_context
    echo "first task never reached status ${status}" >&2
    exit 1
  fi
done

echo "Submitting second conflicting task ${second_task_id}..."
ros2 run fleet_bringup submit_demo_task.sh --task-id "${second_task_id}"

if ! wait_for_task_status "${second_task_id}" "queued" 20; then
  print_failure_context
  echo "second task was not queued" >&2
  exit 1
fi

if ! wait_for_task_status "${second_task_id}" "waiting" 20; then
  print_failure_context
  echo "second task was not postponed by reservations" >&2
  exit 1
fi

if ! wait_for_log "route is blocked by reservations" 20; then
  print_failure_context
  echo "reservation conflict was not logged by fleet_manager" >&2
  exit 1
fi

if ! wait_for_task_status "${first_task_id}" "completed" 20; then
  print_failure_context
  echo "first task did not complete" >&2
  exit 1
fi

for status in assigned executing completed; do
  if ! wait_for_task_status "${second_task_id}" "${status}" 20; then
    print_failure_context
    echo "second task never reached status ${status} after reservations cleared" >&2
    exit 1
  fi
done

echo "Conflict reservation check passed."
