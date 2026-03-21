#!/usr/bin/env bash

set -euo pipefail

launch_log="$(mktemp)"
status_log="$(mktemp)"
launch_pid=""
status_pid=""
invalid_task_id="invalid_$$"
valid_task_id="valid_$$"

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

wait_for_status_pattern() {
  local pattern="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while (( SECONDS < deadline )); do
    if grep -Fq "${pattern}" "${status_log}"; then
      return 0
    fi
    sleep 1
  done

  return 1
}

print_failure_context() {
  echo "Integration check failed. Recent launch output:" >&2
  tail -n 80 "${launch_log}" >&2 || true
  if [[ -f "${status_log}" ]]; then
    echo "Captured task status output:" >&2
    cat "${status_log}" >&2 || true
  fi
}

echo "Launching demo stack for planner-failure recovery check..."
ros2 launch fleet_bringup demo.launch.py >"${launch_log}" 2>&1 &
launch_pid=$!

if ! wait_for_service "/submit_task" 20; then
  print_failure_context
  echo "submit_task service did not become available" >&2
  exit 1
fi

echo "Subscribing for task lifecycle status reporting..."
timeout 40s ros2 topic echo /task_statuses >"${status_log}" 2>&1 &
status_pid=$!
sleep 1

echo "Submitting intentionally invalid task ${invalid_task_id}..."
ros2 run fleet_bringup submit_demo_task.sh \
  --task-id "${invalid_task_id}" \
  --pickup missing_waypoint \
  --dropoff dropoff_zone

if ! wait_for_log "planner rejected task ${invalid_task_id}" 20; then
  print_failure_context
  echo "planner rejection was not observed for ${invalid_task_id}" >&2
  exit 1
fi

for pattern in \
  "task_id: ${invalid_task_id}" \
  "status: queued" \
  "status: failed" \
  "message: unable to plan route on configured graph"
do
  if ! wait_for_status_pattern "${pattern}" 20; then
    print_failure_context
    echo "missing lifecycle status pattern: ${pattern}" >&2
    exit 1
  fi
done

echo "Submitting valid follow-up task ${valid_task_id}..."
ros2 run fleet_bringup submit_demo_task.sh --task-id "${valid_task_id}"

if ! wait_for_log "assigned task ${valid_task_id}" 20; then
  print_failure_context
  echo "valid task ${valid_task_id} was not assigned after planner rejection" >&2
  exit 1
fi

for pattern in \
  "task_id: ${valid_task_id}" \
  "status: assigned" \
  "status: executing" \
  "status: completed"
do
  if ! wait_for_status_pattern "${pattern}" 20; then
    print_failure_context
    echo "missing lifecycle status pattern: ${pattern}" >&2
    exit 1
  fi
done

if ! wait_for_log "published completed state" 20; then
  print_failure_context
  echo "no task completion was observed after assigning ${valid_task_id}" >&2
  exit 1
fi

echo "Planner-failure recovery check passed."
