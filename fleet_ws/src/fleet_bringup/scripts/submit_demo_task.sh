#!/usr/bin/env bash

set -euo pipefail

task_id=""
pickup="pickup_zone"
dropoff="dropoff_zone"
priority="1"
service_name="/submit_task"

usage() {
  cat <<'EOF'
Usage:
  submit_demo_task.sh [--task-id ID] [--pickup WAYPOINT] [--dropoff WAYPOINT]
                      [--priority N] [--service SERVICE]

Defaults:
  --pickup   pickup_zone
  --dropoff  dropoff_zone
  --priority 1
  --service  /submit_task

Examples:
  submit_demo_task.sh
  submit_demo_task.sh --task-id demo_001 --pickup dock_a --dropoff dock_c --priority 2

Requirements:
  Source your ROS 2 environment and workspace before running this script.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --task-id)
      task_id="${2:?missing value for --task-id}"
      shift 2
      ;;
    --pickup)
      pickup="${2:?missing value for --pickup}"
      shift 2
      ;;
    --dropoff)
      dropoff="${2:?missing value for --dropoff}"
      shift 2
      ;;
    --priority)
      priority="${2:?missing value for --priority}"
      shift 2
      ;;
    --service)
      service_name="${2:?missing value for --service}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ -z "${task_id}" ]]; then
  task_id="task_$(date +%Y%m%d_%H%M%S)"
fi

if ! [[ "${priority}" =~ ^[0-9]+$ ]]; then
  echo "priority must be an integer: ${priority}" >&2
  exit 1
fi

request="{task: {task_id: ${task_id}, pickup_waypoint: ${pickup}, dropoff_waypoint: ${dropoff}, priority: ${priority}}}"

echo "Submitting task:"
echo "  service : ${service_name}"
echo "  task_id : ${task_id}"
echo "  pickup  : ${pickup}"
echo "  dropoff : ${dropoff}"
echo "  priority: ${priority}"

ros2 service call "${service_name}" fleet_msgs/srv/SubmitTask "${request}"
