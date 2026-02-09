#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROSDEP_RULE="${ROOT_DIR}/rosdep/local.yaml"
ROSDEP_LIST_FILE="/etc/ros/rosdep/sources.list.d/99-local-rosdep.list"
WORKSPACE_ROOT="$(cd "${ROOT_DIR}/../.." && pwd)"

echo "Registering local rosdep rule: ${ROSDEP_RULE}"
if [[ -f "${ROSDEP_LIST_FILE}" ]]; then
	if grep -Fxq "yaml file://${ROSDEP_RULE}" "${ROSDEP_LIST_FILE}"; then
		echo "Local rosdep rule already registered in ${ROSDEP_LIST_FILE}"
	else
		echo "Appending local rosdep rule to ${ROSDEP_LIST_FILE}"
		sudo sh -c "echo 'yaml file://${ROSDEP_RULE}' >> '${ROSDEP_LIST_FILE}'"
	fi
else
	echo "Creating ${ROSDEP_LIST_FILE}"
	sudo sh -c "echo 'yaml file://${ROSDEP_RULE}' > '${ROSDEP_LIST_FILE}'"
fi

rosdep update

echo "Installing rosdep dependencies from: ${WORKSPACE_ROOT}"
(cd "${WORKSPACE_ROOT}" && rosdep install --from-paths src --ignore-src --rosdistro kilted)

echo "Done."
