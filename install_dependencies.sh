#!/usr/bin/env bash

################### Begin_Citation[10] ###################
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]-$0}")" && pwd)"
ROSDEP_RULE="${ROOT_DIR}/rosdep/local.yaml"
ROSDEP_LIST_FILE="/etc/ros/rosdep/sources.list.d/99-local-rosdep.list"
WORKSPACE_ROOT="$(cd "${ROOT_DIR}/../.." && pwd)"

echo "Registering local rosdep rule: ${ROSDEP_RULE}"
if [[ -f "${ROSDEP_LIST_FILE}" ]]; then
	echo "Removing any stale local rosdep entries from ${ROSDEP_LIST_FILE}"
	sudo sh -c "awk '!/^yaml file:\/\/.*\/rosdep\/local.yaml$/' '${ROSDEP_LIST_FILE}' > '${ROSDEP_LIST_FILE}.tmp' && echo 'yaml file://${ROSDEP_RULE}' >> '${ROSDEP_LIST_FILE}.tmp' && mv '${ROSDEP_LIST_FILE}.tmp' '${ROSDEP_LIST_FILE}'"
else
	echo "Creating ${ROSDEP_LIST_FILE}"
	sudo sh -c "echo 'yaml file://${ROSDEP_RULE}' > '${ROSDEP_LIST_FILE}'"
fi
################### End_Citation [10] ###################

rosdep update

echo "Cloning dependency repositories into: ${WORKSPACE_ROOT}/src"
(cd "${WORKSPACE_ROOT}/src" && vcs import --skip-existing < ${ROOT_DIR}/turtle.repos)

echo "Installing rosdep dependencies from: ${WORKSPACE_ROOT}"
(cd "${WORKSPACE_ROOT}" && rosdep install --from-paths src --ignore-src --rosdistro kilted)

echo "Done."
