#!/usr/bin/env bash

CATKIN_WS_PATH=${PWD}
if [[ $# -eq 1 ]]; then
  CATKIN_WS_PATH=$1
fi

echo -e "Catkin workspace path: ${CATKIN_WS_PATH}"

PKG_NAME="mps_ethercat_sdk_ros2_example"
NODE_NAME="driver_manager_node"

INSTALL_LIB_PATH="${CATKIN_WS_PATH}/install/lib"
INSTALL_NODE_PATH="${INSTALL_LIB_PATH}/${PKG_NAME}/${NODE_NAME}"
PRIVATE_DEVEL_LIB_PATH="${CATKIN_WS_PATH}/devel/.private/${PKG_NAME}/lib"
PRIVATE_DEVEL_NODE_PATH="${PRIVATE_DEVEL_LIB_PATH}/${PKG_NAME}/${NODE_NAME}"
DEVEL_LIB_PATH="${CATKIN_WS_PATH}/devel/lib"
DEVEL_NODE_PATH="${DEVEL_LIB_PATH}/${PKG_NAME}/${NODE_NAME}"

elevate_permissions () {
  NODE_PATH=$1

  sudo setcap cap_net_raw+ep ${NODE_PATH}
}

# Check install path.
if [[ -f "${INSTALL_NODE_PATH}" ]]; then
  echo -e "Give elevated permissions to ${INSTALL_NODE_PATH}"
  elevate_permissions ${INSTALL_NODE_PATH}
  exit 0
fi

# Check private devel path.
if [[ -f "${PRIVATE_DEVEL_NODE_PATH}" ]]; then
  echo -e "Give elevated permissions to ${PRIVATE_DEVEL_NODE_PATH}"
  elevate_permissions ${PRIVATE_DEVEL_NODE_PATH}
  exit 0
fi

# Check devel path.
if [[ -f "${DEVEL_NODE_PATH}" ]]; then
  echo -e "Give elevated permissions to ${DEVEL_NODE_PATH}"
  elevate_permissions ${DEVEL_NODE_PATH}
  exit 0
fi

echo -e "\e[31mCould not give elevated permissions to the ${NODE_NAME}. Did you run the script in your catkin_ws or give the catkin_ws path as an argument?\e[39m"
exit 1
