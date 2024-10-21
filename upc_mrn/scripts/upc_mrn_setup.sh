#!/bin/bash

main()
{
  echo " --- [ UPC_MRN_SETUP script for ETSEIB PCs] ---" 

  # This script creates a ROS2 workspace with the package upc_mrn
  
  WSPATH="${HOME}/data/ros2_ws"
  BASHFILE="${HOME}/data/bashrc.d/mrn.sh"
  touch $BASHFILE
  
  #######################Logging#######################
  DEST=${BASH_SOURCE[0]}
  FILE="${DEST##*/}"
  FILENAME="log_${FILE%.*}"
  LOGDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
  REDIRECTION="${LOGDIR}/${FILENAME}.log"
  > ${REDIRECTION}

  ROSVERSION="humble"
  MYPWD=$(pwd)
  
  ######################################################
  ###CHECK ROS
  echo " --- Checking ROS presence"
  if [ -f /opt/ros/${ROSVERSION}/setup.bash ]; then
    ROSVERSION=${ROSVERSION}
    echo " --- Found installed ROS ${ROSVERSION}"
  else
    echo "ERROR: no ${ROSVERSION1} ROS version installed found. Exiting"
    return
  fi

  #################################################################
  ### WORKSPACE

  if ! [ -d ${WSPATH}/src ];then
    echo " --- Creating ${WSPATH} workspace ..."
    mkdir -p ${WSPATH}/src
  else
    echo " --- Already found $WSPATH workspace"
  fi

  #################################################################
  ### UPC_MRN PACKAGE

  if ! [ -d ${WSPATH}/src/upc_mrn ]; then
    echo " --- Downloading upc_mrn ROS package"
    cd ${WSPATH}/src
    git clone https://gitlab.com/joanvallve/upc_mrn.git
  else
    echo " --- Updating upc_mrn ROS package"
    cd ${WSPATH}/src/upc_mrn
    git pull
  fi
  
  ######################################################
  ### BASHRC
  echo " --- Updating file ${BASHFILE}"

  add_line_bashrc "source ${WSPATH}/install/local_setup.bash"
  add_line_bashrc "source /usr/share/colcon_cd/function/colcon_cd.sh"
  add_line_bashrc "export _colcon_cd_root=/opt/ros/${ROSVERSION}/"
  add_line_bashrc "export IGN_IP=127.0.0.1"
  add_line_bashrc "export ROS_WS=${WSPATH}"
  
  # The following lines are already in ros.sh script
  #add_line_bashrc "source /opt/ros/${ROSVERSION}/setup.bash"
  #add_line_bashrc "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
  #add_line_bashrc "export ROS_LOCALHOST_ONLY=1"
  #add_line_bashrc "export ROS_DOMAIN_ID=0"
  
  source ${BASHFILE}

  ######################################################
  ### COMPILE
  echo " --- Compiling ROS workspace"
  cd ${WSPATH}
  colcon build --symlink-install &>> ${REDIRECTION} || echo "###ERROR: workspace compilation"	

  cd $MYPWD
  source ${BASHFILE}
  echo " --- Done!"
}

function add_line_bashrc #ARGS: line
{
  LINE=$1
  if ! grep -q "${LINE}" ${BASHFILE}; then
    echo " --- Adding line in ${BASHFILE}: ${LINE}"
    echo ${LINE} >> ${BASHFILE}
  fi
}

main "$@"
