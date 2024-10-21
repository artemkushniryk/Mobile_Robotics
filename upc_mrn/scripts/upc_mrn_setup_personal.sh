#!/bin/bash

main()
{
  echo " --- [ UPC_MRN_SETUP script ] ---" 

  # This script creates a ROS2 workspace with the package upc_mrn
  
  WSNAME="ros2_ws"

  ## check if is sudoer
  if ! $(sudo mkdir test_folder &> /dev/null); then
    echo 'User has no root privileges'
    SUDOER=false
  else
    echo 'User has root privileges'
    sudo rm -rf test_folder
    if [ "$UID" -eq 0 -o "$EUID" -eq 0 ]; then
      SUDO=false
    else
      SUDO=true
    fi
    SUDOER=true
  fi
  
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
  ###INSTALL ADDITIONAL PACKAGES
  
  if [ $SUDOER == "true" ]; then
    echo " --- Update apt"
    if [ $SUDO == "true" ]; then
      sudo apt-get update &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
    else
      apt-get install update &>> ${REDIRECTION} || echo "###ERROR, for more info check log file: $LOGDIR/$FILENAME.log"
    fi

    echo " --- Installing extra packages"
    add_apt_pkg vim
    add_apt_pkg ssh
    add_apt_pkg git
    add_apt_pkg python3-colcon-common-extensions
    add_apt_pkg ignition-fortress
  fi

  #################################################################
  ### WORKSPACE

  if ! [ -d ~/${WSNAME}/src ];then
    echo " --- Creating ~/${WSNAME} workspace ..."
    mkdir -p ~/${WSNAME}/src
  else
    echo " --- Already found ~/$WSNAME workspace"
  fi

  #################################################################
  ###ADD ROS PACKAGES
  
  echo " --- Installing extra ros packages"
  if [ $SUDOER == "true" ]; then
    add_apt_rospkg rplidar-ros 
    add_apt_rospkg rqt-tf-tree
    add_apt_rospkg teleop-twist-keyboard
    add_apt_rospkg turtlebot4-simulator 
    add_apt_rospkg turtlebot4-desktop 
    add_apt_rospkg turtlebot4-navigation 
    add_apt_rospkg turtlebot4-node 
    add_apt_rospkg turtlebot4-tutorials
  fi

  if ! [ -d ~/${WSNAME}/src/upc_mrn ]; then
    echo " --- Downloading upc_mrn ROS package"
    cd ~/${WSNAME}/src
    git clone https://gitlab.com/joanvallve/upc_mrn.git
  else
    echo " --- Updating upc_mrn ROS package"
    cd ~/${WSNAME}/src/upc_mrn
    git pull
  fi
  
  ######################################################
  ### BASHRC
  echo " --- Updating file ~./bashrc"

  add_line_bashrc "source /opt/ros/${ROSVERSION}/setup.bash"
  add_line_bashrc "source ~/${WSNAME}/install/local_setup.bash"
  add_line_bashrc "source /usr/share/colcon_cd/function/colcon_cd.sh"
  add_line_bashrc "export _colcon_cd_root=/opt/ros/${ROSVERSION}/"
  add_line_bashrc "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
  add_line_bashrc "export ROS_LOCALHOST_ONLY=1"
  add_line_bashrc "export ROS_DOMAIN_ID=0"
  add_line_bashrc "export ROS_WS=~/${WSNAME}"
  add_line_bashrc "export IGN_IP=127.0.0.1"
  
  source ~/.bashrc

  ######################################################
  ### COMPILE
  echo " --- Compiling ROS workspace"
  cd ~/${WSNAME}
  colcon build --symlink-install &>> ${REDIRECTION} || echo "###ERROR: workspace compilation"	

  cd $MYPWD
  source ~/.bashrc
  echo " --- Done!"
}

function add_apt_pkg #ARGS: pkg
{
  PKG=$1
  echo " --- Installing apt package ${PKG}"
  if [ $SUDO == "true" ]; then
    sudo apt-get install -y ${PKG} &>> ${REDIRECTION} || echo "###ERROR Installing apt-get package ${PKG}, for more info check log file: $LOGDIR/$FILENAME.log"
  else
    apt-get install -y ${PKG} &>> ${REDIRECTION} || echo "###ERROR Installing apt-get package ${PKG}, for more info check log file: $LOGDIR/$FILENAME.log"
  fi
}

function add_apt_rospkg #ARGS: pkg
{
  PKG=$1
  add_apt_pkg ros-${ROSVERSION}-${PKG}
}

function add_line_bashrc #ARGS: line
{
  LINE=$1
  if ! grep -q "${LINE}" ~/.bashrc; then
    echo " --- Adding line in .bashrc: ${LINE}"
    echo ${LINE} >> ~/.bashrc
  fi
}

main "$@"