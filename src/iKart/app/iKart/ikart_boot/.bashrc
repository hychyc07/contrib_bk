#the following is to
#keep scp/sftp working

if [ "$PS1" ]; then
    echo "Setting up yarp and iCub env vars"
fi

# Use bash-completion, if available
if [ -f /etc/bash_completion ]; then
  . /etc/bash_completion
fi


alias ls='ls --color'
alias ll='ls -la'

code=/usr/local/src/robot

export ACE_ROOT=$code/ACE_wrappers

export YARP_DIR=$code/yarp2
export YARP_ROOT=$YARP_DIR
export YARP_CONF=~/.yarp
export YARP_BIN=$YARP_DIR/bin
export YARP_MCAST_SIZE=65000
export YARP_DGRAM_SIZE=65000

if [ -f  $YARP_ROOT/scripts/yarp_completion ]; then
  source $YARP_ROOT/scripts/yarp_completion
fi

export SDL_LINUX_JOYSTICK="'Xbox Gamepad (userspace driver)' 8 0 0"

export ICUB_ROOT=$code/iCub
export IKART_ROOT=$code/iCub/contrib/src/iKart
export IKART_DIR=$code/iCub/contrib/src/iKart/build

export ICUB_DIR=$code/iCub/main/build
export ICUB_CONF=$code/iCub/conf
export ICUB_BIN=$ICUB_DIR/bin
export ICUB_ROBOTNAME=iCubGenova03
export DEVEL_ROOT=$code/devel

export CHRIS_ROOT=$code/chris
export CHRIS_DIR=$CHRIS_ROOT
export CHRIS_BIN=$CHRIS_ROOT/bin

export ITALK_ROOT=$code/italk
export ITALK_DIR=$ITALK_ROOT
export ITALK_BIN=$ITALK_ROOT/bin

#export OPENCV_DIR=$/home/icub/lib/OpenCV-2.0.0
#export OPENCV_ROOT=$/home/icub/lib/OpenCV-2.0.0

export IPOPT_DIR=$code/ipopt/Ipopt-3.9.2/build

export SHARED_SCRIPTS_DIR=/usr/local/src/shared/scripts

export PV_BIN=$ICUB_DIR/src/primateVision/bin_64
export PV_LIB=$ICUB_DIR/src/primateVision/lib_64

export IPP_DIR=/usr/local/src/robot/ipp/

export PATH=$PATH:$YARP_BIN:$ICUB_BIN:$CHRIS_BIN:$ITALK_BIN:$SHARED_SCRIPTS_DIR:$PV_BIN
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACE_ROOT/lib:$YARP_DIR/lib:$PV_LIB
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$IPP_DIR/sharedlib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib:$IPOPT_DIR/lib/coin:$IPOPT_DIR/lib/coin/ThirdParty

#/usr/local/src/robot/iCub/main/src/tools/joystickCheck/joystick_boot.sh 


export TILERA_ROOT=/opt/tilera/TileraMDE-2.1.2.112814/tilepro
#export TILERA_ROOT=/opt/tilera/TileraMDE-3.0.1.125620/tilepro
PATH=$PATH:$TILERA_ROOT/bin
