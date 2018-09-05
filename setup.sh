# This is an example from Stefanie's ethernet environment.  Copy to
# your ws root directory and change the variables appropriately.  This
# is supposed to be like ./baxter.sh.  It sets environment variables
# for your environment.
#
# The walkerbotweb.screenrc file requires this to be in the ws root.  You may
# also set any other ROS environment variables here.

source ../../devel/setup.bash
#export ROS_IP=192.168.42.1
export ROS_HOSTNAME=ubiquityrobot
#export ROS_MASTER_URI=http://<enterhostnamehere>:11311
export PS1="\[\033[00;33m\][walkerbot - ${ROS_MASTER_URI}]\[\033[00m\] $PS1"
