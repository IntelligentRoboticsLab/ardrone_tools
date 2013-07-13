EXPECTED_ARGS=1
if [ $# -ne $EXPECTED_ARGS ]
then
    echo "#####################"
    echo "ERROR: wrong arguments"
    echo ""
    echo "Call should be: ./slave.sh [ipadress]"
    echo "Replace [ipaddress] with the ip address of the MASTER"
    echo ""
    echo "#####################"
    exit
fi

export ROS_MASTER_URI=http://$1:11311
ip=$(ifconfig wlan0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
export ROS_IP=$ip
echo "Connecting to MASTER: $ROS_MASTER_URI"
echo "My own IP is: $ROS_IP"
echo "Launching driver"
roslaunch ardrone_autonomy drone.launch id:=1 ip:=192.168.0.11
