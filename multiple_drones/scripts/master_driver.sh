ip=$(ifconfig eth0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
export ROS_IP=$ip
echo "Connecting to MASTER: $ROS_MASTER_URI"
echo "My own IP is: $ROS_IP"
echo "Launching driver"
roslaunch ardrone_autonomy drone.launch id:=0 ip:=192.168.0.10
