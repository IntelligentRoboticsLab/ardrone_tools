EXPECTED_ARGS=2
if [ $# -ne $EXPECTED_ARGS ]
then
    echo "#####################"
    echo "ERROR: wrong arguments"
    echo ""
    echo "Call should be: echo \"./data/wifi.sh [wifiname] [ipaddress]\" | telnet 192.168.1.1"
    echo "Replace [wifiname] and [ipaddress] with the correct information"
    echo ""
    echo "#####################"
    exit
fi

echo "Connect to $1 with ip $2"
echo "-> Kill all dhcp servers"
#killall udhcpd
echo "-> Take the connection down"
#ifconfig ath0 down
echo "-> Reconfigure the drone"
#iwconfig ath0 mode managed essid $1
echo "-> Connect it to the router"
#ifconfig ath0 $2 netmask 255.255.255.0 up
