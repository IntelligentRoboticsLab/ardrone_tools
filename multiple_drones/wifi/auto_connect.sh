#!/bin/sh

# Script to automatically connect to a network
echo "Auto connect"
echo "Automatically connecting to a network when detected!"

# Script information
SCRIPT=`readlink -f $0`                                                 
INSTALL_PATH=`dirname $SCRIPT`                                          
LOG=$INSTALL_PATH/log 

# Config ini containing the basic information
CONFIG="$INSTALL_PATH/wifi.ini"

# Get the information of the network
ESSID=`grep essid $CONFIG | awk -F "=" '{print $2}'`
ESSID=`echo $ESSID`
PASSWORD=`grep password $CONFIG | awk -F "=" '{print $2}'`
PASSWORD=`echo $PASSWORD`

# Original setup drone
DRONE_SSID=`grep ssid_single_player /data/config.ini | awk -F "=" '{print $2}'`
DRONE_SSID=`echo $DRONE_SSID`
DRONE_ADDRESS=192.168.1.
DRONE_LAST_NUMBERS="2 3 4 5"

RECONNECT=0

if [ -n "$DRONE_SSID" ]
then
    echo "Drone SSID is found: $DRONE_SSID" >> $LOG
else
    echo "Drone SSID is not found: ardrone_wifi" >> $LOG
    DRONE_SSID=ardrone_wifi
fi

date >> $LOG
echo "" >> $LOG
echo "Network Configuration" >> $LOG
echo "SSID: $ESSID" >> $LOG
echo "PASSWORD $PASSWORD" >> $LOG
echo "DRONE SSID: $DRONE_SSID" >> $LOG

while [ 1 ]
do
    WIFI_CONFIG=`iwconfig ath0`

    if echo $WIFI_CONFIG | grep -q "Master" ; then
        CONNECTED=0

        for i in $DRONE_LAST_NUMBERS
        do
            if ping -W 1 -c 1 -q $DRONE_ADDRESS$i ; then
                CONNECTED=1
                break
            fi
        done

        if [ "$CONNECTED" -eq 1 ] ; then
            echo "Connected to device" >> $LOG
            echo "Connected to device"
            date >> $LOG
            sleep 10
            continue
        fi
    fi
    echo $WIFI_CONFIG
    echo `date`
    if `echo $WIFI_CONFIG | grep -q "Signal level:-96 dBm"` || `echo $WIFI_CONFIG | grep -q "Master"` ; then
        iwconfig ath0 mode managed
        NETWORKS=`iwlist ath0 scan`

        if echo $NETWORKS | grep -q $ESSID;
        then
            # connect or reconnect
            echo "Connect/Reconnecting"
            echo "Connected to network" >> $LOG
            wpa_passphrase $ESSID $PASSWORD > /etc/wpa_supplicant.conf
            ifconfig ath0 0.0.0.0; iwconfig ath0 essid '$ESSID' && wpa_supplicant -B -Dwext -iath0 -c/etc/wpa_supplicant.conf; wait 5; /sbin/udhcpc -i ath0;
            RECONNECT=1
        else
            echo "Network not found" >> $LOG
            echo "Hosting new network"
            iwconfig ath0 mode master
            if [ "$RECONNECT" -eq 1 ] ; then
                kill udhcpc
                udhcpd /tmp/udhcpd.conf
                ifconfig ath0 down
                iwconfig ath0 mode master essid $DRONE_SSID channel auto commit
                ifconfig ath0 192.168.1.1 netmask 255.255.255.0 up
                RECONNECT=0
            fi
        fi
    else
        # Connected
        sleep 10
    fi
done
