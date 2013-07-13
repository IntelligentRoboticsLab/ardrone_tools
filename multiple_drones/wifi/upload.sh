#!/usr/bin/env bash
set -ue

DRONEIP=${1:-"192.168.1.1"}

echo "Uploading binaries..."
curl -T wifi.ini "ftp://$DRONEIP"
curl -T auto_connect.sh "ftp://$DRONEIP"
sleep 1

{( sleep 1; echo "
  mkdir /home/default/wifi
  mv /data/video/wifi.ini /home/default/wifi
  mv /data/video/auto_connect.sh /home/default/wifi
  chmod +x /home/default/wifi/auto_connect.sh*
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "Auto connect installed."
