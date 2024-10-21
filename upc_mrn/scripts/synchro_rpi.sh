#!/bin/bash

echo " --- [ SYNCHRONIZATION script for Turtlebot4's Raspberry Pi] ---" 
turtlebot_ip=10.10.73.24${ROS_DOMAIN_ID}
echo "RPi IP: $turtlebot_ip"

# compute offset in milliseconds
time_offset=$(($(ssh ubuntu@${turtlebot_ip} date +%s%N)/1000000 - $(date +%s%N)/1000000))
echo "time offset: ${time_offset} ms"

# remove sign
if [[ $time_offset -lt 0 ]]; then
  echo "negative offset"
  time_offset=${time_offset#-}
  negative=true
else
  echo "positive offset"
  negative=false
fi

# convert to seconds
time_offset_s=0$(bc <<< "scale=3; $time_offset / 1000")
echo "time offset: ${time_offset_s} s"

# adjust RPi time
if [[ "$negative" = true ]]; then
  echo "correcting negative offset..."
  ssh ubuntu@$turtlebot_ip "echo turtlebot4 | sudo -S date -s '${time_offset_s} second' && exit"
else
  echo "correcting positive offset..."
  ssh ubuntu@$turtlebot_ip "echo turtlebot4 | sudo -S date -s '${time_offset_s} second ago' && exit"
fi

# check offset again
time_offset=$(($(ssh ubuntu@${turtlebot_ip} date +%s%N)/1000000 - $(date +%s%N)/1000000))
echo "New time offset: ${time_offset} ms"
