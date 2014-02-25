#!/bin/bash

# set here the Bluetooth address of the robot bt2CAN adapter
BT_ADDRESS='10:00:E8:6C:F0:4F'

# if you want to use a specific local bluetooth adapter, 
# specify here its BT address
LOCAL_BT_INTERFACE='hci0'

EXEC='asebamedulla'
#EXEC='asebaswitch'

RFCOMM='0'
device="/dev/rfcomm$RFCOMM"

# bind to rfcomm
if [ -e $device ]; then 
        echo "$device already exists. Fine, using it."
else
        if [[ -n $LOCAL_BT_INTERFACE ]]; then 
            echo "Binding to $device from local adapter $LOCAL_BT_INTERFACE ..."
            sudo rfcomm -i $LOCAL_BT_INTERFACE bind $RFCOMM $BT_ADDRESS
        else
            echo "Binding to $device ..."
            sudo rfcomm bind $RFCOMM $BT_ADDRESS
        fi
fi

echo "Connecting to target..."
$EXEC -v "ser:device=$device;fc=hard;baud=921600"

