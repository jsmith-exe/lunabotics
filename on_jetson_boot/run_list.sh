#!/bin/bash
: 'Add this to a service file, e.g. /etc/systemd/system/can0-setup.service
[Unit]
Description=Setup CAN0 interface
After=network.target

[Service]
Type=oneshot
ExecStart=/home/qpl/lunabotics/on_jetson_boot/run_list.sh
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
'

echo "Executing startup scripts..."
source /home/qpl/lunabotics/on_jetson_boot/can_setup.sh
echo "Executed startup scripts."
