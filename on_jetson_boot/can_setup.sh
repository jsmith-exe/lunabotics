sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
sudo ip link set can0 up type can bitrate 1000000 restart-ms 1000
echo "CAN0 is up!"