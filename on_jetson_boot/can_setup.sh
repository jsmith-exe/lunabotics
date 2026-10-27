sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400
# 1 Mbit/s from the 50 MHz clock: 1 + 19 + 20 + 10 = 50 tq per bit, sample point 0.80.
# sjw 10 gives a usable resync margin; the kernel's default of 1 tq (2% of a bit) caused bus errors.
# restart-ms 1: with 100 ms, mttcan stopped transmitting after a bus-off (ENOBUFS until the link was cycled).
# txqueuelen 100 absorbs configure-time bursts.
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can tq 20 prop-seg 19 phase-seg1 20 phase-seg2 10 sjw 10 restart-ms 1 berr-reporting on
sudo ip link set can0 txqueuelen 100
sudo ip link set can0 up
echo "CAN0 is up!"
