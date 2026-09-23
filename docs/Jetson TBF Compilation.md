The Jetson may not have the sch_tbf module available by default, so you may need to compile it from source.
The sch_tbf module is used for network limiting.
Instructions, from Claude, and have worked, are as follows:

```bash
mkdir ~/sch_tbf && cd ~/sch_tbf

wget https://raw.githubusercontent.com/torvalds/linux/v5.15/net/sched/sch_tbf.c

cat > Makefile << 'EOF'
obj-m += sch_tbf.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
EOF

make
```
If it compiles cleanly:
```bash
sudo insmod sch_tbf.ko

# Persist across reboots
sudo cp sch_tbf.ko /lib/modules/$(uname -r)/kernel/net/sched/
sudo depmod -a
echo "sch_tbf" | sudo tee /etc/modules-load.d/sch_tbf.conf
```