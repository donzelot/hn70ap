#init required before reset becomes available
#exit required to avoid running openocd as a server
openocd \
	-f interface/ftdi/jtag-lock-pick_tiny_2.cfg \
	-c "transport select jtag" \
	-c "reset_config srst_only" \
	-f target/stm32f4x.cfg \
	-c "init" \
	-c "reset run" \
	-c "exit"

