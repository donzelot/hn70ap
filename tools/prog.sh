openocd \
	-f interface/ftdi/jtag-lock-pick_tiny_2.cfg \
	-c "transport select jtag" \
	-c "reset_config srst_only connect_deassert_srst" \
	-f target/stm32f4x.cfg \
	-c "program nuttx verify reset exit"

