#init required before reset becomes available
#exit required to avoid running openocd as a server
openocd \
	-f interface/stlink-v2.cfg \
	-c "transport select hla_swd" \
	-c "reset_config none separate" \
	-f target/stm32f4x.cfg \
	-c "init" \
	-c "reset run" \
	-c "exit"

