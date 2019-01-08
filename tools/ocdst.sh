openocd \
	-f interface/stlink-v2.cfg \
	-c "transport select hla_swd" \
	-c "reset_config none separate" \
	-f target/stm32f4x.cfg
