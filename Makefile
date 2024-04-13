.PHONY: rv1106 rp2040
rv1106:

rp2040:
	#make -C rv1106 update-rv1106-bins
	make -C rp2040/multichip_rp2040_firmware do-indirect-flash

rp2040-reset:
	ssh rv1106 -C "rp2040-flash-disable; rp2040-reset"
