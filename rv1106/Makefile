build-rp2040-firmware:
	mkdir -p fs/lib/firmware
	cd ../rp2040/multichip_rp2040_firmware/ && \
		make release
	cp ../rp2040/multichip_rp2040_firmware/target/thumbv6m-none-eabi/release/multichip-firmware-rp2040.FLASH.bin fs/lib/firmware/multichip-rp2040.release.bin
build-serial-flash:
	cd serial-flash && \
	GOOS=linux GOARCH=arm go build -o serial-flash-arm -ldflags="-s -w"
	mv serial-flash/serial-flash-arm fs/usr/bin/serial-flash
build-bins: build-serial-flash build-rp2040-firmware
update-rv1106-bins: build-bins
	chmod +x fs/usr/bin/*
	scp \
		-rp \
		fs/* root@172.32.0.93:/
