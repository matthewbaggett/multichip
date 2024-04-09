build-padded-bootloader:
	cd ../rp2040-serial-bootloader/ \
		&& cmake . \
		&& make clean \
		&& make
	cp ../rp2040-serial-bootloader/bootloader.bin firmware/serial-bootloader.bin
	truncate -s 16k firmware/serial-bootloader.bin

build-serial-flash:
	cd ../../rv1106/serial-flash && \
		go build -o serial-flash
	cp ../../rv1106/serial-flash/serial-flash tools/serial-flash

build: build-serial-flash build-padded-bootloader
	cargo build

do-usb-flash:
	cargo run

do-serial-flash:
	cp ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040 ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.elf
	./tools/serial-flash /dev/ttyUSB0 ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.elf