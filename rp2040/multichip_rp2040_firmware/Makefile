build-padded-bootloader:
	cd ../rp2040-serial-bootloader/ \
		&& PICO_SDK_FETCH_FROM_GIT=yes cmake . \
		&& cmake . \
		&& make clean \
		&& make
	cp ../rp2040-serial-bootloader/bootloader.bin firmware/serial-bootloader.bin
	truncate -s 16k firmware/serial-bootloader.bin

build-serial-flash:
	cd ../../rv1106/serial-flash && \
		go build -o serial-flash
	cp ../../rv1106/serial-flash/serial-flash tools/serial-flash

build-deps: build-serial-flash build-padded-bootloader
build:
	cargo build

do-usb-flash:
	cargo run

do-serial-flash: build
	cp ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040 ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.elf
	./tools/serial-flash \
		/dev/ttyUSB0 \
		./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.elf

build-generic-bootloaders:
	cp target/thumbv6m-none-eabi/debug/build/rp2040-boot2-*/out/boot2_generic_03h.padded.bin firmware/generic-bootloader.bin
	cp firmware/generic-bootloader.bin firmware/generic-bootloader.16k.bin
	truncate -s 16k firmware/generic-bootloader.16k.bin