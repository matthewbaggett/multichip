build-bootloader:
	cd ../rp2040-serial-bootloader/ \
		&& PICO_SDK_FETCH_FROM_GIT=yes cmake . \
		&& cmake . \
		&& make clean \
		&& make
	cp ../rp2040-serial-bootloader/bootloader.bin firmware/serial-bootloader.bin

build-serial-flash:
	cd ../../rv1106/serial-flash && \
		go build -o ../../rp2040/multichip-firmware-rp2040/tools/serial-flash-amd64

build-deps: build-bootloader
build: build-deps
	cargo build
	cp ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040 ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.elf
	/usr/lib/arm-none-eabi/bin/objcopy \
		-Obinary \
		--only-section=.vector_table \
		--only-section=.text \
		--only-section=.rodata \
		./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.elf \
		./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.FLASH.bin
release:
	cargo build --release
	cp ./target/thumbv6m-none-eabi/release/multichip-firmware-rp2040 ./target/thumbv6m-none-eabi/release/multichip-firmware-rp2040.elf
	/usr/lib/arm-none-eabi/bin/objcopy \
		-Obinary \
		--only-section=.vector_table \
		--only-section=.text \
		--only-section=.rodata \
		./target/thumbv6m-none-eabi/release/multichip-firmware-rp2040.elf \
		./target/thumbv6m-none-eabi/release/multichip-firmware-rp2040.FLASH.bin

do-usb-flash: build
	../rp2040-serial-bootloader/gen_imghdr.py \
		-a 0x10004000 \
		./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.FLASH.bin \
		./target/thumbv6m-none-eabi/debug/bootloader_header
	/usr/lib/arm-none-eabi/bin/objcopy \
		--update-section .app_hdr=./target/thumbv6m-none-eabi/debug/bootloader_header \
		./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.elf \
		./target/thumbv6m-none-eabi/debug/combined-firmware.elf
	elf2uf2-rs --deploy ./target/thumbv6m-none-eabi/debug/combined-firmware.elf

do-serial-flash: build
	./tools/serial-flash /dev/ttyUSB0 ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.FLASH.bin 0x10004000

test-all-flash:
	$(MAKE) do-usb-flash
	sleep 5;
	$(MAKE) do-serial-flash

update-rv1106-bins:
	$(MAKE) -C ../../rv1106 update-rv1106-bins
do-indirect-flash: update-rv1106-bins build
	scp ./target/thumbv6m-none-eabi/debug/multichip-firmware-rp2040.FLASH.bin rv1106:/tmp/new-firmware.bin
	ssh rv1106 -C "rp2040-update /tmp/new-firmware.bin"
