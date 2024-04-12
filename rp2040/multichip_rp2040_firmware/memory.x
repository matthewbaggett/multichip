MEMORY
{
  BOOTLOADER   : ORIGIN = 0x10000000, LENGTH = 12k
  FLASH_HEADER : ORIGIN = 0x10000000 + 12k, LENGTH = 4k
  FLASH        : ORIGIN = 0x10000000 + 16k, LENGTH = 2048K - 16k
  RAM          : ORIGIN = 0x20000000, LENGTH = 256k
}

SECTIONS
{
    .serial_bootloader : {
        KEEP (*(.serial_bootloader))
    } > BOOTLOADER

    .app_hdr : {
    	LONG(0xdeaddead)
    	LONG(0)
    	LONG(0xdeaddead)
    } > FLASH_HEADER
}
