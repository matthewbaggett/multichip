MEMORY
{
  FLASH_BL     : ORIGIN = 0x10000000, LENGTH = 12k
  FLASH_IMGHDR : ORIGIN = 0x10000000 + 12k, LENGTH = 4k
  FLASH        : ORIGIN = 0x10000000 + 16k, LENGTH = 2048K - 16k
  RAM          : ORIGIN = 0x20000000, LENGTH = 256k
}

SECTIONS
{

    /* Insert boot3, which is the combined boot2 + boot3 */
    .boot3 : {
        KEEP (*(.boot3))
    } > FLASH_BL

    /*
     * Name a section for the image header.
     * The contents will get replaced post-build
     */
    .app_hdr : {
	LONG(0xdeaddead)
	LONG(0)
	LONG(0xdeaddead)
    } > FLASH_IMGHDR

}

