MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* To suit Raspberry Pi RP2040 SoC */
  BOOT_LOADER : ORIGIN = 0x10000000, LENGTH = 16k
  /* Adjust this to suit the size of your specific flash chip */
  FLASH : ORIGIN = 0x10000000 + 16k, LENGTH = 2048K - 16k - 0x100
  BOOT2 : ORIGIN = 0x10000000 + 16k + 2048K - 16k - 0x100, LENGTH = 0x100
  RAM : ORIGIN = 0x20000000, LENGTH = 264K
}

SECTIONS {

  /* ### Boot loader */
  .boot_loader ORIGIN(BOOT_LOADER) :
  {
    KEEP(*(.boot_loader*));
  } > BOOT_LOADER

} INSERT BEFORE .text;