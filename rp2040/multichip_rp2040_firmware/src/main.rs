#![no_std]
#![no_main]

use core::cell::RefCell;
use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::i2c::{self, Config, InterruptHandler};
use embassy_rp::peripherals::{I2C0, PIO0, USB};
use embassy_rp::spi;
use embassy_rp::spi::{Blocking, Spi};
use embassy_rp::uart;
use embassy_rp::usb::{Driver, Instance};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Ticker};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb_logger;
use embedded_graphics::image::{Image, ImageRawLE};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::*;
use embedded_hal_async::i2c::I2c;
//use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use crate::my_display_interface::SPIDeviceInterface;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::text::Text;
use st7789::{Orientation, ST7789};
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".serial_bootloader"]
#[used]
pub static BOOTLOADER: [u8; 11316] = *include_bytes!("../firmware/serial-bootloader.bin");

bind_interrupts!(struct Irqs {
        I2C0_IRQ => InterruptHandler<I2C0>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

type ButtonType = Mutex<ThreadModeRawMutex, Option<Input<'static>>>;
type JogballButtonType = Mutex<ThreadModeRawMutex, Option<Input<'static>>>;
static JOGBALL_UP: JogballButtonType = Mutex::new(None);
static JOGBALL_DOWN: JogballButtonType = Mutex::new(None);
static JOGBALL_LEFT: JogballButtonType = Mutex::new(None);
static JOGBALL_RIGHT: JogballButtonType = Mutex::new(None);
static JOGBALL_CLICK: ButtonType = Mutex::new(None);
static USER_BUTTON_CLICK: ButtonType = Mutex::new(None);

#[derive(Clone, Copy)]
enum HumanInput {
    JogUp,
    JogDown,
    JogLeft,
    JogRight,
    JogClick,
    UserButtonClick,
}
static CHANNEL_JOGBALL: Channel<ThreadModeRawMutex, HumanInput, 64> = Channel::new();

type LedType = Mutex<ThreadModeRawMutex, Option<Output<'static>>>;
static DEBUG_LED: LedType = Mutex::new(None);
static WHITE_LED: LedType = Mutex::new(None);
static GREEN_LED: LedType = Mutex::new(None);
static RED_LED: LedType = Mutex::new(None);
static BLUE_LED: LedType = Mutex::new(None);
static BACKLIGHT: LedType = Mutex::new(None);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello there!");

    let p = embassy_rp::init(Default::default());

    let uart_config = uart::Config::default();
    let mut uart = uart::Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, uart_config);
    uart.blocking_write("Hello World!\r\n".as_bytes()).unwrap();

    // Configure button/jogball inputs
    let jogball_up = Input::new(p.PIN_7, Pull::None);
    let jogball_down = Input::new(p.PIN_8, Pull::None);
    let jogball_left = Input::new(p.PIN_3, Pull::None);
    let jogball_right = Input::new(p.PIN_6, Pull::None); // verified
    let jogball_click = Input::new(p.PIN_2, Pull::Up);
    let user_button_click = Input::new(p.PIN_24, Pull::Up);
    {
        // inner scope is so that once the mutex is written to, the MutexGuard is dropped, thus the Mutex is released
        *(JOGBALL_UP.lock().await) = Some(jogball_up);
        *(JOGBALL_DOWN.lock().await) = Some(jogball_down);
        *(JOGBALL_LEFT.lock().await) = Some(jogball_left);
        *(JOGBALL_RIGHT.lock().await) = Some(jogball_right);
        *(JOGBALL_CLICK.lock().await) = Some(jogball_click);
        *(USER_BUTTON_CLICK.lock().await) = Some(user_button_click);
    }
    let jogball_fut = async {
        log::info!("Awaiting Jogball changes");
        loop {
            match CHANNEL_JOGBALL.receive().await {
                HumanInput::JogClick => log::info!("Jogball Click"),
                HumanInput::JogUp => log::info!("Jogball Up"),
                HumanInput::JogDown => log::info!("Jogball Down"),
                HumanInput::JogLeft => log::info!("Jogball Left"),
                HumanInput::JogRight => log::info!("Jogball Right"),
                HumanInput::UserButtonClick => log::info!("User button clicked"),
            }

            match CHANNEL_JOGBALL.receive().await {
                HumanInput::JogClick => uart.blocking_write("Jogball Click\r\n".as_bytes()).unwrap(),
                HumanInput::JogUp => uart.blocking_write("Jogball Up\r\n".as_bytes()).unwrap(),
                HumanInput::JogDown => uart.blocking_write("Jogball Down\r\n".as_bytes()).unwrap(),
                HumanInput::JogLeft => uart.blocking_write("Jogball Left\r\n".as_bytes()).unwrap(),
                HumanInput::JogRight => uart.blocking_write("Jogball Right\r\n".as_bytes()).unwrap(),
                HumanInput::UserButtonClick => uart.blocking_write("User button clicked\r\n".as_bytes()).unwrap(),
            }
        }
    };

    // Configure various directly driven LEDs
    let debug_led = Output::new(AnyPin::from(p.PIN_25), Level::Low);
    let white_led = Output::new(AnyPin::from(p.PIN_9), Level::Low);
    let green_led = Output::new(AnyPin::from(p.PIN_10), Level::Low);
    let red_led = Output::new(AnyPin::from(p.PIN_12), Level::Low);
    let blue_led = Output::new(AnyPin::from(p.PIN_11), Level::Low);
    let mut backlight = Output::new(AnyPin::from(p.PIN_13), Level::Low);
    backlight.set_high();
    {
        // inner scope is so that once the mutex is written to, the MutexGuard is dropped, thus the Mutex is released
        *(DEBUG_LED.lock().await) = Some(debug_led);
        *(WHITE_LED.lock().await) = Some(white_led);
        *(GREEN_LED.lock().await) = Some(green_led);
        *(RED_LED.lock().await) = Some(red_led);
        *(BLUE_LED.lock().await) = Some(blue_led);
        *(BACKLIGHT.lock().await) = Some(backlight);
    }

    // i2c stuff
    let i2c_sda = p.PIN_4;
    let i2c_scl = p.PIN_5;

    info!("set up i2c ");
    let mut i2c = i2c::I2c::new_async(p.I2C0, i2c_scl, i2c_sda, Irqs, Config::default());

    // SPI stuff
    let spi_miso = p.PIN_16; // not used
    let spi_mosi = p.PIN_19;
    let spi_clk = p.PIN_18;

    // Display stuff
    let disp_reset = p.PIN_21;
    let disp_dc = p.PIN_20; // Display Data/Command pin.
    let disp_select = p.PIN_17; // not used.

    let mut display_config = spi::Config::default();
    display_config.frequency = 64_000_000;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;

    let spi: Spi<'_, _, Blocking> = Spi::new_blocking(p.SPI0, spi_clk, spi_mosi, spi_miso, display_config.clone());
    let spi_bus: embassy_sync::blocking_mutex::Mutex<NoopRawMutex, _> = embassy_sync::blocking_mutex::Mutex::new(RefCell::new(spi));

    let display_spi = SpiDeviceWithConfig::new(&spi_bus, Output::new(disp_select, Level::High), display_config);

    let disp_dc = Output::new(disp_dc, Level::Low); // Data/Command pin. 0 = command, 1 = data
    let disp_reset = Output::new(disp_reset, Level::Low);

    // display interface abstraction from SPI and DC
    let di = SPIDeviceInterface::new(display_spi, disp_dc);

    // create driver
    let mut display = ST7789::new(di, disp_reset, 240, 240);

    // initialize
    display.init(&mut Delay).unwrap();

    // set default orientation
    display.set_orientation(Orientation::Landscape).unwrap();

    display.clear(Rgb565::BLACK).unwrap();

    let raw_image_data = ImageRawLE::new(include_bytes!("./ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(34, 68));

    // Display a test image
    ferris.draw(&mut display).unwrap();

    // Write some test text.
    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    Text::new("Hello embedded_graphics \n + embassy + RP2040!", Point::new(20, 200), style).draw(&mut display).unwrap();

    // Set up USB bits
    // Create the driver, from the HAL.
    let usb_driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("MultiChip");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();
    let mut logger_state = State::new();

    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let serial_logger_acm_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);
    let mut serial_acm_class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Creates the logger and returns the logger future
    // Note: You'll need to use log::info! afterwards instead of info! for this to work (this also applies to all the other log::* macros)
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, serial_logger_acm_class);

    // Build the builder.
    let mut usb = builder.build();

    let usb_fut = usb.run();

    let echo_fut = async {
        loop {
            serial_acm_class.wait_connection().await;
            log::info!("Connected");
            let _ = echo(&mut serial_acm_class).await;
            log::info!("Disconnected");
        }
    };

    // Led Tasks
    let multiple = 5;
    unwrap!(spawner.spawn(toggle_led(&DEBUG_LED, Duration::from_millis(240) * multiple)));
    unwrap!(spawner.spawn(toggle_led(&WHITE_LED, Duration::from_millis(100) * multiple)));
    unwrap!(spawner.spawn(toggle_led(&GREEN_LED, Duration::from_millis(90) * multiple)));
    //unwrap!(spawner.spawn(toggle_led(&RED_LED,   Duration::from_millis(80) * multiple)));
    //unwrap!(spawner.spawn(toggle_led(&BLUE_LED,  Duration::from_millis(110) * multiple)));
    //unwrap!(spawner.spawn(toggle_led(&BACKLIGHT, Duration::from_millis(150) * multiple)));

    // Jogball tasks
    unwrap!(spawner.spawn(jogball(&JOGBALL_UP, CHANNEL_JOGBALL.sender(), HumanInput::JogUp)));
    unwrap!(spawner.spawn(jogball(&JOGBALL_DOWN, CHANNEL_JOGBALL.sender(), HumanInput::JogDown)));
    unwrap!(spawner.spawn(jogball(&JOGBALL_LEFT, CHANNEL_JOGBALL.sender(), HumanInput::JogLeft)));
    unwrap!(spawner.spawn(jogball(&JOGBALL_RIGHT, CHANNEL_JOGBALL.sender(), HumanInput::JogRight)));
    unwrap!(spawner.spawn(click(&JOGBALL_CLICK, CHANNEL_JOGBALL.sender(), HumanInput::JogClick)));
    unwrap!(spawner.spawn(click(&USER_BUTTON_CLICK, CHANNEL_JOGBALL.sender(), HumanInput::UserButtonClick)));

    log::info!("Starting up MultiChip on RP2040!");
    // Run everything concurrently.

    embassy_futures::join::join3(
        jogball_fut,
        usb_fut,
        //uart_heartbeat,
        join(echo_fut, log_fut),
    )
    .await;
}

#[embassy_executor::task(pool_size = 4)]
async fn jogball(jogball_btn: &'static JogballButtonType, control: Sender<'static, ThreadModeRawMutex, HumanInput, 64>, direction: HumanInput) {
    loop {
        let mut jogball_btn_unlocked = jogball_btn.lock().await;
        jogball_btn_unlocked.as_mut().unwrap().wait_for_any_edge().await;
        control.send(direction).await;
    }
}
#[embassy_executor::task(pool_size = 2)]
async fn click(btn: &'static ButtonType, control: Sender<'static, ThreadModeRawMutex, HumanInput, 64>, direction: HumanInput) {
    loop {
        let mut btn_unlocked = btn.lock().await;
        btn_unlocked.as_mut().unwrap().wait_for_falling_edge().await;
        control.send(direction).await;
    }
}

#[embassy_executor::task(pool_size = 6)]
async fn toggle_led(led: &'static LedType, delay: Duration) {
    let mut ticker = Ticker::every(delay);
    loop {
        {
            let mut led_unlocked = led.lock().await;
            //log::info!("Toggling LED");
            if let Some(pin_ref) = led_unlocked.as_mut() {
                pin_ref.toggle();
            }
        }

        ticker.next().await;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    info!("Entered echo async");
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];

        info!("data: {:x}", data);
        for _ in 1..2 {
            class.write_packet(data).await?;
        }
    }
}
mod my_display_interface {
    use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
    use embedded_hal_1::digital::OutputPin;
    use embedded_hal_1::spi::SpiDevice;

    /// SPI display interface.
    ///
    /// This combines the SPI peripheral and a data/command pin
    pub struct SPIDeviceInterface<SPI, DC> {
        spi: SPI,
        dc: DC,
    }

    impl<SPI, DC> SPIDeviceInterface<SPI, DC>
    where
        SPI: SpiDevice,
        DC: OutputPin,
    {
        /// Create new SPI interface for communciation with a display driver
        pub fn new(spi: SPI, dc: DC) -> Self {
            Self { spi, dc }
        }
    }

    impl<SPI, DC> WriteOnlyDataCommand for SPIDeviceInterface<SPI, DC>
    where
        SPI: SpiDevice,
        DC: OutputPin,
    {
        fn send_commands(&mut self, cmds: DataFormat<'_>) -> Result<(), DisplayError> {
            // 1 = data, 0 = command
            self.dc.set_low().map_err(|_| DisplayError::DCError)?;

            send_u8(&mut self.spi, cmds).map_err(|_| DisplayError::BusWriteError)?;
            Ok(())
        }

        fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
            // 1 = data, 0 = command
            self.dc.set_high().map_err(|_| DisplayError::DCError)?;

            send_u8(&mut self.spi, buf).map_err(|_| DisplayError::BusWriteError)?;
            Ok(())
        }
    }

    fn send_u8<T: SpiDevice>(spi: &mut T, words: DataFormat<'_>) -> Result<(), T::Error> {
        match words {
            DataFormat::U8(slice) => spi.write(slice),
            DataFormat::U16(slice) => {
                use byte_slice_cast::*;
                spi.write(slice.as_byte_slice())
            }
            DataFormat::U16LE(slice) => {
                use byte_slice_cast::*;
                for v in slice.as_mut() {
                    *v = v.to_le();
                }
                spi.write(slice.as_byte_slice())
            }
            DataFormat::U16BE(slice) => {
                use byte_slice_cast::*;
                for v in slice.as_mut() {
                    *v = v.to_be();
                }
                spi.write(slice.as_byte_slice())
            }
            DataFormat::U8Iter(iter) => {
                let mut buf = [0; 32];
                let mut i = 0;

                for v in iter.into_iter() {
                    buf[i] = v;
                    i += 1;

                    if i == buf.len() {
                        spi.write(&buf)?;
                        i = 0;
                    }
                }

                if i > 0 {
                    spi.write(&buf[..i])?;
                }

                Ok(())
            }
            DataFormat::U16LEIter(iter) => {
                use byte_slice_cast::*;
                let mut buf = [0; 32];
                let mut i = 0;

                for v in iter.map(u16::to_le) {
                    buf[i] = v;
                    i += 1;

                    if i == buf.len() {
                        spi.write(&buf.as_byte_slice())?;
                        i = 0;
                    }
                }

                if i > 0 {
                    spi.write(&buf[..i].as_byte_slice())?;
                }

                Ok(())
            }
            DataFormat::U16BEIter(iter) => {
                use byte_slice_cast::*;
                let mut buf = [0; 64];
                let mut i = 0;
                let len = buf.len();

                for v in iter.map(u16::to_be) {
                    buf[i] = v;
                    i += 1;

                    if i == len {
                        spi.write(&buf.as_byte_slice())?;
                        i = 0;
                    }
                }

                if i > 0 {
                    spi.write(&buf[..i].as_byte_slice())?;
                }

                Ok(())
            }
            _ => unimplemented!(),
        }
    }
}
