#![no_std]
#![no_main]


use cyw43::Control;
use cyw43_pio::PioSpi;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::{join,join3};
use embassy_rp::{bind_interrupts};
use embassy_rp::gpio::{Level, Input, Output, AnyPin, Pin, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{Pio};
use embassy_rp::usb::{Driver, Instance};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb_logger;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Ticker, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use log::log;
use rp2040_boot2;

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 16384] =
    *include_bytes!("../firmware/serial-bootloader.bin");

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

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

#[derive(Clone, Copy)]
enum JogBall {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    CLICK,
}
static CHANNEL_JOGBALL: Channel<ThreadModeRawMutex, JogBall, 64> = Channel::new();

type LedType = Mutex<ThreadModeRawMutex, Option<Output<'static>>>;
static WHITE_LED: LedType = Mutex::new(None);
static GREEN_LED: LedType = Mutex::new(None);
static RED_LED: LedType = Mutex::new(None);
static BLUE_LED: LedType = Mutex::new(None);
static BACKLIGHT: LedType = Mutex::new(None);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello there!");

    let p = embassy_rp::init(Default::default());

    // Configure button/jogball inputs
    let jogball_up = Input::new(p.PIN_7, Pull::None);
    let jogball_down = Input::new(p.PIN_8, Pull::None);
    let jogball_left = Input::new(p.PIN_3, Pull::None);
    let jogball_right = Input::new(p.PIN_6, Pull::None); // verified
    let jogball_click = Input::new(p.PIN_2, Pull::Up);
    {
        // inner scope is so that once the mutex is written to, the MutexGuard is dropped, thus the Mutex is released
        *(JOGBALL_UP.lock().await) = Some(jogball_up);
        *(JOGBALL_DOWN.lock().await) = Some(jogball_down);
        *(JOGBALL_LEFT.lock().await) = Some(jogball_left);
        *(JOGBALL_RIGHT.lock().await) = Some(jogball_right);
        *(JOGBALL_CLICK.lock().await) = Some(jogball_click);
    }

    // Configure various directly driven LEDs
    let white_led = Output::new(AnyPin::from(p.PIN_9), Level::Low);
    let green_led = Output::new(AnyPin::from(p.PIN_10), Level::Low);
    let red_led = Output::new(AnyPin::from(p.PIN_12), Level::Low);
    let blue_led = Output::new(AnyPin::from(p.PIN_11), Level::Low);
    let backlight = Output::new(AnyPin::from(p.PIN_13), Level::Low);
    {
        // inner scope is so that once the mutex is written to, the MutexGuard is dropped, thus the Mutex is released
        *(WHITE_LED.lock().await) = Some(white_led);
        *(GREEN_LED.lock().await) = Some(green_led);
        *(RED_LED.lock().await) = Some(red_led);
        *(BLUE_LED.lock().await) = Some(blue_led);
        *(BACKLIGHT.lock().await) = Some(backlight);
    }


    let jogball_fut = async {
        log::info!("Awaiting Jogball changes");
        loop{
            match CHANNEL_JOGBALL.receive().await {
                JogBall::CLICK => log::info!("Jogball CLICK"),
                JogBall::UP => log::info!("Jogball Up"),
                JogBall::DOWN => log::info!("Jogball Down"),
                JogBall::LEFT => log::info!("Jogball Left"),
                JogBall::RIGHT => log::info!("Jogball Right"),
            }
        }
    };

    let fw = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

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

    let blinken_lights_future = async {
        let mut state_of_led = false;
        loop {
            Timer::after(Duration::from_millis(1000)).await;
            state_of_led = !state_of_led;
            control.gpio_set(0, state_of_led).await;
        }
    };

    // Led Tasks
    unwrap!(spawner.spawn(toggle_led(&WHITE_LED,Duration::from_hz(10))));
    unwrap!(spawner.spawn(toggle_led(&GREEN_LED,Duration::from_hz(9))));
    unwrap!(spawner.spawn(toggle_led(&RED_LED,Duration::from_hz(8))));
    unwrap!(spawner.spawn(toggle_led(&BLUE_LED,Duration::from_hz(11))));
    unwrap!(spawner.spawn(toggle_led(&BACKLIGHT,Duration::from_hz(15))));

    // Jogball tasks
    unwrap!(spawner.spawn(jogball(&JOGBALL_UP,    CHANNEL_JOGBALL.sender(), JogBall::UP)));
    unwrap!(spawner.spawn(jogball(&JOGBALL_DOWN,  CHANNEL_JOGBALL.sender(), JogBall::DOWN)));
    unwrap!(spawner.spawn(jogball(&JOGBALL_LEFT,  CHANNEL_JOGBALL.sender(), JogBall::LEFT)));
    unwrap!(spawner.spawn(jogball(&JOGBALL_RIGHT, CHANNEL_JOGBALL.sender(), JogBall::RIGHT)));
    unwrap!(spawner.spawn(click(&JOGBALL_CLICK,   CHANNEL_JOGBALL.sender(), JogBall::CLICK)));

    log::info!("Starting up MultiChip on RP2040!");
    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join3(
        jogball_fut,
        join(blinken_lights_future,usb_fut),
        join(echo_fut, log_fut)
    ).await;
}

#[embassy_executor::task(pool_size = 4)]
async fn jogball(jogball_btn: &'static JogballButtonType, control: Sender<'static, ThreadModeRawMutex, JogBall,64>, direction: JogBall){

    loop {
        let mut jogball_btn_unlocked  = jogball_btn.lock().await;
        jogball_btn_unlocked.as_mut().unwrap().wait_for_any_edge().await;
        control.send(direction).await;
    }
}
#[embassy_executor::task]
async fn click(btn: &'static ButtonType, control: Sender<'static, ThreadModeRawMutex, JogBall,64>, direction: JogBall ){

    loop {
        let mut btn_unlocked  = btn.lock().await;
        btn_unlocked.as_mut().unwrap().wait_for_falling_edge().await;
        control.send(direction).await;
    }
}

#[embassy_executor::task(pool_size = 5)]
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

