#![no_std]
#![no_main]

pub mod serial_flasher;

use core::cell::RefCell;
use assign_resources::assign_resources;
use cfg_if::cfg_if;

use cortex_m_rt::{entry, exception};
use defmt_rtt as _;
use embassy_boot_rp::*;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, gpio, peripherals, Peripherals, uart};
use embassy_rp::flash::Flash;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::interrupt::typelevel::Interrupt;
use embassy_rp::peripherals::*;
use embassy_rp::uart::{Async, Blocking, BufferedUart, Uart};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage::nor_flash::NorFlash;
use log::info;
use static_cell::StaticCell;
use variegated_log::log_info;
use crate::serial_flasher::main_loop;
use {defmt_rtt as _, panic_probe as _};

static mut rx_buf: [u8; 512] = [0u8; 512];
static mut tx_buf: [u8; 512] = [0u8; 512];

#[cfg(all(feature = "uart0", not(feature = "uart1")))]
type UartType = UART0;

#[cfg(all(feature = "uart1", not(feature = "uart0")))]
type UartType = UART1;

#[cfg(all(feature = "uart0", not(feature = "uart1")))]
bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<UartType>;
});

#[cfg(all(feature = "uart1", not(feature = "uart0")))]
bind_interrupts!(struct Irqs {
    UART1_IRQ => uart::BufferedInterruptHandler<UartType>;
});

cfg_if::cfg_if! {
    if #[cfg(feature = "apec-r0d")] {
        const FLASH_SIZE: usize = 2*1024*1024;

        assign_resources! {
            led: LedResources {
                led1: PIN_2,
            }
            trigger: TriggerResources {
                serial_boot: PIN_22,
            }
            bootloader: BootloaderResources {
                rx_pin: PIN_21,
                tx_pin: PIN_20,
                uart: UART1,
                watchdog: WATCHDOG,
                flash: FLASH,
            }
        }
    } else if #[cfg(feature = "open-lcc-r2a")] {
        compile_error!("Not implemented yet");
    } else {
        compile_error!("Invalid feature flag combination");
    }
}

static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(main_task(spawner, p)).unwrap();
    });
}

#[embassy_executor::task]
async fn main_task(spawner: Spawner, p: Peripherals) -> ! {
    for i in 0..10000 {
        cortex_m::asm::nop();
    }

    let resources = split_resources!(p);

    let start = Instant::now();
    let delay = Duration::from_millis(1000);
    while Instant::now() - start < delay {}
    log_info!("Getting started");

    let status_output = Output::new(resources.led.led1, Level::High);

    let serial_boot_input = gpio::Input::new(resources.trigger.serial_boot, gpio::Pull::Up);

    let config = uart::Config::default();
    let bootloader_uart = unsafe { Uart::new_blocking(resources.bootloader.uart, resources.bootloader.tx_pin, resources.bootloader.rx_pin, config).into_buffered(Irqs, &mut tx_buf, &mut rx_buf) };

    if serial_boot_input.is_high() {
        log_info!("Serial boot is high, booting into serial boot loader");
        boot_flasher::<_, FLASH_SIZE>(bootloader_uart, resources.bootloader.flash, resources.bootloader.watchdog);
    } else {
        log_info!("Booting normally");
        boot_normally(resources.bootloader.flash, resources.bootloader.watchdog, status_output);
    }
}

fn boot_flasher<UartT, const FLASH_SIZE: usize>(uart: BufferedUart<UartT>, flash: FLASH, watchdog: WATCHDOG) -> ! where UartT: uart::Instance {
    main_loop::<_, FLASH_SIZE>(flash, watchdog, uart);

    // Override bootloader watchdog
/*    let mut watchdog = Watchdog::new(watchdog);
    watchdog.start(Duration::from_secs(8));

    let flash = Flash::<_, _, FLASH_SIZE>::new_blocking(flash);
    let flash = Mutex::new(RefCell::new(flash));

    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash);
    let mut aligned = AlignedBuffer([0; 1]);
    let mut updater = BlockingFirmwareUpdater::new(config, &mut aligned.0);
    
    main_loop(flash)*/
/*
    watchdog.feed();
    let mut offset = 0;
    let mut buf: AlignedBuffer<4096> = AlignedBuffer([0; 4096]);
    defmt::info!("preparing update");
    let writer = updater
        .prepare_update()
        .map_err(|e| defmt::warn!("E: {:?}", defmt::Debug2Format(&e)))
        .unwrap();
    defmt::info!("writer created, starting write");
    for chunk in APP_B.chunks(4096) {
        buf.0[..chunk.len()].copy_from_slice(chunk);
        defmt::info!("writing block at offset {}", offset);
        writer.write(offset, &buf.0[..]).unwrap();
        offset += chunk.len() as u32;
    }
    watchdog.feed();
    defmt::info!("firmware written, marking update");
    updater.mark_updated().unwrap();
    Timer::after_secs(2).await;
    defmt::info!("update marked, resetting");*/
    cortex_m::peripheral::SCB::sys_reset();
}

fn boot_normally(flash: FLASH, watchdog: WATCHDOG, mut status_output: Output) -> ! {
    // Uncomment this if you are debugging the bootloader with debugger/RTT attached,
    // as it prevents a hard fault when accessing flash 'too early' after boot.

    for i in 0..10000 {
        cortex_m::asm::nop();
    }


    let flash = WatchdogFlash::<FLASH_SIZE>::start(flash, watchdog, Duration::from_secs(8));
    let flash: Mutex<NoopRawMutex, _> = Mutex::new(RefCell::new(flash));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active_offset = config.active.offset();

    loop {
        status_output.toggle();
        let start = Instant::now();
        let delay = Duration::from_millis(500);
        while Instant::now() - start < delay {}
    }
    let bl: BootLoader = BootLoader::prepare(config);

    unsafe { bl.load(embassy_rp::flash::FLASH_BASE as u32 + active_offset) }
}
