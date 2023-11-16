//! embassy serial
//!
//! This is an example of running the embassy executor and asynchronously
//! writing to and reading from uart

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use esp32c6_hal::{
    clock::ClockControl,
    embassy,
    interrupt,
    peripherals::{Interrupt, Peripherals, UART0},
    prelude::*,
    Uart,
};
use esp_backtrace as _;
use esp_hal_common::uart::{config::AtCmdConfig, UartRx, UartTx};
use static_cell::make_static;
use heapless::String;

// rx_fifo_full_threshold
const READ_BUF_SIZE: usize = 64;
// EOT (ENTER)
const AT_CMD: u8 = 0x0a;

#[embassy_executor::task]
async fn writer(mut tx: UartTx<'static, UART0>, signal: &'static Signal<NoopRawMutex, usize>) {
    use core::fmt::Write;
    embedded_io_async::Write::write(
        &mut tx,
        b"\x1b[32mESP32C6 >\x1b[0m ",
    )
    .await
    .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(mut rx: UartRx<'static, UART0>, signal: &'static Signal<NoopRawMutex, usize>) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        let mut cmd: String<255> = String::from("");
        match r {
            Ok(len) => {

                for i in 0..len {
                    if rbuf[i] == 0x8 {
                        cmd.pop();
                    } else if rbuf[i] != b'\n' && rbuf[i] != 0x0 {
                        cmd.push(rbuf[i] as char);
                    }
                }
                signal.signal(len);
            }
            Err(e) => esp_println::println!("RX Error: {:?}", e),
        }

        match cmd.as_str() {
            "help" => {
                esp_println::print!("\nCommands:\n    command1\n    command2\n    command3\n    command4\n    command5\n");
            }
            "command1" => {
                esp_println::print!("run command1\n");
            }
            "command2" => {
                esp_println::print!("run command2\n");
            }
            "command3" => {
                esp_println::print!("run command3\n");
            }
            "command4" => {
                esp_println::print!("run command4\n");
            }
            "command5" => {
                esp_println::print!("run command5\n");
            }
            _ => {
                if !cmd.is_empty() {
                    esp_println::print!("Unknown command\n");
                }
            }
        }
        cmd.clear();
        esp_println::print!("\x1b[32mESP32C6 >\x1b[0m ");
    }
}

#[main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    #[cfg(feature = "embassy-time-systick")]
    embassy::init(
        &clocks,
        esp32c6_hal::systimer::SystemTimer::new(peripherals.SYSTIMER),
    );

    #[cfg(feature = "embassy-time-timg0")]
    {
        let timer_group0 = esp32c6_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
        embassy::init(&clocks, timer_group0.timer0);
    }

    let mut uart0 = Uart::new(peripherals.UART0, &clocks);
    uart0.set_at_cmd(AtCmdConfig::new(None, None, None, AT_CMD, None));
    uart0
        .set_rx_fifo_full_threshold(READ_BUF_SIZE as u16)
        .unwrap();
    let (tx, rx) = uart0.split();

    interrupt::enable(Interrupt::UART0, interrupt::Priority::Priority1).unwrap();

    let signal = &*make_static!(Signal::new());

    spawner.spawn(reader(rx, &signal)).ok();
    spawner.spawn(writer(tx, &signal)).ok();
}
