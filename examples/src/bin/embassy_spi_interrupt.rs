//! Embassy SPI
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! Connect MISO and MOSI pins to see the outgoing data is read as incoming
//! data.
//!
//! The following wiring is assumed:
//! SCLK => GPIO0
//! MISO => GPIO2
//! MOSI => GPIO4
//! CS   => GPIO5

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: embassy embassy-generic-timers

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    dma::*,
    dma_buffers,
    gpio::{Io, Level, Output, Input, Pull},
    prelude::*,
    spi::{master::Spi, SpiMode},
    timer::timg::TimerGroup,
};
use esp_println::println;

const SPI_BUFFER_SIZE_BYTE: usize   = 128;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = Output::new(io.pins.gpio2, Level::Low);
    let miso = Input::new(io.pins.gpio4, Pull::None);
    let mosi = Output::new(io.pins.gpio12, Level::Low);
    let cs = Output::new(io.pins.gpio13, Level::High);
    let mut ble_interrupt_pin: Input<'static> = Input::new(io.pins.gpio18, Pull::Up);
    let mut tm_interrupt_pin: Output<'static> = Output::new(io.pins.gpio19, Level::High);
    let dma = Dma::new(peripherals.DMA);

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2"))] {
            let dma_channel = dma.spi2channel;
        } else {
            let dma_channel = dma.channel0;
        }
    }

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(SPI_BUFFER_SIZE_BYTE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let mut spi = Spi::new(peripherals.SPI2, 8000.kHz(), SpiMode::Mode0)
        .with_pins(sclk, mosi, miso, cs)
        .with_dma(dma_channel.configure_for_async(false, DmaPriority::Priority0))
        .with_buffers(dma_rx_buf, dma_tx_buf);

    let mut send_buffer = [0u8; SPI_BUFFER_SIZE_BYTE];
    for i in 0..send_buffer.len() {
        send_buffer[i] = i as u8;
    }

    let mut cnt: u8 = 0u8;
    loop {
        cnt = (cnt + 1) % (u8::MAX);
        send_buffer.fill(cnt);
        let mut buffer = [0u8; SPI_BUFFER_SIZE_BYTE];
        esp_println::println!("reading bytes");
        ble_interrupt_pin.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(10)).await;
        embedded_hal_async::spi::SpiBus::transfer(&mut spi, &mut buffer, &send_buffer)
            .await
            .unwrap();

        if ble_interrupt_pin.is_low() {
            ble_interrupt_pin.wait_for_high().await;
        }

        tm_interrupt_pin.set_low();
        Timer::after(Duration::from_millis(1000)).await;
        tm_interrupt_pin.set_high();

        esp_println::println!("Bytes received: {:?}", buffer);
    }
}
