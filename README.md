# `HM01B0`

> no_std driver for Arducam HM01B0 QVGA Camera Module for Raspberry Pico.

## What works

- Camera Configuration

## TODO

- Add embassy-rp2040 PIO readout methods
- Add embassy-rp235x PIO readout methods

## Supported chips

* `HM01B0`

## Basic usage

```
[dependencies.hm01b0]
version = "<version>"
```

Use embedded-hal implementation to create I2C hm01b0 and perform PIO readout.

```rust
use embassy_rp::gpio::Level;
use embassy_rp::pio::Direction as PioDirection;

embassy_rp::bind_interrupts!(struct IrqsPio {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut config = embassy_rp::i2c::Config::default();
    config.frequency = 400_000;  // 100 KHz - 400 KHz for HM01B0
    let mut i2c = embassy_rp::i2c::I2c::new_blocking(p.I2C0, scl, sda, config);
    let mut cam = hm01b0::HM01B0::new(i2c).unwrap();

    cam.reset();

    // Configure PIO serial readout
    let pio = p.PIO0;
    let embassy_rp::pio::Pio {
        mut common,
        sm0: mut sm,
        ..
    } = embassy_rp::pio::Pio::new(pio, IrqsPio);

    let vsync_pin = p.PIN_16;
    let hsync_pin = p.PIN_15;
    let pclck_pin = p.PIN_14;
    let d0_pin = p.PIN_6;

    let hsync_pin_id = hsync_pin.pin();
    let vsync_pin_id = vsync_pin.pin();
    let pclck_pin_id = pclck_pin.pin();

    let mut hsync = common.make_pio_pin(hsync_pin);
    let mut vsync = common.make_pio_pin(vsync_pin);
    let mut pclck = common.make_pio_pin(pclck_pin);
    let mut d0    = common.make_pio_pin(d0_pin);

    hsync.set_pull(embassy_rp::gpio::Pull::Up);
    vsync.set_pull(embassy_rp::gpio::Pull::Up);
    pclck.set_pull(embassy_rp::gpio::Pull::Up);
    d0.set_pull(embassy_rp::gpio::Pull::Up);

    sm.set_pins(Level::High, &[&d0, &hsync, &vsync, &pclck]);
    sm.set_pin_dirs(PioDirection::In, &[&d0, &hsync, &vsync, &pclck]);

    let num_border_px = 4;
    let num_pclk_per_px = 8;
    let data_bits = 1;

    let mut a: embassy_rp::pio::program::Assembler<18> = embassy_rp::pio::program::Assembler::new();

    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut border_pixel_x = a.label();
    let mut border_pixel_y = a.label();
    let mut readout = a.label();

    a.pull(false, true);  // Move the TX FIFO to the OSR and stall until data is available
    a.wait(0, embassy_rp::pio::program::WaitSource::GPIO, vsync_pin_id, false);
    a.wait(1, embassy_rp::pio::program::WaitSource::GPIO, vsync_pin_id, false);  // VSYNC rising edge - Ready to transmit image, HSYNC to begin toggling
    a.set(embassy_rp::pio::program::SetDestination::Y, (num_border_px / 2) - 1);
    a.bind(&mut border_pixel_y);
    a.wait(1, embassy_rp::pio::program::WaitSource::GPIO, hsync_pin_id, false);
    a.wait(0, embassy_rp::pio::program::WaitSource::GPIO, hsync_pin_id, false);  // HSYNC falling edge - Done transmitting last pixel (skipped Y border pixel)
    a.jmp(embassy_rp::pio::program::JmpCondition::YDecNonZero, &mut border_pixel_y);
    a.bind(&mut wrap_target);
    a.mov(embassy_rp::pio::program::MovDestination::X, embassy_rp::pio::program::MovOperation::None, embassy_rp::pio::program::MovSource::OSR);  // Clear and reset the OSR
    a.wait(1, embassy_rp::pio::program::WaitSource::GPIO, hsync_pin_id, false);  // HSYNC rising edge - Ready to transmit pixel, PCLCK to begin toggling
    a.set(embassy_rp::pio::program::SetDestination::Y, (num_border_px / 2) * num_pclk_per_px - 1);
    a.bind(&mut border_pixel_x);
    a.wait(1, embassy_rp::pio::program::WaitSource::GPIO, pclck_pin_id, false);
    a.wait(0, embassy_rp::pio::program::WaitSource::GPIO, pclck_pin_id, false);  // PCLCK falling edge - Pixel ready to read (Register::PclkPolarity)
    a.jmp(embassy_rp::pio::program::JmpCondition::YDecNonZero, &mut border_pixel_x);
    a.bind(&mut readout);
    a.wait(1, embassy_rp::pio::program::WaitSource::GPIO, pclck_pin_id, false);
    a.r#in(embassy_rp::pio::program::InSource::PINS, data_bits);  // Read data pins (D0) into state machine input-shift-register, then auto-shifts to RX FIFO when full (to DMA)
    a.wait(0, embassy_rp::pio::program::WaitSource::GPIO, pclck_pin_id, false);
    a.jmp(embassy_rp::pio::program::JmpCondition::XDecNonZero, &mut readout);
    a.wait(0, embassy_rp::pio::program::WaitSource::GPIO, hsync_pin_id, false);
    a.bind(&mut wrap_source);

    let prg = a.assemble_with_wrap(wrap_source, wrap_target);
    let prg = common.load_program(&prg);

    let mut cfg = embassy_rp::pio::Config::default();

    cfg.set_in_pins(&[&d0]);

    cfg.shift_in = embassy_rp::pio::ShiftConfig {
        auto_fill: true,
        threshold: 8,
        direction: embassy_rp::pio::ShiftDirection::Left,
    };

    cfg.use_program(&prg, &[]);

    cfg.fifo_join = embassy_rp::pio::FifoJoin::Duplex;
    sm.set_config(&cfg);

    cam.set_bit_control(hm01b0::BitControl::Serial);
    cam.set_msb_en(hm01b0::Active::Enable);

    // Configure Full Frame image read buffer
    const IMAGE_BUFFER_MAX_SIZE: usize = hm01b0::H_PIXEL_MAX * hm01b0::V_PIXEL_MAX;
    static mut FULL_IMAGE_BUFFER: [u8; IMAGE_BUFFER_MAX_SIZE] = [0u8; IMAGE_BUFFER_MAX_SIZE];

    let camera_mode = hm01b0::CameraMode::FullFrame;
    cam.set_camera_mode(camera_mode);
    let image_size: usize = hm01b0::FULLFRAME_H_PIXEL * hm01b0::FULLFRAME_V_PIXEL;
    let mut image_buffer = unsafe { &mut FULL_IMAGE_BUFFER[..image_size] };

    // Prepare PIO state machine to skip border pixels
    sm.clear_fifos();
    let num_border_rows: u32 = (hm01b0::FULLFRAME_H_PIXEL as u32 * num_pclk_per_px as u32) - 1;
    sm.tx().push(num_border_rows);

    // Read image
    cam.set_sensor_mode(hm01b0::SensorModeSelection::Streaming);
    sm.set_enable(true);
    let mut dma_0 = p.DMA_CH0.into_ref();
    sm.rx().dma_pull(dma_0.reborrow(), &mut image_buffer, false).await;
    sm.set_enable(false);
    cam.set_sensor_mode(hm01b0::SensorModeSelection::Standby);
}
```

## License

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
