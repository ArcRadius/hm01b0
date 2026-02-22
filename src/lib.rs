//! A Raspberry Pico driver to interface with the HM01B0 camera.
//! Utilizes RP2040/RP2350 PIO to read camera data lines.
//!
//! This driver is built using [`embedded-hal`] traits.

#![no_std]

extern crate embedded_hal;

/// The default address for the HM01B0
pub const DEFAULT_ADDRESS: u8 = 0x24;

/// 320 pixel columns + 4 border columns
pub const H_PIXEL_MAX: usize = 324;
/// 320 pixel rows + 4 border rows
pub const V_PIXEL_MAX: usize = 324;
/// FullFrame camera mode pixel columns
pub const FULLFRAME_H_PIXEL: usize = 320;
/// FullFrame camera mode pixel rows
pub const FULLFRAME_V_PIXEL: usize = 320;
/// QVGA camera mode pixel columns
pub const QVGA_H_PIXEL: usize = 320;
/// QVGA camera mode pixel rows
pub const QVGA_V_PIXEL: usize = 240;
/// QQVGA camera mode pixel columns
pub const QQVGA_H_PIXEL: usize = 160;
/// QQVGA camera mode pixel rows
pub const QQVGA_V_PIXEL: usize = 120;

// Frame rate control constraints

const MIN_LINE_LENGTH_PCK_FULL: u16 = 0x0178;
const MIN_LINE_LENGTH_PCK_QVGA: u16 = 0x0178;
const MIN_LINE_LENGTH_PCK_QQVGA: u16 = 0x00D7;

const MIN_FRAME_LENGTH_LINES_FULL: u16 = 0x0158;
const MIN_FRAME_LENGTH_LINES_QVGA: u16 = 0x0104;
const MIN_FRAME_LENGTH_LINES_QQVGA: u16 = 0x0080;

pub enum Register {
    // Sensor ID

    /// 16-bit sensor Part number High Byte
    ModelIdH = 0x0000,
    /// 16-bit sensor Part number Low Byte
    ModelIdL = 0x0001,
    /// Silicon Revision Number
    SiloconRev = 0x0002,
    /// Frame counter
    FrameCounter = 0x0005,

    // Sensor mode control

    /// Color sensor pixel order
    PixelOrder = 0x0006,
    /// Sensor mode selection
    ModeSelect = 0x0100,
    /// Image orientation
    ImageOrientation = 0x0101,
    /// Software reset
    SwReset = 0x0103,
    /// Group parameter hold
    GroupParameterHold = 0x0104,

    // Sensor exposure gain control

    /// Coarse integration time in lines High Byte
    IntegrationH = 0x0202,
    /// Coarse integration time in lines Low Byte
    IntegrationL = 0x0203,
    /// Analog global gain code
    AnalogGain = 0x0205,
    /// Digital global gain code High Bits
    DigitalGainH = 0x020E,
    /// Digital global gain code Low Bits
    DigitalGainL = 0x020F,

    // Frame timing control

    /// Frame length lines High Byte
    FrameLengthLinesH = 0x0340,
    /// Frame length lines Low Byte
    FrameLengthLinesL = 0x0341,
    /// Line length High Byte
    LineLengthPckH = 0x0342,
    /// Line length Low Byte
    LineLengthPckL = 0x0343,

    // Binning mode control

    /// Read out for x
    ReadoutX = 0x0383,
    /// Read out for y
    ReadoutY = 0x0387,
    /// Binning mode
    BinningMode = 0x0390,

    // Test pattern control

    /// Test pattern mode
    TestPatternMode = 0x0601,

    // Black level control

    /// Black level configuration
    BlcConfiguration = 0x1000,
    /// Black level target
    BlcTarget = 0x1003,
    /// BLI enable
    BliEnable = 0x1006,
    /// Black level 2 target
    Blc2Target = 0x1007,

    // Sensor reserved

    /// DPC control
    DpcControl = 0x1008,
    /// Single hot pixel threshold
    SingleThrHot = 0x100B,
    /// Single cold pixel threshold
    SingleThrCold = 0x100C,

    // VSYNC, HSYNC and pixel shift register

    /// VSYNC, HSYNC and pixel shift enable
    ShiftingEnable = 0x1012,

    // Statistic control and read only

    /// ROI statistic control
    StatisticCtrl = 0x2000,
    /// Motion detection LROI, X start High Byte
    MdLroiXStartH = 0x2011,
    /// Motion detection LROI, X start Low Byte
    MdLroiXStartL = 0x2012,
    /// Motion detection LROI, Y start High Byte
    MdLroiYStartH = 0x2013,
    /// Motion detection LROI, Y start Low Byte
    MdLroiYStartL = 0x2014,
    /// Motion detection LROI, X end High Byte
    MdLroiXSendH = 0x2015,
    /// Motion detection LROI, X end Low Byte
    MdLroiXSendL = 0x2016,
    /// Motion detection LROI, Y end High Byte
    MdLroiYSendH = 0x2017,
    /// Motion detection LROI, Y end Low Byte
    MdLroiYSendL = 0x2018,

    // Automatic exposure gain control

    /// AE Control loop enable
    AeCtrl = 0x2100,
    /// AE target mean
    AeTargetMean = 0x2101,
    /// AE min mean
    AeMinMean = 0x2102,
    /// Converge in threshold
    ConvergeInTh = 0x2103,
    /// Converge out threshold
    ConvergeOutTh = 0x2104,
    /// Maximum INTG High Byte
    MaxIntgH = 0x2105,
    /// Maximum INTG Low Byte
    MaxIntgL = 0x2106,
    /// Minimum INTG
    MinIntg = 0x2107,
    /// Maximum analog gain in full frame mode
    MaxAgainFull = 0x2108,
    /// Maximum analog gain in bin2 mode
    MaxAgainBin2 = 0x2109,
    /// Minimum Again
    MinAGain = 0x210A,
    /// Maximum Dgain
    MaxDGain = 0x210B,
    /// Minimum Dgain
    MinDGain = 0x210C,
    /// Damping factor
    DampingFactor = 0x210D,
    /// Flicker step control
    FsCtrl = 0x210E,
    /// Flicker step 60Hz parameter High Byte
    Fs60hzH = 0x210F,
    /// Flicker step 60Hz parameter Low Byte
    Fs60hzL = 0x2110,
    /// Flicker step 50Hz parameter High Byte
    Fs50hzH = 0x2111,
    /// Flicker step 50Hz parameter Low Byte
    Fs50hzL = 0x2112,
    /// Flicker step hysteresis threshold
    FsHystTh = 0x2113,

    // Motion detection control

    /// Motion detection control
    MdCtrl = 0x2150,
    /// I2c clear
    I2cClear = 0x2153,
    /// wMean difference threshold H
    WMeanDiffThH = 0x2155,
    /// wMean difference threshold M
    WMeanDiffThM = 0x2156,
    /// wMean difference threshold L
    WMeanDiffThL = 0x2157,
    /// MD threshold H
    MdThh = 0x2158,
    /// MD threshold M1
    MdThm1 = 0x2159,
    /// MD threshold M2
    MdThm2 = 0x215a,
    /// MD threshold L
    MdThl = 0x215b,

    // Sensor timing control

    /// QVGA enable
    QvgaWinEn = 0x3010,
    /// Bit mode enable
    SixBitModeEn = 0x3011,
    /// PMU AutoSleep framecount
    PmuProgrammableFramecnt = 0x3020,
    /// Advance VSYNC field from 0 to 20
    AdvanceVsync = 0x3022,
    /// Advance HSYNC field from 0 to 20
    AdvanceHsync = 0x3023,
    /// Applies gain in N+1 frame if integration is not updated in the same CMU frame
    EarlyGain = 0x3035,

    // IO and clock control

    /// Interface bit width control
    BitControl = 0x3059,
    /// Clock divider, gating and LDO control
    OscClkDiv = 0x3060,
    /// LDO power control
    LdoPowerControl = 0x3061,
    /// IO drive strength control
    IoDriveStr = 0x3062,
    /// IO drive strength control
    IoDriveStr2 = 0x3063,
    /// Trigger sync mode enable
    TriggerSyncModeEnable = 0x3064,
    /// Output pin status control
    OutputPinStatusControl = 0x3065,
    /// Master clock control
    MclkControl = 0x3067,
    /// Pixel Clock Polarity
    PclkPolarity = 0x3068,

    // I2C slave registers

    /// I2C ID Selection
    I2CIdSel = 0x3400,
    /// User defined I2C ID
    I2CIdReg = 0x3401,
}

/// Camera Configuration Mode
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum CameraMode {
    /// Full Frame Video Graphics Array
    FullFrame,
    /// Quarter Video Graphics Array
    QVGA,
    /// Quarter Quarter Video Graphics Array
    QQVGA,
}

#[derive(Debug, Copy, Clone)]
/// Color Sensor Pixel Order
pub enum PixelOrder {
    GR = 0x00,
    RG = 0x01,
    BG = 0x02,
    GB = 0x03,
}

#[derive(Debug, Copy, Clone)]
/// HM01B0 Sensor mode selection
pub enum SensorModeSelection {
    /// No output
    Standby = 0x00,
    /// Output frames continuously
    Streaming = 0x01,
    /// Output N frames
    StreamingNFrames = 0x03,
    /// Output frame on trigger
    StreamingHwTrigger = 0x05,
}

#[derive(Debug, Copy, Clone)]
/// HM01B0 image orientation
pub enum ImageOrientation {
    /// Default
    Default = 0x00,
    /// Horizontal Mirror
    HMirror = 0x01,
    /// Vertical Mirror
    VMirror = 0x02,
    /// Horizontal-Vertical Mirror
    HVMirror = 0x03,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 group parameter hold
pub enum GroupParameterHold {
    /// Comsume on capture
    Consume = 0x00,
    /// Hold image
    Hold = 0x01,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 pixel readout
pub enum Readout {
    /// Full readout
    Full = 0x01,
    /// BIN2 timing readout
    Bin2Timing = 0x03,
}

#[derive(Debug, Copy, Clone)]
/// HM01B0 image orientation
pub enum BinningMode {
    /// Default
    Default = 0x00,
    /// Vertical Binning
    VBinning = 0x01,
    /// Horizontal Binning
    HBinning = 0x02,
    /// Horizontal-Vertical Binning
    HVBinning = 0x03,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 Test Pattern Control
pub enum TestPattern {
    /// Test pattern disable
    NoPattern = 0x00,
    /// Color bar pattern
    ColorBar = 0x01,
    /// Walking 1 pattern
    Walking1 = 0x11,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 Black Level Configuration
pub enum BlcConfig {
    /// Black level control disabled
    Disable = 0x00,
    /// Black level control enabled
    Enable = 0x01,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 QVGA windowing enable
pub enum QvgaWinEn {
    /// QVGA windowing disabled
    Disable = 0x00,
    /// QVGA windowing enabled
    Enable = 0x01,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 Bit Control
pub enum BitControl {
    /// Default config
    Default = 0x02,
    /// Serial enabled
    Serial = 0x22,
    /// 4-bit enabled
    FourBit = 0x42,
    /// Serial, 4-bit enabled
    SerialFourBit = 0x62,
}

// Data format control

#[derive(Debug, Copy, Clone)]
// HM01B0 Oscillator sys clock divider
pub enum SysClockDivider {
    Div8 = 0b00,
    Div4 = 0b01,
    Div2 = 0b10,
    Div1 = 0b11,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 Oscillator reg clock divider
pub enum RegClockDivider {
    Div4 = 0b00,
    Div8 = 0b01,
    Div1 = 0b10,
    Div2 = 0b11,
}

#[derive(Debug, Copy, Clone)]
// HM01B0 Enable/Disable Type
pub enum Active {
    Disable = 0x0,
    Enable = 0x1,
}

pub struct HM01B0<I2C: embedded_hal::i2c::I2c> {
    /// I2C communication bus
    com: I2C,
    /// I2C sddress
    addr: u8,
}

impl<I2C: embedded_hal::i2c::I2c> HM01B0<I2C> {
    /// Create HM01B0 driver with specified address
    pub fn new_with_address<E>(i2c: I2C, addr: u8) -> Result<HM01B0<I2C>, E>
    where
        I2C: embedded_hal::i2c::I2c<Error = E>,
    {
        let mut camera = HM01B0 {
            com: i2c,
            addr: DEFAULT_ADDRESS,
        };

        // Set user address
        camera.write_reg8(Register::I2CIdReg, addr);
        // Select it for use
        camera.write_reg8(Register::I2CIdSel, 1);
        // Modify camera to use new address
        camera.addr = addr;

        Ok(camera)
    }

    /// Create HM01B0 driver with default address
    pub fn new<E>(i2c: I2C) -> Result<HM01B0<I2C>, E>
    where
        I2C: embedded_hal::i2c::I2c<Error = E>,
    {
        let camera = HM01B0 {
            com: i2c,
            addr: DEFAULT_ADDRESS,
        };

        Ok(camera)
    }
}

impl<I2C: embedded_hal::i2c::I2c> HM01B0<I2C> {

    /// Set Pixel Order
    pub fn set_pixel_order(&mut self, order: PixelOrder) {
        self.write_reg8(Register::PixelOrder, order as u8);
    }

    /// Set sensor mode selection
    pub fn set_sensor_mode(&mut self, mode: SensorModeSelection) {
        self.write_reg8(Register::ModeSelect, mode as u8);
    }

    /// Set image orientation
    pub fn set_image_orientation(&mut self, orientation: ImageOrientation) {
        self.write_reg8(Register::ImageOrientation, orientation as u8);
    }

    /// Set group parameter hold
    pub fn set_group_parameter_hold(&mut self, hold: GroupParameterHold) {
        self.write_reg8(Register::GroupParameterHold, hold as u8);
    }

    /// Set digital gain
    pub fn set_digital_gain(&mut self, gain: u8) {
        let offset_gain = (gain as u16) << 2;
        self.write_reg16(Register::DigitalGainH, offset_gain);
    }

    /// Get digital gain
    pub fn get_digital_gain(&mut self) -> u8 {
        return ((self.read_reg16(Register::DigitalGainH) >> 2) & 0xFF) as u8;
    }

    /// Set frame length
    pub fn set_frame_length(&mut self, length: u16) {
        self.write_reg16(Register::FrameLengthLinesH, length);
    }

    /// Set line length
    pub fn set_line_length(&mut self, length: u16) {
        self.write_reg16(Register::LineLengthPckH, length);
    }

    /// Set integration
    pub fn set_integration(&mut self, integration: u16) {
        self.write_reg16(Register::IntegrationH, integration);
    }

    /// Set readout X
    pub fn set_readout_x(&mut self, readout: Readout) {
        self.write_reg8(Register::ReadoutX, readout as u8);
    }

    /// Set readout Y
    pub fn set_readout_y(&mut self, readout: Readout) {
        self.write_reg8(Register::ReadoutY, readout as u8);
    }

    /// Set binning mode
    pub fn set_binning_mode(&mut self, mode: BinningMode) {
        self.write_reg8(Register::BinningMode, mode as u8);
    }

    /// Set test pattern
    pub fn set_test_pattern(&mut self, pattern: TestPattern) {
        self.write_reg8(Register::TestPatternMode, pattern as u8);
    }

    /// Set black level configuration
    pub fn set_blc_config(&mut self, config: BlcConfig) {
        self.write_reg8(Register::BlcConfiguration, config as u8);
    }

    /// Set black level target
    pub fn set_blc_target(&mut self, target: u8) {
        self.write_reg8(Register::BlcTarget, target as u8);
        self.write_reg8(Register::Blc2Target, target as u8);
    }

    /// Set black level enable
    pub fn set_bli_enable(&mut self, enable: BlcConfig) {
        self.write_reg8(Register::BliEnable, enable as u8);
    }

    /// Set QVGA enable
    pub fn set_qvga_enable(&mut self, enable: QvgaWinEn) {
        self.write_reg8(Register::QvgaWinEn, enable as u8);
    }

    /// Set Bit Control
    pub fn set_bit_control(&mut self, control: BitControl) {
        self.write_reg8(Register::BitControl, control as u8);
    }

    /// Set clock divider, gating and LDO control
    pub fn set_osc_clk_div(&mut self, sys_div: SysClockDivider, reg_div: RegClockDivider, msb_en: Active, gated_clk_en: Active, dvdd_ldo_en_d: Active) {
        let osc_clk_div: u8 = (sys_div as u8) + ((reg_div as u8) << 2) + ((msb_en as u8) << 4) + ((gated_clk_en as u8) << 5) + ((dvdd_ldo_en_d as u8) << 6);
        self.write_reg8(Register::OscClkDiv, osc_clk_div);
    }

    /// Set sys clock divider
    pub fn set_sys_div(&mut self, sys_div: SysClockDivider) {
        let osc_clk_div: u8 = (self.read_reg8(Register::OscClkDiv) & 0b00000011) + (sys_div as u8);
        self.write_reg8(Register::OscClkDiv, osc_clk_div);
    }

    /// Set reg clock divider
    pub fn set_reg_div(&mut self, reg_div: RegClockDivider) {
        let osc_clk_div: u8 = (self.read_reg8(Register::OscClkDiv) & 0b00001100) + ((reg_div as u8) << 2);
        self.write_reg8(Register::OscClkDiv, osc_clk_div);
    }

    /// Set MSB enable
    pub fn set_msb_en(&mut self, msb_en: Active) {
        let osc_clk_div: u8 = (self.read_reg8(Register::OscClkDiv) & 0b00010000) + ((msb_en as u8) << 4);
        self.write_reg8(Register::OscClkDiv, osc_clk_div);
    }

    /// Set gate clock enable
    pub fn set_gated_clk_en(&mut self, gated_clk_en: Active) {
        let osc_clk_div: u8 = (self.read_reg8(Register::OscClkDiv) & 0b00100000) + ((gated_clk_en as u8) << 5);
        self.write_reg8(Register::OscClkDiv, osc_clk_div);
    }

    /// Set DVDD DLO enable
    pub fn set_dvdd_ldo_en_d(&mut self, dvdd_ldo_en_d: Active) {
        let osc_clk_div: u8 = (self.read_reg8(Register::OscClkDiv) & 0b01000000) + ((dvdd_ldo_en_d as u8) << 6);
        self.write_reg8(Register::OscClkDiv, osc_clk_div);
    }

    /// Perform software reset
    pub fn reset(&mut self) {
        self.write_reg8(Register::SwReset, 1 as u8);
    }

    /// Write single byte to register address
    pub fn write_reg8(&mut self, reg: Register, byte: u8) {
        let mut write = [0_u8; 3];
        write[..2].copy_from_slice(&(reg as u16).to_be_bytes());
        write[2..].copy_from_slice(&[byte]);

        let mut buffer = [0];
        let _ = self
            .com
            .write_read(self.addr, &write, &mut buffer);
    }

    /// Write 2 bytes to register high address
    pub fn write_reg16(&mut self, reg_h: Register, bytes: u16) {
        let mut write = [0_u8; 4];
        write[..2].copy_from_slice(&(reg_h as u16).to_be_bytes());
        write[2..].copy_from_slice(&bytes.to_be_bytes());

        let mut buffer = [0];
        let _ = self
            .com
            .write_read(self.addr, &write, &mut buffer);
    }

    /// Read single byte from register address
    pub fn read_reg8(&mut self, reg: Register) -> u8 {
        let mut data: [u8; 1] = [0];
        let _ = self.com.write_read(self.addr, &(reg as u16).to_be_bytes(), &mut data);
        data[0]
    }

    /// Read two bytes from register high address
    pub fn read_reg16(&mut self, reg: Register) -> u16 {
        let mut data: [u8; 2] = [0, 0];
        let _ = self.com.write_read(self.addr, &(reg as u16).to_be_bytes(), &mut data);
        return u16::from_be_bytes(data)
    }

    pub fn set_camera_mode(&mut self, mode: CameraMode) {
        let model_id = self.read_reg16(Register::ModelIdH);
        if model_id != 0x01B0 {
            panic!("Invalid model id!");
        }

        if CameraMode::FullFrame == mode {
            self.set_readout_x(Readout::Full);
            self.set_readout_y(Readout::Full);
            self.set_binning_mode(BinningMode::Default);
            self.set_qvga_enable(QvgaWinEn::Disable);
            self.set_frame_length(MIN_FRAME_LENGTH_LINES_FULL);
            self.set_line_length(MIN_LINE_LENGTH_PCK_FULL);
            self.set_integration(MIN_LINE_LENGTH_PCK_FULL / 2);
        } else if CameraMode::QVGA == mode {
            self.set_readout_x(Readout::Full);
            self.set_readout_y(Readout::Full);
            self.set_binning_mode(BinningMode::Default);
            self.set_qvga_enable(QvgaWinEn::Enable);
            self.set_frame_length(MIN_FRAME_LENGTH_LINES_QVGA);
            self.set_line_length(MIN_LINE_LENGTH_PCK_QVGA);
            self.set_integration(MIN_LINE_LENGTH_PCK_QVGA / 2);
        } else {  // CameraMode::QQVGA
            self.set_readout_x(Readout::Bin2Timing);
            self.set_readout_y(Readout::Bin2Timing);
            self.set_binning_mode(BinningMode::HVBinning);
            self.set_qvga_enable(QvgaWinEn::Enable);
            self.set_frame_length(MIN_FRAME_LENGTH_LINES_QQVGA);
            self.set_line_length(MIN_LINE_LENGTH_PCK_QQVGA);
            self.set_integration(MIN_LINE_LENGTH_PCK_QQVGA / 2);
        }

        self.set_osc_clk_div(SysClockDivider::Div8, RegClockDivider::Div1, Active::Disable, Active::Disable, Active::Disable);
        self.set_group_parameter_hold(GroupParameterHold::Hold);
    }
}
