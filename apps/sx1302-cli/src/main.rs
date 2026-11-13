use clap::{Parser, Subcommand};
use sensecap_board::{inspect_carrier, reset_carrier, Carrier};
use sx1302_core::Region;
use sx1302_firmware::{load_bin_or_var, AGC_FW, ARB_FW};
use sx1302_hal::{ChannelPlan, Concentrator, TxRequest};
use sx1302_transport::SpiConfig;
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "sx1302-cli", about = "Native Rust SX1302 HAL diagnostics")]
struct Cli {
    /// Carrier reset wiring: sensecap-m1, wm1302-pi-hat, rak2287, or rak5146.
    #[arg(long, global = true, default_value = "sensecap-m1")]
    board: String,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Inspect host, SPI/I2C/GPIO, and unverified SenseCAP pins (JSON).
    Board {
        #[command(subcommand)]
        inner: BoardCmd,
    },
    /// Non-destructive SPI probe of COMMON_VERSION (no GPIO reset).
    Detect {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
        #[arg(long, default_value_t = 2_000_000)]
        speed: u32,
    },
    /// Internal SX1250 reset + RX clock + OTP EUI. No Pi GPIO, no TX.
    Bringup {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
        #[arg(long, default_value_t = 2_000_000)]
        speed: u32,
    },
    /// Reset GPIOs for the selected --board carrier.
    Gpio {
        #[command(subcommand)]
        inner: GpioCmd,
    },
    /// Load AGC/ARB firmware blobs (8192 bytes, Semtech SHA 4b42025).
    LoadFw {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
        #[arg(long)]
        agc: PathBuf,
        #[arg(long)]
        arb: PathBuf,
    },
    /// Radio bring-up + SX1250 SET_TX + PA_EN. `--region` is required (AS923-1 = 920–925 MHz).
    Tx {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
        #[arg(long, default_value_t = 2_000_000)]
        speed: u32,
        #[arg(long, help = "EU868, US915, or AS923-1 (920–925 MHz, default TX 923.2 MHz)")]
        region: String,
        #[arg(long)]
        freq: Option<u32>,
    },
    /// GPIO reset + radio + AGC/ARB. AS923-1, US915 or FCC-800.
    Start {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
        #[arg(long, default_value = "AS923-1")]
        region: String,
        /// 0-based sub-band: US915 0–7 (default 1 = FSB2), FCC-800 0–4 (default 0).
        #[arg(long)]
        subband: Option<u8>,
    },
    /// Poll RX FIFO (raw bytes). Run after `start`.
    Rx {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
        /// Keep polling this many seconds (0 = once).
        #[arg(long, default_value_t = 0)]
        seconds: u64,
    },
    /// LoRa packet via TX_TOP_A. Region AS923-1, default 923.2 MHz SF7.
    TxPkt {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
        #[arg(long, default_value = "AS923-1")]
        region: String,
        #[arg(long)]
        freq: Option<u32>,
        #[arg(long, default_value = "7")]
        sf: u8,
        #[arg(long, default_value = "00")]
        payload: String,
    },
    /// ATECC608 read-only probe on I2C-1 0x60.
    Atecc,
    /// SX126x GPIO-CS identify (no TX).
    Sx126,
    /// SX126 LBT RSSI sample (SPI_NO_CS + GPIO6, no TX).
    Lbt {
        #[arg(long, default_value_t = 923_200_000)]
        freq: u32,
        #[arg(long, default_value_t = -80)]
        rssi_target: i16,
    },
    /// Read SX1302 PPS + freerun counters (legacy 1 µs).
    Timestamp {
        #[arg(long, default_value = "/dev/spidev0.0")]
        spi: String,
    },
    /// User LED GPIO22 (Clockwork Bird M1 map).
    Led {
        #[arg(long, conflicts_with = "off")]
        on: bool,
        #[arg(long)]
        off: bool,
    },
}

#[derive(Subcommand)]
enum BoardCmd {
    Inspect,
}

#[derive(Subcommand)]
enum GpioCmd {
    Reset,
}

fn main() {
    let cli = Cli::parse();
    let carrier = Carrier::parse(&cli.board).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1);
    });
    match cli.cmd {
        Cmd::Board { inner: BoardCmd::Inspect } => {
            let report = inspect_carrier(carrier);
            println!("{}", serde_json::to_string_pretty(&report).unwrap());
        }
        Cmd::Detect { spi, speed } => run_hal(spi, speed, false),
        Cmd::Bringup { spi, speed } => run_hal(spi, speed, true),
        Cmd::Gpio { inner: GpioCmd::Reset } => match reset_carrier(carrier) {
            Ok(notes) => {
                for n in notes {
                    println!("{n}");
                }
            }
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        },
        Cmd::LoadFw { spi, agc, arb } => {
            let cfg = SpiConfig {
                path: spi,
                ..Default::default()
            };
            let r = (|| {
                let c = Concentrator::open(cfg)?;
                let agc = load_bin_or_var(&agc)?;
                let arb = load_bin_or_var(&arb)?;
                let _ = (AGC_FW, ARB_FW);
                c.load_mcu_firmware(&agc, &arb)
            })();
            match r {
                Ok(notes) => {
                    for n in notes {
                        println!("{n}");
                    }
                }
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
        }
        Cmd::Tx {
            spi,
            speed,
            region,
            freq,
        } => {
            let region = match Region::parse(&region) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            };
            let cfg = SpiConfig {
                path: spi,
                speed_hz: speed,
                ..Default::default()
            };
            match Concentrator::open(cfg).and_then(|c| c.start_tx(region, freq)) {
                Ok(r) => println!("{}", serde_json::to_string_pretty(&r).unwrap()),
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
        }
        Cmd::Start { spi, region, subband } => {
            let region = match Region::parse(&region) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            };
            let plan = match ChannelPlan::preset(region, subband) {
                Ok(p) => p,
                Err(e) => {
                    eprintln!("{e} (start programs AS923-1, US915 or FCC-800)");
                    std::process::exit(1);
                }
            };
            if let Err(e) = reset_carrier(carrier) {
                eprintln!("{e}");
                std::process::exit(1);
            }
            let cfg = SpiConfig {
                path: spi,
                ..Default::default()
            };
            match Concentrator::open(cfg).and_then(|c| c.start(&plan)) {
                Ok(notes) => {
                    for n in notes {
                        println!("{n}");
                    }
                }
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
        }
        Cmd::Rx { spi, seconds } => {
            let cfg = SpiConfig {
                path: spi,
                ..Default::default()
            };
            let c = match Concentrator::open(cfg) {
                Ok(c) => c,
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            };
            let t0 = std::time::Instant::now();
            let mut last: Option<sx1302_hal::RxDump> = None;
            let mut dec_min = 255u8;
            let mut dec_max = 0u8;
            loop {
                match c.rx_poll() {
                    Ok(mut d) => {
                        if let Some(v) = d.rssi_dec_a {
                            dec_min = dec_min.min(v);
                            dec_max = dec_max.max(v);
                            d.rssi_dec_min = Some(dec_min);
                            d.rssi_dec_max = Some(dec_max);
                        }
                        if d.nb_bytes > 0 || !d.packets.is_empty() || seconds == 0 {
                            println!("{}", serde_json::to_string_pretty(&d).unwrap());
                        }
                        if seconds > 0 && d.packets.iter().any(|p| p.size > 0) {
                            break;
                        }
                        last = Some(d);
                    }
                    Err(e) => {
                        eprintln!("{e}");
                        std::process::exit(1);
                    }
                }
                if seconds == 0 || t0.elapsed().as_secs() >= seconds {
                    if seconds > 0 {
                        if let Some(d) = last {
                            println!("{}", serde_json::to_string_pretty(&d).unwrap());
                        }
                    }
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
        }
        Cmd::TxPkt {
            spi,
            region,
            freq,
            sf,
            payload,
        } => {
            let region = match Region::parse(&region) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            };
            let bytes = match hex_bytes(&payload) {
                Ok(b) => b,
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            };
            let mut req = match TxRequest::as923_1_sf7(bytes) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            };
            req.region = region;
            req.sf = sf;
            if let Some(f) = freq {
                req.freq_hz = f;
            }
            let cfg = SpiConfig {
                path: spi,
                ..Default::default()
            };
            match Concentrator::open(cfg).and_then(|c| c.send_lora(&req)) {
                Ok(s) => println!("{s}"),
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
        }
        Cmd::Atecc => match sensecap_secure_element::inspect_readonly() {
            Ok(r) => println!("{}", serde_json::to_string_pretty(&r).unwrap()),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        },
        Cmd::Sx126 => match sx126x_radio::identify() {
            Ok(r) => println!("{}", serde_json::to_string_pretty(&r).unwrap()),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        },
        Cmd::Lbt { freq, rssi_target } => match sx126x_radio::lbt_rssi(freq, rssi_target) {
            Ok(r) => println!("{}", serde_json::to_string_pretty(&r).unwrap()),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1);
            }
        },
        Cmd::Timestamp { spi } => {
            let cfg = SpiConfig {
                path: spi,
                ..Default::default()
            };
            match Concentrator::open(cfg).and_then(|c| c.read_timestamp()) {
                Ok(r) => println!("{}", serde_json::to_string_pretty(&r).unwrap()),
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
        }
        Cmd::Led { on, off } => {
            let want_on = on || !off;
            if !on && !off {
                eprintln!("sx1302-cli led --on | --off");
                std::process::exit(2);
            }
            match sensecap_led::set(want_on) {
                Ok(r) => println!("{}", serde_json::to_string_pretty(&r).unwrap()),
                Err(e) => {
                    eprintln!("{e}");
                    std::process::exit(1);
                }
            }
        }
    }
}

fn hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if s.len() % 2 != 0 {
        return Err("hex payload odd length".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect()
}

fn run_hal(spi: String, speed: u32, bringup: bool) {
    let cfg = SpiConfig {
        path: spi,
        speed_hz: speed,
        ..Default::default()
    };
    let r = Concentrator::open(cfg).and_then(|c| {
        if bringup {
            c.bringup()
        } else {
            c.detect()
        }
    });
    match r {
        Ok(r) => println!("{}", serde_json::to_string_pretty(&r).unwrap()),
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    }
}
