use clap::Parser;
use sx1302_hal::Concentrator;
use sx1302_transport::SpiConfig;

#[derive(Parser)]
struct Args {
    #[arg(long, default_value = "/dev/spidev0.0")]
    spi: String,
    #[arg(long, default_value_t = 1000)]
    iters: u32,
}

fn main() {
    let args = Args::parse();
    let c = match Concentrator::open(SpiConfig {
        path: args.spi,
        ..Default::default()
    }) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    };
    let port = c.bus().sx1302();
    for _ in 0..args.iters {
        let _ = port.probe_version();
    }
    match c.bus().metrics() {
        Ok(m) => println!("{}", serde_json::to_string_pretty(&m).unwrap()),
        Err(e) => eprintln!("{e}"),
    }
}
