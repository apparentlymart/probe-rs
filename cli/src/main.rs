use probe::debug_probe::MasterProbe;
use memory::MI;
use coresight::ap_access::APAccess;
use coresight::access_ports::{
    generic_ap::GenericAP,
};
use coresight::ap_access::access_port_is_valid;
use coresight::access_ports::AccessPortError;
use std::time::Instant;

use probe::debug_probe::{
    DebugProbe,
    DebugProbeError,
    DebugProbeType,
    DebugProbeInfo,
    Port,
};

use structopt::StructOpt;

fn parse_hex(src: &str) -> Result<u32, std::num::ParseIntError> {
    u32::from_str_radix(src, 16)
}

#[derive(StructOpt)]
#[structopt(
    name = "ST-Link CLI",
    about = "Get info about the connected ST-Links",
    author = "Noah Hüsser <yatekii@yatekii.ch>"
)]
enum CLI {
    /// List all connected ST-Links
    #[structopt(name = "list")]
    List {},
    /// Gets infos about the selected ST-Link
    #[structopt(name = "info")]
    Info {
        /// The number associated with the ST-Link to use
        n: usize,
    },
    /// Resets the target attached to the selected ST-Link
    #[structopt(name = "reset")]
    Reset {
        /// The number associated with the ST-Link to use
        n: usize,
        /// Whether the reset pin should be asserted or deasserted. If left open, just pulse it
        assert: Option<bool>,
    },
    /// Dump memory from attached target
    #[structopt(name = "dump")]
    Dump {
        /// The number associated with the ST-Link to use
        n: usize,
        /// The address of the memory to dump from the target (in hexadecimal without 0x prefix)
        #[structopt(parse(try_from_str = "parse_hex"))]
        loc: u32,
        /// The amount of memory (in words) to dump
        words: u32,
    },
    /// Download memory to attached target
    // #[structopt(name = "download")]
    // Download {
    //     /// The number associated with the ST-Link to use
    //     n: usize,
    //     /// The address of the memory to download to the target (in hexadecimal without 0x prefix)
    //     #[structopt(parse(try_from_str = "parse_hex"))]
    //     loc: u32,
    //     /// The the word to write to memory
    //     #[structopt(parse(try_from_str = "parse_hex"))]
    //     word: u32,
    // },
    #[structopt(name = "trace")]
    Trace {
        /// The number associated with the ST-Link to use
        n: usize,
        /// The address of the memory to dump from the target (in hexadecimal without 0x prefix)
        #[structopt(parse(try_from_str = "parse_hex"))]
        loc: u32,
    },
}

fn main() {
    // Initialize the logging backend.
    pretty_env_logger::init();

    let matches = CLI::from_args();

    match matches {
        CLI::List {} => list_connected_devices(),
        CLI::Info { n } => show_info_of_device(n).unwrap(),
        CLI::Reset { n, assert } => reset_target_of_device(n, assert).unwrap(),
        CLI::Dump { n, loc, words } => dump_memory(n, loc, words).unwrap(),
        //CLI::Download { n, loc, word } => download(n, loc, word).unwrap(),
        CLI::Trace { n, loc } => trace_u32_on_target(n, loc).unwrap(),
    }
}

fn list_connected_devices() {
    let links = get_connected_devices();

    if links.len() > 0 {
        println!("The following devices were found:");
        links
            .iter()
            .enumerate()
            .for_each(|(num, link)| println!( "[{}]: {:?}", num, link));
    } else {
        println!("No devices were found.");
    }
}

#[derive(Debug)]
enum Error {
    DebugProbe(DebugProbeError),
    AccessPort(AccessPortError),
    //Custom(&'static str),
    StdIO(std::io::Error),
}

impl From<AccessPortError> for Error {
    fn from(error: AccessPortError) -> Self {
        Error::AccessPort(error)
    }
}

impl From<DebugProbeError> for Error {
    fn from(error: DebugProbeError) -> Self {
        Error::DebugProbe(error)
    }
}

impl From<std::io::Error> for Error {
    fn from(error: std::io::Error) -> Self {
        Error::StdIO(error)
    }
}

fn show_info_of_device(n: usize) -> Result<(), Error> {
    with_device(n, |link| {
        println!("Device information:");

        link
            .write_register(Port::DebugPort, 0x2, 0x2)?;

        let target_info = link
            .read_register(Port::DebugPort, 0x4)?;
        let target_info = parse_target_id(target_info);
        println!("\nTarget Identification Register (TARGETID):");
        println!(
            "\tRevision = {}, Part Number = {}, Designer = {}",
            target_info.0, target_info.3, target_info.2
        );

        let target_info = link
            .read_register(Port::DebugPort, 0x0)?;
        let target_info = parse_target_id(target_info);
        println!("\nIdentification Code Register (IDCODE):");
        println!(
            "\tProtocol = {},\n\tPart Number = {:x},\n\tJEDEC Manufacturer ID = {:x}",
            if target_info.0 == 0x4 {
                "JTAG-DP"
            } else if target_info.0 == 0x3 {
                "SW-DP"
            } else {
                "Unknown Protocol"
            },
            target_info.1,
            target_info.2
        );

        println!("\nAvailable Ports:");

        for port in 0..1 { //255 {
            use coresight::access_ports::{
                generic_ap::{
                    IDR,
                    APClass,
                },
                memory_ap::{
                    MemoryAP,
                    BASE,
                    BASE2,
                    BaseaddrFormat,
                },
            };

            let access_port = GenericAP::new(port);
            if access_port_is_valid(link, access_port) {

                let idr = link.read_register_ap(access_port, IDR::default())?;
                println!("{:#x?}", idr);

                if idr.CLASS == APClass::MEMAP {
                    let access_port: MemoryAP = access_port.into();

                    let base_register = link.read_register_ap(access_port, BASE::default())?;

                    let mut baseaddr = if BaseaddrFormat::ADIv5 == base_register.Format {
                        let base2 = link.read_register_ap(access_port, BASE2::default())?;
                        (u64::from(base2.BASEADDR) << 32)
                    } else { 0 };
                    baseaddr |= u64::from(base_register.BASEADDR << 12);

                    memory::romtable::RomTable::try_parse(link, baseaddr as u64);

                    let mut reader = memory::romtable::RomTableReader::new(link, baseaddr as u64);

                    for e in reader.entries().unwrap().iter() {
                        println!("ROM Table Entry: Component @ 0x{:08x}", e.component_addr());
                    }
                }
            }
        }

        // TODO: seems broken
        // if target_info.3 != 1
        //     || !(target_info.0 == 0x3 || target_info.0 == 0x4)
        //     || !(target_info.1 == 0xBA00 || target_info.1 == 0xBA02)
        // {
        //     return Err(Error::Custom(
        //         "The IDCODE register has not-expected contents.",
        //     ));
        // }
        Ok(())
    })
}

// revision | partno | designer | reserved
// 4 bit    | 16 bit | 11 bit   | 1 bit
fn parse_target_id(value: u32) -> (u8, u16, u16, u8) {
    (
        (value >> 28) as u8,
        (value >> 12) as u16,
        ((value >> 1) & 0x07FF) as u16,
        (value & 0x01) as u8,
    )
}

fn dump_memory(n: usize, loc: u32, words: u32) -> Result<(), Error> {
    with_device(n as usize, |link| {
        let mut data = vec![0 as u32; words as usize];

        // Start timer.
        let instant = Instant::now();

        link.read_block(loc, &mut data.as_mut_slice()).or_else(|e| Err(Error::AccessPort(e)))?;
        // Stop timer.
        let elapsed = instant.elapsed();

        // Print read values.
        for word in 0..words {
            println!("Addr 0x{:08x?}: 0x{:08x}", loc + 4 * word, data[word as usize]);
        }
        // Print stats.
        println!("Read {:?} words in {:?}", words, elapsed);

        Ok(())
    })
}

fn reset_target_of_device(n: usize, _assert: Option<bool>) -> Result<(), Error> {
    with_device(n as usize, |link: &mut MasterProbe| {
        //link.get_interface_mut::<DebugProbe>().unwrap().target_reset().or_else(|e| Err(Error::DebugProbe(e)))?;
        link.target_reset().or_else(|e| Err(Error::DebugProbe(e)))?;

        Ok(())
    })
}

fn trace_u32_on_target(n: usize, loc: u32) -> Result<(), Error> {
    use std::io::prelude::*;
    use std::thread::sleep;
    use std::time::Duration;
    use scroll::{Pwrite};

    let mut xs = vec![];
    let mut ys = vec![];

    let start = Instant::now();

    with_device(n, |link| {
        loop {
            // Prepare read.
            let elapsed = start.elapsed();
            let instant = elapsed.as_secs() * 1000 + u64::from(elapsed.subsec_millis());

            // Read data.
            let value: u32 = link.read(loc)?;

            xs.push(instant);
            ys.push(value);

            // Send value to plot.py.
            // Unwrap is safe as there is always an stdin in our case!
            let mut buf = [0 as u8; 8];
            // Unwrap is safe!
            buf.pwrite(instant, 0).unwrap();
            buf.pwrite(value, 4).unwrap();
            std::io::stdout()
                    .write(&buf)
                    .or_else(|e| Err(Error::StdIO(e)))?;
            std::io::stdout()
                    .flush()
                    .or_else(|e| Err(Error::StdIO(e)))?;

            // Schedule next read.
            let elapsed = start.elapsed();
            let instant = elapsed.as_secs() * 1000 + u64::from(elapsed.subsec_millis());
            let poll_every_ms = 50;
            let time_to_wait = poll_every_ms - instant % poll_every_ms;
            sleep(Duration::from_millis(time_to_wait));
        }
    })
}


/// Takes a closure that is handed an `DAPLink` instance and then executed.
/// After the closure is done, the USB device is always closed,
/// even in an error case inside the closure!
fn with_device<F>(n: usize, mut f: F) -> Result<(), Error>
where
    F: FnMut(&mut MasterProbe) -> Result<(), Error>
{
    let device = {
        let mut list = daplink::tools::list_daplink_devices();
        list.extend(stlink::tools::list_stlink_devices());

        list.remove(n)
    };

    let mut probe = match device.probe_type {
        DebugProbeType::DAPLink => {
            let mut link = daplink::DAPLink::new_from_probe_info(device)?;

            link.attach(Some(probe::protocol::WireProtocol::Swd))?;
            
            MasterProbe::from_specific_probe(link)
        },
        DebugProbeType::STLink => {
            let mut link = stlink::STLink::new_from_probe_info(device)?;

            link.attach(Some(probe::protocol::WireProtocol::Swd))?;
            
            MasterProbe::from_specific_probe(link)
        },
    };

    let ap = GenericAP::new(0);
    use coresight::access_ports::generic_ap::{
        IDR
    };

    let _ = dbg!(probe.read_register_ap(ap, IDR::default()));
    
    f(&mut probe)
}


fn get_connected_devices() -> Vec<DebugProbeInfo>{
    let mut links = daplink::tools::list_daplink_devices();
    links.extend(stlink::tools::list_stlink_devices());
    links
}