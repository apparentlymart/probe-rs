use crate::util::common_options::{CargoOptions, FlashOptions, ProbeOptions};
use crate::util::flash::run_flash_download;
use crate::util::rtt;
use anyhow::{Context, Result};
use probe_rs::flashing::FileDownloadError;
use probe_rs::MemoryInterface;
use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::time::Duration;
use time::UtcOffset;

pub fn run(
    common: ProbeOptions,
    path: &str,
    chip_erase: bool,
    disable_double_buffering: bool,
    timestamp_offset: UtcOffset,
) -> Result<()> {
    let mut session = common.simple_attach()?;

    let mut file = match File::open(path) {
        Ok(file) => file,
        Err(e) => return Err(FileDownloadError::IO(e)).context("Failed to open binary file."),
    };

    let mut loader = session.target().flash_loader();
    loader.load_elf_data(&mut file)?;

    run_flash_download(
        &mut session,
        Path::new(path),
        &FlashOptions {
            list_chips: false,
            list_probes: false,
            disable_progressbars: false,
            disable_double_buffering,
            reset_halt: false,
            log: None,
            restore_unwritten: false,
            flash_layout_output_path: None,
            elf: None,
            work_dir: None,
            cargo_options: CargoOptions::default(),
            probe_options: common,
        },
        loader,
        chip_erase,
    )?;

    let rtt_config = rtt::RttConfig::default();

    let scan_ranges = session.target().rtt_scan_regions.clone();

    let mut core = session.core(0)?;
    core.reset()?;
    core.run()?;

    let mut rtt_attempts = 0;
    let mut rtta = loop {
        match rtt::attach_to_rtt(
            &mut core,
            &scan_ranges,
            Path::new(path),
            &rtt_config,
            timestamp_offset,
        ) {
            Ok(target_rtt) => break Some(target_rtt),
            Err(error) => {
                if rtt_attempts < 2 {
                    rtt_attempts += 1;
                    log::debug!(
                        "RTT attach attempt {:?} failed (will retry): {}",
                        rtt_attempts,
                        error
                    );
                    std::thread::sleep(Duration::from_secs(1));
                    continue;
                } else {
                    log::error!("{:?} Continuing without RTT... ", error);
                    break None;
                }
            }
        };
    };

    if let Some(rtta) = &mut rtta {
        let mut stdout = std::io::stdout();
        loop {
            for (_ch, data) in rtta.poll_rtt_fallible(&mut core)? {
                stdout.write_all(data.as_bytes())?;
            }

            // Poll RTT with a frequency of 10 Hz
            //
            // If the polling frequency is too high,
            // the USB connection to the probe can become unstable.
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    Ok(())
}
