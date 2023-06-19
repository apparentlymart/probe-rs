//! Sequences for NXP chips.

use std::{
    sync::Arc,
    thread,
    time::{Duration, Instant},
};

use anyhow::anyhow;

use crate::{
    architecture::arm::{
        ap::{AccessPort, AccessPortError, ApAccess, GenericAp, MemoryAp, DRW, IDR, TAR},
        communication_interface::{FlushableArmAccess, Initialized},
        core::armv7m::{Aircr, Demcr, Dhcsr},
        dp::{Abort, Ctrl, DpAccess, Select, DPIDR},
        memory::adi_v5_memory_interface::ArmProbe,
        ApAddress, ArmCommunicationInterface, ArmError, DapAccess, DpAddress,
    },
    core::MemoryMappedRegister,
};

use super::ArmDebugSequence;

/// Start the debug port, and return if the device was (true) or wasn't (false)
/// powered down.
///
/// Note that this routine only supports SWD protocols. See the inline TODOs to
/// understand where JTAG support should go.
fn debug_port_start(
    interface: &mut ArmCommunicationInterface<Initialized>,
    dp: DpAddress,
    select: Select,
) -> Result<bool, ArmError> {
    interface.write_dp_register(dp, select)?;

    let ctrl = interface.read_dp_register::<Ctrl>(dp)?;

    let powered_down = !(ctrl.csyspwrupack() && ctrl.cdbgpwrupack());

    if powered_down {
        let mut ctrl = Ctrl(0);
        ctrl.set_cdbgpwrupreq(true);
        ctrl.set_csyspwrupreq(true);

        interface.write_dp_register(dp, ctrl)?;

        let start = Instant::now();

        let mut timeout = true;

        while start.elapsed() < Duration::from_micros(100_0000) {
            let ctrl = interface.read_dp_register::<Ctrl>(dp)?;

            if ctrl.csyspwrupack() && ctrl.cdbgpwrupack() {
                timeout = false;
                break;
            }
        }

        if timeout {
            return Err(ArmError::Timeout);
        }

        // TODO: Handle JTAG Specific part

        // TODO: Only run the following code when the SWD protocol is used

        // Init AP Transfer Mode, Transaction Counter, and Lane Mask (Normal Transfer Mode, Include all Byte Lanes)
        let mut ctrl = Ctrl(0);

        ctrl.set_cdbgpwrupreq(true);
        ctrl.set_csyspwrupreq(true);

        ctrl.set_mask_lane(0b1111);

        interface.write_dp_register(dp, ctrl)?;

        let mut abort = Abort(0);

        abort.set_orunerrclr(true);
        abort.set_wderrclr(true);
        abort.set_stkerrclr(true);
        abort.set_stkcmpclr(true);

        interface.write_dp_register(dp, abort)?;
    }

    Ok(powered_down)
}

/// The sequence handle for the LPC55Sxx family.
pub struct LPC55Sxx(());

impl LPC55Sxx {
    /// Create a sequence handle for the LPC55Sxx.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self(()))
    }
}

impl ArmDebugSequence for LPC55Sxx {
    fn debug_port_start(
        &self,
        interface: &mut ArmCommunicationInterface<Initialized>,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        tracing::info!("debug_port_start");

        let powered_down = self::debug_port_start(interface, dp, Select(0))?;

        if powered_down {
            enable_debug_mailbox(interface, dp)?;
        }

        Ok(())
    }

    fn reset_catch_set(
        &self,
        interface: &mut dyn ArmProbe,
        _core_type: crate::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        let mut reset_vector = 0xffff_ffff;
        let mut demcr = Demcr(interface.read_word_32(Demcr::get_mmio_address())?);

        demcr.set_vc_corereset(false);

        interface.write_word_32(Demcr::get_mmio_address(), demcr.into())?;

        // Write some stuff
        interface.write_word_32(0x40034010, 0x00000000)?; // Program Flash Word Start Address to 0x0 to read reset vector (STARTA)
        interface.write_word_32(0x40034014, 0x00000000)?; // Program Flash Word Stop Address to 0x0 to read reset vector (STOPA)
        interface.write_word_32(0x40034080, 0x00000000)?; // DATAW0: Prepare for read
        interface.write_word_32(0x40034084, 0x00000000)?; // DATAW1: Prepare for read
        interface.write_word_32(0x40034088, 0x00000000)?; // DATAW2: Prepare for read
        interface.write_word_32(0x4003408C, 0x00000000)?; // DATAW3: Prepare for read
        interface.write_word_32(0x40034090, 0x00000000)?; // DATAW4: Prepare for read
        interface.write_word_32(0x40034094, 0x00000000)?; // DATAW5: Prepare for read
        interface.write_word_32(0x40034098, 0x00000000)?; // DATAW6: Prepare for read
        interface.write_word_32(0x4003409C, 0x00000000)?; // DATAW7: Prepare for read

        interface.write_word_32(0x40034FE8, 0x0000000F)?; // Clear FLASH Controller Status (INT_CLR_STATUS)
        interface.write_word_32(0x40034000, 0x00000003)?; // Read single Flash Word (CMD_READ_SINGLE_WORD)
        interface.flush()?;

        let start = Instant::now();

        let mut timeout = true;

        while start.elapsed() < Duration::from_micros(10_0000) {
            let value = interface.read_word_32(0x40034FE0)?;

            if (value & 0x4) == 0x4 {
                timeout = false;
                break;
            }
        }

        if timeout {
            tracing::warn!("Failed: Wait for flash word read to finish");
            return Err(ArmError::Timeout);
        }

        if (interface.read_word_32(0x4003_4fe0)? & 0xB) == 0 {
            tracing::info!("No Error reading Flash Word with Reset Vector");

            reset_vector = interface.read_word_32(0x0000_0004)?;
        }

        if reset_vector != 0xffff_ffff {
            tracing::info!("Breakpoint on user application reset vector");

            interface.write_word_32(0xE000_2008, reset_vector | 1)?;
            interface.write_word_32(0xE000_2000, 3)?;
        }

        if reset_vector == 0xffff_ffff {
            tracing::info!("Enable reset vector catch");

            let mut demcr = Demcr(interface.read_word_32(Demcr::get_mmio_address())?);

            demcr.set_vc_corereset(true);

            interface.write_word_32(Demcr::get_mmio_address(), demcr.into())?;
        }

        let _ = interface.read_word_32(Dhcsr::get_mmio_address())?;

        tracing::debug!("reset_catch_set -- done");

        Ok(())
    }

    fn reset_catch_clear(
        &self,
        interface: &mut dyn ArmProbe,
        _core_type: crate::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        interface.write_word_32(0xE000_2008, 0x0)?;
        interface.write_word_32(0xE000_2000, 0x2)?;

        let mut demcr = Demcr(interface.read_word_32(Demcr::get_mmio_address())?);

        demcr.set_vc_corereset(false);

        interface.write_word_32(Demcr::get_mmio_address(), demcr.into())
    }

    fn reset_system(
        &self,
        interface: &mut dyn ArmProbe,
        _core_type: crate::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);

        let mut result = interface.write_word_32(Aircr::get_mmio_address(), aircr.into());

        if result.is_ok() {
            result = interface.flush();
        }

        if let Err(e) = result {
            tracing::debug!("Error requesting reset: {:?}", e);
        }

        tracing::info!("Waiting after reset");
        thread::sleep(Duration::from_millis(10));

        wait_for_stop_after_reset(interface)
    }
}

fn wait_for_stop_after_reset(memory: &mut dyn ArmProbe) -> Result<(), ArmError> {
    tracing::info!("Wait for stop after reset");

    thread::sleep(Duration::from_millis(10));

    let dp = memory.ap().ap_address().dp;
    let interface = memory.get_arm_communication_interface()?;

    enable_debug_mailbox(interface, dp)?;

    let mut timeout = true;

    let start = Instant::now();

    tracing::info!("Polling for reset");

    while start.elapsed() < Duration::from_micros(50_0000) {
        let dhcsr = Dhcsr(memory.read_word_32(Dhcsr::get_mmio_address())?);

        if !dhcsr.s_reset_st() {
            timeout = false;
            break;
        }
    }

    if timeout {
        return Err(ArmError::Timeout);
    }

    let dhcsr = Dhcsr(memory.read_word_32(Dhcsr::get_mmio_address())?);

    if !dhcsr.s_halt() {
        let mut dhcsr = Dhcsr(0);
        dhcsr.enable_write();
        dhcsr.set_c_halt(true);
        dhcsr.set_c_debugen(true);

        memory.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
    }

    Ok(())
}

fn enable_debug_mailbox(
    interface: &mut ArmCommunicationInterface<Initialized>,
    dp: DpAddress,
) -> Result<(), ArmError> {
    tracing::info!("LPC55xx connect srcipt start");

    let ap = ApAddress { dp, ap: 2 };

    let status: IDR = interface.read_ap_register(GenericAp::new(ap))?;

    tracing::info!("APIDR: {:?}", status);
    tracing::info!("APIDR: 0x{:08X}", u32::from(status));

    let status: u32 = interface.read_dp_register::<DPIDR>(dp)?.into();

    tracing::info!("DPIDR: 0x{:08X}", status);

    // Active DebugMailbox
    interface.write_raw_ap_register(ap, 0x0, 0x0000_0021)?;
    interface.flush()?;

    // DAP_Delay(30000)
    thread::sleep(Duration::from_micros(30000));

    let _ = interface.read_raw_ap_register(ap, 0)?;

    // Enter Debug session
    interface.write_raw_ap_register(ap, 0x4, 0x0000_0007)?;
    interface.flush()?;

    // DAP_Delay(30000)
    thread::sleep(Duration::from_micros(30000));

    let _ = interface.read_raw_ap_register(ap, 8)?;

    tracing::info!("LPC55xx connect srcipt end");
    Ok(())
}

/// Debug sequences for MIMXRT10xx MCUs.
///
/// In its current form, it uses no custom debug sequences. Instead, it ensures a reliable
/// reset sequence.
///
/// # On custom reset catch
///
/// Some tools use a custom reset catch that looks at the program image, finds the
/// reset vector, then places a breakpoint on that reset vector. This implementation
/// isn't doing that. That would be necessary if we don't control the kind of reset
/// that's happening. Since we're definitely using a SYSRESETREQ, we can rely on the
/// normal reset catch.
///
/// If the design changes such that the kind of reset isn't in our control, we'll
/// need to handle those cases.
pub struct MIMXRT10xx(());

impl MIMXRT10xx {
    /// Create a sequence handle for the MIMXRT10xx.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self(()))
    }

    /// Runtime validation of core type.
    fn check_core_type(&self, core_type: crate::CoreType) -> Result<(), ArmError> {
        const EXPECTED: crate::CoreType = crate::CoreType::Armv7em;
        if core_type != EXPECTED {
            tracing::warn!(
                "MIMXRT10xx core type supplied as {core_type:?}, but the actual core is a {EXPECTED:?}"
            );
            // Not an issue right now. Warning because it's curious.
        }
        Ok(())
    }
}

impl ArmDebugSequence for MIMXRT10xx {
    fn reset_system(
        &self,
        interface: &mut dyn ArmProbe,
        core_type: crate::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.check_core_type(core_type)?;

        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);

        // Reset happens very quickly, and takes a bit. Ignore write and flush
        // errors that will occur due to the reset reaction.
        interface
            .write_word_32(Aircr::get_mmio_address(), aircr.into())
            .ok();
        interface.flush().ok();

        // Wait for the reset to finish...
        std::thread::sleep(Duration::from_millis(100));

        let start = Instant::now();
        while start.elapsed() < Duration::from_micros(50_0000) {
            let dhcsr = match interface.read_word_32(Dhcsr::get_mmio_address()) {
                Ok(val) => Dhcsr(val),
                Err(ArmError::AccessPort {
                    source:
                        AccessPortError::RegisterRead { .. } | AccessPortError::RegisterWrite { .. },
                    ..
                }) => {
                    // Some combinations of debug probe and target (in
                    // particular, hs-probe and ATSAMD21) result in
                    // register read errors while the target is
                    // resetting.
                    //
                    // See here for more info: https://github.com/probe-rs/probe-rs/pull/1174#issuecomment-1275568493
                    continue;
                }
                Err(err) => return Err(err),
            };

            // Wait until the S_RESET_ST bit is cleared on a read
            if !dhcsr.s_reset_st() {
                return Ok(());
            }
        }

        Err(ArmError::Timeout)
    }
}

/// Debug sequences for MIMXRT11xx MCUs.
///
/// Currently only supports the Cortex M7.
pub struct MIMXRT11xx(());

impl MIMXRT11xx {
    /// Create a sequence handle for the MIMXRT10xx.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self(()))
    }

    fn prepare_cm7_trap_code(
        &self,
        ap: MemoryAp,
        interface: &mut ArmCommunicationInterface<Initialized>,
    ) -> Result<(), ArmError> {
        const START: u32 = 0x2001FF00;
        const IOMUX_LPSR_GPR26: u32 = 0x40C0C068;

        interface.write_ap_register(ap, TAR { address: START })?;
        interface.write_ap_register(ap, DRW { data: START + 0x20 })?;

        interface.write_ap_register(ap, TAR { address: START + 4 })?;
        interface.write_ap_register(ap, DRW { data: 0x23105 })?;

        interface.write_ap_register(
            ap,
            TAR {
                address: IOMUX_LPSR_GPR26,
            },
        )?;
        interface.write_ap_register(ap, DRW { data: START >> 7 })?;
        Ok(())
    }

    fn prepare_cm4_trap_code(
        &self,
        ap: MemoryAp,
        interface: &mut ArmCommunicationInterface<Initialized>,
    ) -> Result<(), ArmError> {
        const START: u32 = 0x20250000;
        const IOMUX_LPSR_GPR0: u32 = 0x40c0c000;
        const IOMUX_LPSR_GPR1: u32 = 0x40c0c004;
        interface.write_ap_register(ap, TAR { address: START })?;
        interface.write_ap_register(ap, DRW { data: START + 0x20 })?;

        interface.write_ap_register(ap, TAR { address: START + 4 })?;
        interface.write_ap_register(ap, DRW { data: 0x23F041 })?;

        interface.write_ap_register(
            ap,
            TAR {
                address: IOMUX_LPSR_GPR0,
            },
        )?;
        interface.write_ap_register(
            ap,
            DRW {
                data: START & 0xFFFF,
            },
        )?;

        interface.write_ap_register(
            ap,
            TAR {
                address: IOMUX_LPSR_GPR1,
            },
        )?;
        interface.write_ap_register(ap, DRW { data: START >> 16 })?;
        Ok(())
    }

    fn release_cm4(
        &self,
        ap: MemoryAp,
        interface: &mut ArmCommunicationInterface<Initialized>,
    ) -> Result<(), ArmError> {
        const SRC_SCR: u32 = 0x40c04000;
        interface.write_ap_register(ap, TAR { address: SRC_SCR })?;
        interface.write_ap_register(ap, DRW { data: 1 })?;
        Ok(())
    }

    fn change_reset_modes(
        &self,
        ap: MemoryAp,
        interface: &mut ArmCommunicationInterface<Initialized>,
    ) -> Result<(), ArmError> {
        const SRC_SBMR: u32 = 0x40c04004;
        interface.write_ap_register(ap, TAR { address: SRC_SBMR })?;
        let DRW { data: mut src_sbmr } = interface.read_ap_register(ap)?;
        src_sbmr |= 0xF << 10; // Puts both cores into "do not reset."
        interface.write_ap_register(ap, DRW { data: src_sbmr })?;
        Ok(())
    }
}

impl ArmDebugSequence for MIMXRT11xx {
    fn debug_port_start(
        &self,
        interface: &mut ArmCommunicationInterface<Initialized>,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        tracing::debug!("debug_port_start");
        // Note that debug_port_start only supports SWD protocols,
        // which means the MIMXRT11xx only supports SWD right now.
        // See its documentation and TODOs.
        self::debug_port_start(interface, dp, Select(0))?;

        let ap = ApAddress { dp, ap: 0 };
        let ap = MemoryAp::new(ap);

        tracing::debug!("Prepare trap code for Cortex M7");
        self.prepare_cm7_trap_code(ap, interface)?;

        tracing::debug!("Prepare trap code for Cortex M4");
        self.prepare_cm4_trap_code(ap, interface)?;

        tracing::debug!("Release the CM4");
        self.release_cm4(ap, interface)?;

        tracing::debug!("Change reset mode of both cores");
        self.change_reset_modes(ap, interface)?;
        Ok(())
    }

    fn reset_system(
        &self,
        interface: &mut dyn ArmProbe,
        _: crate::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        // It's unpredictable to VECTRESET a core if it's not halted and
        // in debug state.
        tracing::debug!("Halting MIMXRT11xx core before VECTRESET");
        let mut dhcsr = Dhcsr(0);
        dhcsr.set_c_halt(true);
        dhcsr.set_c_debugen(true);
        dhcsr.enable_write();

        interface.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
        std::thread::sleep(Duration::from_millis(100));

        // Initial testing showed that a SYSRESET (the default reset approach)
        // can result in an unreliable programming sequence, particularly if
        // the target we're reprogramming is interrupting / excepting.
        //
        // The debug port setup (above) will trap the core(s) after this VECRESET.
        // Once that trap happens, we're ready to debug / flash.
        tracing::debug!("Resetting MIMXRT11xx with VECTRESET");
        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_vectreset(true);

        interface
            .write_word_32(Aircr::get_mmio_address(), aircr.into())
            .ok();
        interface.flush().ok();

        std::thread::sleep(Duration::from_millis(100));

        interface.read_word_32(Dhcsr::get_mmio_address())?;
        Ok(())
    }
}

/// Debug sequences for MIMXRT5xxS MCUs.
///
/// MCUs in this series do not have any on-board flash memory, and instead
/// there is an unprogrammable boot ROM which attempts to find a suitable
/// program from a variety of different sources. The entry point for
/// application code therefore varies depending on the boot medium.
///
/// Because the system begins execution in the boot ROM, it isn't possible
/// to use a standard reset vector catch on this platform. Instead, the series
/// datasheet (section 60.3.4) describes the following protocol:
///
/// - Set a data watchpoint for a read from location 0x50002034.
/// - Use SYSRESETREQ to reset the core and peripherals.
/// - Wait 100ms to allow the boot ROM to re-enable debug.
/// - Check whether the core is halted due to the watchpoint, by checking DHCSR.
/// - If the core doesn't halt or halts for some reason other than the
///   watchpoint, use the special debug mailbox protocol to exit the ISP mode
///   and enter an infinite loop, at which point we can halt the MCU explicitly.
/// - Clear the data watchpoint.
///
/// The debug mailbox protocol handles, among other things, recovering debug
/// access when the part enters its ISP mode. ISP mode has debug disabled to
/// prevent tampering with the system's security features. Datasheet
/// section 60.3.1 describes the special debug recovery process.
///
/// This type's [`ArmDebugSequence`] implementation implements these two
/// sequences to ensure that probe-rs can reset and halt the core.
pub struct MIMXRT5xxS {}

impl MIMXRT5xxS {
    const DWT_COMP0: u64 = 0xE0001020;
    const DWT_MASK0: u64 = 0xE0001024;
    const DWT_FUNCTION0: u64 = 0xE0001028;
    const BOOTROM_READ_SIGNAL_ADDR: u32 = 0x50002034;

    /// Create a sequence handle for the MIMXRT5xxS.
    pub fn create() -> Arc<dyn ArmDebugSequence> {
        Arc::new(Self {})
    }

    /// Runtime validation of core type.
    fn check_core_type(&self, core_type: crate::CoreType) -> Result<(), ArmError> {
        if core_type != crate::CoreType::Armv8m {
            // Caller has selected the wrong chip name, presumably.
            return Err(ArmError::ArchitectureRequired(&["ARMv8"]));
        }
        Ok(())
    }

    /// Polls until DHCSR indicates that the core has halted in debug mode,
    /// or until a timeout expires (in which case it returns a timeout error).
    fn wait_for_debug_halt(&self, core: &mut dyn ArmProbe) -> Result<(), ArmError> {
        let start = Instant::now();
        while start.elapsed() < Duration::from_micros(500_000) {
            let dhcsr = match core.read_word_32(Dhcsr::get_mmio_address()) {
                Ok(val) => Dhcsr(val),
                Err(ArmError::AccessPort {
                    source:
                        AccessPortError::RegisterRead { .. } | AccessPortError::RegisterWrite { .. },
                    ..
                }) => {
                    // Some combinations of debug probe and target (in
                    // particular, hs-probe and ATSAMD21) result in
                    // register read errors while the target is
                    // resetting.
                    //
                    // See here for more info: https://github.com/probe-rs/probe-rs/pull/1174#issuecomment-1275568493
                    continue;
                }
                Err(err) => return Err(err),
            };

            // We're hoping to find the core halted.
            if dhcsr.s_halt() {
                return Ok(());
            }
        }
        Err(ArmError::Timeout)
    }
}

impl ArmDebugSequence for MIMXRT5xxS {
    /// Arranges for a catch on reset by setting a data watchpoint on the
    /// special address documented in the platform datasheet, since the
    /// main reset vector is in the system boot ROM rather than in user code.
    fn reset_catch_set(
        &self,
        core: &mut dyn ArmProbe,
        core_type: probe_rs_target::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        self.check_core_type(core_type)?;

        tracing::debug!("set reset catch debug watchpoint for MIMXRT5xxS");

        // TODO: Back up what's already in the DWT unit 0 registers before we
        // clobber them, so we can restore in reset_watch_clear?

        // Disable whatever the first data watchpoint is doing.
        core.write_word_32(Self::DWT_FUNCTION0, 0)?;
        core.flush()?;

        // Read clears the flag for whether the watchpoint has matched.
        // We don't actually care about the result, because if it was
        // set then it was in response to something other than our goal here.
        core.read_word_32(Self::DWT_FUNCTION0)?;

        // Enable halting debug in DHCSR.
        let mut dhcsr = Dhcsr(0);
        dhcsr.set_c_debugen(true);
        dhcsr.enable_write();
        core.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
        core.flush()?;

        // Halt on read from the special address given in the datasheet.
        core.write_word_32(Self::DWT_COMP0, Self::BOOTROM_READ_SIGNAL_ADDR)?;
        core.write_word_32(Self::DWT_MASK0, 0x00000000)?;
        core.write_word_32(
            Self::DWT_FUNCTION0,
            0b0101, // Generate debug watchpoint event on read
        )?;
        core.flush()?;

        tracing::debug!("debug watchpoint for MIMXRT5xxS is set");

        Ok(())
    }

    fn reset_catch_clear(
        &self,
        core: &mut dyn ArmProbe,
        core_type: probe_rs_target::CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        self.check_core_type(core_type)?;

        tracing::debug!("clear reset catch debug watchpoint for MIMXRT5xxS");

        // Disable the zeroth data watchpoint.
        core.write_word_32(Self::DWT_FUNCTION0, 0)?;
        core.flush()?;

        tracing::debug!("debug watchpoint for MIMXRT5xxS is cleared");

        Ok(())
    }

    fn reset_system(
        &self,
        interface: &mut dyn ArmProbe,
        core_type: crate::CoreType,
        _: Option<u64>,
    ) -> Result<(), ArmError> {
        self.check_core_type(core_type)?;

        tracing::debug!("system reset for MIMXRT5xxS");

        let mut aircr = Aircr(0);
        aircr.vectkey();
        aircr.set_sysresetreq(true);
        // The reset request happens so quickly that our write or flush will
        // often appear to fail, so we ignore errors here.
        interface
            .write_word_32(Aircr::get_mmio_address(), aircr.into())
            .ok();
        interface.flush().ok();

        // Datasheet requires that we wait 100ms to give the boot ROM time
        // to enable debug.
        std::thread::sleep(Duration::from_millis(100));

        // Now we'll poll DHCSR for a while in the hope that the core hits
        // our data watchpoint and gets halted. This is the happy path for
        // when the system manages to boot into valid application code that
        // leaves debug enabled.
        match self.wait_for_debug_halt(interface) {
            Ok(_) => {
                // We've been successful only if our watchpoint indicates that
                // it has matched at least once since we set it up.
                let v = (interface.read_word_32(Self::DWT_FUNCTION0)? >> 24) & 0b1;
                if v == 1 {
                    return Ok(());
                }
                tracing::trace!("MIMXRT5xxS core halted, but not due to our watchpoint; attempting debug mailbox reset");
            }
            Err(ArmError::Timeout) => {
                tracing::trace!("MIMXRT5xxS core did not halt; attempting debug mailbox reset");
            }
            Err(e) => return Err(e),
        }

        // If we fall out here then the core didn't end up in a debug-halted
        // state, which might be because it's entered ISP mode with debugging
        // disabled.
        let mut mbox = MIMXRT5xxSDebugMailbox::new(interface);
        mbox.request_resync()?;
        mbox.start_dm_ap()?;
        mbox.start_debug_session()?;

        // The core should now be spinning in an infinite loop, and so we'll
        // halt it here so that the caller can do programming or whatever else
        // it was intending to do.
        tracing::trace!("halting MIMXRT5xxS core after mailbox reset");
        let mut dhcsr = Dhcsr(0);
        dhcsr.set_c_halt(true);
        dhcsr.set_c_debugen(true);
        dhcsr.enable_write();
        interface.write_word_32(Dhcsr::get_mmio_address(), dhcsr.into())?;
        interface.flush()?;

        tracing::trace!("waiting for MIMXRT5xxS core to halt");
        self.wait_for_debug_halt(interface)
    }
}

/// Interface to the MIMXRT5xxS debug mailbox, which allows reacquiring debug
/// access to the system when it's running boot ROM code, which normally
/// blocks debug access.
struct MIMXRT5xxSDebugMailbox<'a> {
    interface: &'a mut dyn ArmProbe,
}

impl<'a> MIMXRT5xxSDebugMailbox<'a> {
    const REG_BASE: u64 = 0x4010F000;
    const REG_CSW: u64 = Self::REG_BASE;
    const REG_REQUEST: u64 = Self::REG_BASE + 4;
    const REG_RETURN: u64 = Self::REG_BASE + 8;

    fn new(interface: &'a mut dyn ArmProbe) -> Self {
        Self { interface }
    }

    fn request_resync(&mut self) -> Result<(), ArmError> {
        self.raw_csw_write(
            0b1, // RESYNCH_REQ
        )?;

        // We need to poll CSW until it returns zero to signal that the
        // resync has completed.
        let start = Instant::now();
        while start.elapsed() < Duration::from_micros(500_000) {
            let v = match self.raw_csw_read() {
                Ok(v) => v,
                Err(_) => continue, // errors expected for a while after resync
            };
            if v == 0 {
                return Ok(());
            }
        }
        Err(ArmError::Other(anyhow!("Debug mailbox resync timeout")))
    }

    fn start_dm_ap(&mut self) -> Result<(), ArmError> {
        self.raw_request_write(0x00000001)?;
        let status = self.raw_response_read()? & 0xffff;
        if status != 0x0000 {
            return Err(ArmError::Other(anyhow!(
                "Start DM-AP failed: {:08x}",
                status
            )));
        }
        Ok(())
    }

    fn start_debug_session(&mut self) -> Result<(), ArmError> {
        self.raw_request_write(0x00000007)?;
        let status = self.raw_response_read()? & 0xffff;
        if status != 0x0000 {
            return Err(ArmError::Other(anyhow!(
                "DM-AP Start Debug Session failed: {:08x}",
                status
            )));
        }
        Ok(())
    }

    fn raw_csw_write(&mut self, v: u32) -> Result<(), ArmError> {
        self.interface.write_word_32(Self::REG_CSW, v)?;
        self.interface.flush()?;
        Ok(())
    }

    fn raw_csw_read(&mut self) -> Result<u32, ArmError> {
        self.interface.read_word_32(Self::REG_CSW)
    }

    fn raw_request_write(&mut self, v: u32) -> Result<(), ArmError> {
        self.interface.write_word_32(Self::REG_REQUEST, v)?;
        self.interface.flush()?;
        Ok(())
    }

    fn raw_response_read(&mut self) -> Result<u32, ArmError> {
        self.interface.read_word_32(Self::REG_RETURN)
    }
}
