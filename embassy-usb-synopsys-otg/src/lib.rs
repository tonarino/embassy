#![cfg_attr(not(test), no_std)]
#![allow(async_fn_in_trait)]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

// This must go FIRST so that all the other modules see its macros.
mod fmt;

use crate::regs::Diepint;
use crate::regs::Doepint;
use crate::regs::Gintsts;
use crate::regs::Grxsts;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::AtomicUsize;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Bus as _, Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointIn, EndpointInfo, EndpointOut,
    EndpointType, Event, Unsupported,
};

use crate::fmt::Bytes;

pub mod otg_v1;

use otg_v1::{regs, vals, Otg};

fn print_interrupts(ints: Gintsts) {
    if ints.0 == 0 {
        return;
    }

    info!("USB interrupts:");

    if ints.mmis() {
        info!("\tMode mismatch interrupt");
    }

    if ints.otgint() {
        info!("\tOTG interrupt");
    }

    if ints.sof() {
        info!("\tStart of frame interrupt");
    }

    if ints.rxflvl() {
        info!("\tRxFIFO non-empty interrupt");
    }

    if ints.nptxfe() {
        info!("\tNon-periodic TxFIFO empty interrupt");
    }

    if ints.ginakeff() {
        info!("\tGlobal IN non-periodic NAK effective interrupt");
    }

    if ints.goutnakeff() {
        info!("\tGlobal OUT NAK effective interrupt");
    }

    if ints.esusp() {
        info!("\tEarly suspend interrupt");
    }

    if ints.usbsusp() {
        info!("\tUSB suspend interrupt");
    }

    if ints.usbrst() {
        info!("\tUSB reset interrupt");
    }

    if ints.enumdne() {
        info!("\tEnumeration done interrupt");
    }

    if ints.isoodrp() {
        info!("\tIsochronous OUT packet dropped interrupt");
    }

    if ints.eopf() {
        info!("\tEnd of periodic frame interrupt");
    }

    if ints.iepint() {
        info!("\tIN endpoint interrupt");
    }

    if ints.oepint() {
        info!("\tOUT endpoint interrupt");
    }

    if ints.iisoixfr() {
        info!("\tIncomplete isochronous IN transfer interrupt");
    }

    if ints.ipxfr_incompisoout() {
        info!("\tIncomplete periodic transfer (host mode) / Incomplete isochronous OUT transfer (device mode)");
    }

    if ints.datafsusp() {
        info!("\tData fetch suspended");
    }

    if ints.hprtint() {
        info!("\tHost port interrupt");
    }

    if ints.hcint() {
        info!("\tHost channels interrupt");
    }

    if ints.ptxfe() {
        info!("\tPeriodic TxFIFO empty interrupt");
    }

    if ints.cidschg() {
        info!("\tConnector ID status change interrupt");
    }

    if ints.discint() {
        info!("\tDisconnect detected interrupt");
    }

    if ints.srqint() {
        info!("\tSession request/new session detected interrupt");
    }

    if ints.wkupint() {
        info!("\tResume/remote wakeup detected interrupt");
    }
}

fn print_rx_status(status: Grxsts) {
    info!("RX Status:");

    info!("\tEndpoint number: {}", status.epnum());
    info!("\tByte count: {}", status.bcnt());
    info!("\tData packet ID: {}", status.dpid() as u8);
    info!("\tPacket status: {}", status.pktstsd() as u8);
    info!("\tFrame number: {}", status.frmnum());
}

fn print_doepint(reg: Doepint) {
    info!("Doepint Status:");

    if reg.xfrc() {
        info!("\tTransfer Complete");
    }
    if reg.epdisd() {
        info!("\tEndpoint Disabled");
    }
    if reg.stup() {
        info!("\tSetup");
    }
    if reg.otepdis() {
        info!("\tOUT token received when endpoint disabled");
    }
    if reg.b2bstup() {
        info!("\tBack to back setup");
    }
    if reg.stpktrx() {
        info!("\tSetup Packet Received  ");
    }
}

fn print_diepint(reg: Diepint) {
    info!("Diepint Status:");

    if reg.xfrc() {
        info!("\tTransfer Complete");
    }
    if reg.epdisd() {
        info!("\tEndpoint Disabled");
    }
    if reg.toc() {
        info!("\tTimeout condition");
    }
    if reg.ittxfe() {
        info!("\tIN token received when Tx FIFO is empty");
    }
    if reg.inepne() {
        info!("\tIN endpoint NAK effective");
    }
    if reg.txfe() {
        info!("\tTX FIFO empty");
    }
}

/// Handle interrupts.
pub unsafe fn on_interrupt<const MAX_EP_COUNT: usize>(r: Otg, state: &State<MAX_EP_COUNT>) {
    trace!("irq");

    let ints = r.gintsts().read();
    print_interrupts(ints);

    if ints.wkupint() || ints.usbsusp() || ints.usbrst() || ints.enumdne() || ints.otgint() || ints.srqint() {
        // Mask interrupts and notify `Bus` to process them
        r.gintmsk().write(|w| {
            w.set_iepint(true);
            w.set_oepint(true);
        });
        state.bus_waker.wake();
    }

    // IN endpoint interrupt
    if ints.iepint() {
        let mut ep_mask = r.daint().read().iepint();
        let mut ep_num = 0;

        // Iterate over endpoints while there are non-zero bits in the mask
        while ep_mask != 0 {
            if ep_mask & 1 != 0 {
                let ep_ints = r.diepint(ep_num).read();
                print_diepint(ep_ints);
                // clear all
                r.diepint(ep_num).write_value(ep_ints);

                if ep_ints.xfrc() {
                    if state.ep_states[ep_num].in_transfer_done.load(Ordering::Acquire) {
                        error!(
                            "got transfer complete inerrupt on IN ep#{} but no transfer is setup",
                            ep_num
                        )
                    }

                    trace!("in ep={} transfer complete", ep_num);
                    state.ep_states[ep_num].in_transfer_done.store(true, Ordering::Relaxed);
                }

                state.ep_states[ep_num].in_waker.wake();
                trace!("in ep={} irq val={:08x}", ep_num, ep_ints.0);
            }

            ep_mask >>= 1;
            ep_num += 1;
        }
    }

    // out endpoint interrupt
    if ints.oepint() {
        info!("oepint");
        let mut ep_mask = r.daint().read().oepint();
        let mut ep_num = 0;

        // Iterate over endpoints while there are non-zero bits in the mask
        while ep_mask != 0 {
            if ep_mask & 1 != 0 {
                let ep_ints = r.doepint(ep_num).read();
                print_doepint(ep_ints);
                // clear all
                r.doepint(ep_num).write_value(ep_ints);

                if ep_ints.stup() {
                    info!("rxdpid_stupcnt: {}", r.doeptsiz(ep_num).read().rxdpid_stupcnt());
                    info!("setup buffer: {}", Bytes(u32_to_u8(&state.cp_state.setup_data)));

                    assert!(ep_num == 0);

                    // Reset the DMA address for our setup buffer
                    // TODO(goodhoko): this shouldn't be needed. We should instead reset the
                    // address before enabling the EP again in `ControlPipe::setup()`.
                    r.doepdma(ep_num).write_value(state.cp_state.setup_data.as_ptr() as u32);

                    // Setup packet arrived. Let future in setup() know.
                    if !state.cp_state.awaiting_setup_packet.load(Ordering::Relaxed) {
                        error!("Received a SETUP packet but there's no future to process it");
                    }

                    state.cp_state.awaiting_setup_packet.store(false, Ordering::Release);
                } else if ep_ints.xfrc() {
                    if state.ep_states[ep_num].out_transfer_done.load(Ordering::Acquire) {
                        error!(
                            "got transfer complete interrupt on OUT ep#{} but no transfer is setup",
                            ep_num
                        )
                    }
                    info!("marking OUT transfer on EP#{} as done", ep_num);
                    state.ep_states[ep_num].out_transfer_done.store(true, Ordering::Relaxed);

                    let remaining_bytes = r.doeptsiz(ep_num).read().xfrsiz();

                    let bytes_read = state.ep_states[ep_num]
                        .out_transfer_requested_bytes
                        .load(Ordering::Relaxed)
                        - remaining_bytes as usize;

                    state.ep_states[ep_num]
                        .out_transferred_bytes
                        .store(bytes_read as usize, Ordering::Relaxed);

                    trace!("out ep={} transfer complete, {} bytes read", ep_num, bytes_read);
                }

                state.ep_states[ep_num].out_waker.wake();
                info!("out ep={} irq val={:08x}", ep_num, ep_ints.0);
            }

            ep_mask >>= 1;
            ep_num += 1;
        }
    }
}

/// USB PHY type
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PhyType {
    /// Internal Full-Speed PHY
    ///
    /// Available on most High-Speed peripherals.
    InternalFullSpeed,
    /// Internal High-Speed PHY
    ///
    /// Available on a few STM32 chips.
    InternalHighSpeed,
    /// External ULPI Full-Speed PHY (or High-Speed PHY in Full-Speed mode)
    ExternalFullSpeed,
    /// External ULPI High-Speed PHY
    ExternalHighSpeed,
}

impl PhyType {
    /// Get whether this PHY is any of the internal types.
    pub fn internal(&self) -> bool {
        match self {
            PhyType::InternalFullSpeed | PhyType::InternalHighSpeed => true,
            PhyType::ExternalHighSpeed | PhyType::ExternalFullSpeed => false,
        }
    }

    /// Get whether this PHY is any of the high-speed types.
    pub fn high_speed(&self) -> bool {
        match self {
            PhyType::InternalFullSpeed | PhyType::ExternalFullSpeed => false,
            PhyType::ExternalHighSpeed | PhyType::InternalHighSpeed => true,
        }
    }

    fn to_dspd(&self) -> vals::Dspd {
        match self {
            PhyType::InternalFullSpeed => vals::Dspd::FULL_SPEED_INTERNAL,
            PhyType::InternalHighSpeed => vals::Dspd::HIGH_SPEED,
            PhyType::ExternalFullSpeed => vals::Dspd::FULL_SPEED_EXTERNAL,
            PhyType::ExternalHighSpeed => vals::Dspd::HIGH_SPEED,
        }
    }
}

struct EpState {
    in_waker: AtomicWaker,
    out_waker: AtomicWaker,

    in_transfer_done: AtomicBool,

    out_transfer_done: AtomicBool,
    out_transfer_requested_bytes: AtomicUsize,
    out_transferred_bytes: AtomicUsize,
}

// SAFETY: The EndpointAllocator ensures that the buffer points to valid memory exclusive for each endpoint and is
// large enough to hold the maximum packet size. Access to the buffer is synchronized between the USB interrupt and the
// EndpointOut impl using the out_size atomic variable.
unsafe impl Send for EpState {}
unsafe impl Sync for EpState {}

struct ControlPipeSetupState {
    /// Holds received SETUP packets. Available if [Ep0State::setup_ready] is true.
    setup_data: [u32; 12],
    awaiting_setup_packet: AtomicBool,
}

/// USB OTG driver state.
pub struct State<const EP_COUNT: usize> {
    cp_state: ControlPipeSetupState,
    ep_states: [EpState; EP_COUNT],
    bus_waker: AtomicWaker,
}

unsafe impl<const EP_COUNT: usize> Send for State<EP_COUNT> {}
unsafe impl<const EP_COUNT: usize> Sync for State<EP_COUNT> {}

impl<const EP_COUNT: usize> State<EP_COUNT> {
    /// Create a new State.
    pub const fn new() -> Self {
        Self {
            cp_state: ControlPipeSetupState {
                // TODO(goodhoko): how many bytes do we *actually* need for setup packets?
                // We ever only read two words (8 bytes) at a time from this buffer i.e. one SETUP
                // packet. The data sheet however asks to allocate at least 3 SETUP packets worth of
                // space???
                setup_data: [0; 12],
                awaiting_setup_packet: AtomicBool::new(false),
            },
            ep_states: [const {
                EpState {
                    in_waker: AtomicWaker::new(),
                    out_waker: AtomicWaker::new(),
                    in_transfer_done: AtomicBool::new(true),
                    out_transfer_done: AtomicBool::new(true),
                    out_transfer_requested_bytes: AtomicUsize::new(0),
                    out_transferred_bytes: AtomicUsize::new(0),
                }
            }; EP_COUNT],
            bus_waker: AtomicWaker::new(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct EndpointData {
    ep_type: EndpointType,
    max_packet_size: u16,
    fifo_size_words: u16,
}

/// USB driver config.
#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// Enable VBUS detection.
    ///
    /// The USB spec requires USB devices monitor for USB cable plug/unplug and react accordingly.
    /// This is done by checking whether there is 5V on the VBUS pin or not.
    ///
    /// If your device is bus-powered (powers itself from the USB host via VBUS), then this is optional.
    /// (If there's no power in VBUS your device would be off anyway, so it's fine to always assume
    /// there's power in VBUS, i.e. the USB cable is always plugged in.)
    ///
    /// If your device is self-powered (i.e. it gets power from a source other than the USB cable, and
    /// therefore can stay powered through USB cable plug/unplug) then you MUST set this to true.
    ///
    /// If you set this to true, you must connect VBUS to PA9 for FS, PB13 for HS, possibly with a
    /// voltage divider. See ST application note AN4879 and the reference manual for more details.
    pub vbus_detection: bool,

    /// Enable transceiver delay.
    ///
    /// Some ULPI PHYs like the Microchip USB334x series require a delay between the ULPI register write that initiates
    /// the HS Chirp and the subsequent transmit command, otherwise the HS Chirp does not get executed and the deivce
    /// enumerates in FS mode. Some USB Link IP like those in the STM32H7 series support adding this delay to work with
    /// the affected PHYs.
    pub xcvrdly: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            vbus_detection: false,
            xcvrdly: false,
        }
    }
}

/// USB OTG driver.
pub struct Driver<'d, const MAX_EP_COUNT: usize> {
    config: Config,
    ep_in: [Option<EndpointData>; MAX_EP_COUNT],
    ep_out: [Option<EndpointData>; MAX_EP_COUNT],
    instance: OtgInstance<'d, MAX_EP_COUNT>,
}

impl<'d, const MAX_EP_COUNT: usize> Driver<'d, MAX_EP_COUNT> {
    /// Initializes the USB OTG peripheral.
    ///
    /// # Arguments
    ///
    /// * `instance` - The USB OTG peripheral instance and its configuration.
    /// * `config` - The USB driver configuration.
    pub fn new(instance: OtgInstance<'d, MAX_EP_COUNT>, config: Config) -> Self {
        Self {
            config,
            ep_in: [None; MAX_EP_COUNT],
            ep_out: [None; MAX_EP_COUNT],
            instance,
        }
    }

    /// Returns the total amount of words (u32) allocated in dedicated FIFO.
    fn allocated_fifo_words(&self) -> u16 {
        self.instance.extra_rx_fifo_words + ep_fifo_size(&self.ep_out) + ep_fifo_size(&self.ep_in)
    }

    /// Creates an [`Endpoint`] with the given parameters.
    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, D>, EndpointAllocError> {
        trace!(
            "allocating type={:?} mps={:?} interval_ms={}, dir={:?}",
            ep_type,
            max_packet_size,
            interval_ms,
            D::dir()
        );

        let fifo_size_words = match D::dir() {
            Direction::Out => (max_packet_size + 3) / 4,
            // INEPTXFD requires minimum size of 16 words
            Direction::In => u16::max((max_packet_size + 3) / 4, 16),
        };

        if fifo_size_words + self.allocated_fifo_words() > self.instance.fifo_depth_words {
            error!("Not enough FIFO capacity");
            return Err(EndpointAllocError);
        }

        let eps = match D::dir() {
            Direction::Out => &mut self.ep_out[..self.instance.endpoint_count],
            Direction::In => &mut self.ep_in[..self.instance.endpoint_count],
        };

        // Find endpoint slot
        let slot = if let Some(addr) = ep_addr {
            // Use the specified endpoint address
            let requested_index = addr.index();
            if requested_index >= self.instance.endpoint_count {
                return Err(EndpointAllocError);
            }
            if requested_index == 0 && ep_type != EndpointType::Control {
                return Err(EndpointAllocError); // EP0 is reserved for control
            }
            if eps[requested_index].is_some() {
                return Err(EndpointAllocError); // Already allocated
            }
            Some((requested_index, &mut eps[requested_index]))
        } else {
            // Find any free endpoint slot
            eps.iter_mut().enumerate().find(|(i, ep)| {
                if *i == 0 && ep_type != EndpointType::Control {
                    // reserved for control pipe
                    false
                } else {
                    ep.is_none()
                }
            })
        };

        let index = match slot {
            Some((index, ep)) => {
                *ep = Some(EndpointData {
                    ep_type,
                    max_packet_size,
                    fifo_size_words,
                });
                index
            }
            None => {
                error!("No free endpoints available");
                return Err(EndpointAllocError);
            }
        };

        trace!("  index={}", index);

        let state = &self.instance.state.ep_states[index];

        Ok(Endpoint {
            _phantom: PhantomData,
            regs: self.instance.regs,
            state,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, D::dir()),
                ep_type,
                max_packet_size,
                interval_ms,
            },
        })
    }
}

impl<'d, const MAX_EP_COUNT: usize> embassy_usb_driver::Driver<'d> for Driver<'d, MAX_EP_COUNT> {
    type EndpointOut = Endpoint<'d, Out>;
    type EndpointIn = Endpoint<'d, In>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d, MAX_EP_COUNT>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let ep_out = self
            .alloc_endpoint(EndpointType::Control, None, control_max_packet_size, 0)
            .unwrap();
        let ep_in = self
            .alloc_endpoint(EndpointType::Control, None, control_max_packet_size, 0)
            .unwrap();
        assert_eq!(ep_out.info.addr.index(), 0);
        assert_eq!(ep_in.info.addr.index(), 0);

        trace!("start");

        let regs = self.instance.regs;
        let cp_setup_state = &self.instance.state.cp_state;
        (
            Bus {
                config: self.config,
                ep_in: self.ep_in,
                ep_out: self.ep_out,
                inited: false,
                instance: self.instance,
            },
            ControlPipe {
                max_packet_size: control_max_packet_size,
                setup_state: cp_setup_state,
                ep_out,
                ep_in,
                regs,
            },
        )
    }
}

/// USB bus.
pub struct Bus<'d, const MAX_EP_COUNT: usize> {
    config: Config,
    ep_in: [Option<EndpointData>; MAX_EP_COUNT],
    ep_out: [Option<EndpointData>; MAX_EP_COUNT],
    instance: OtgInstance<'d, MAX_EP_COUNT>,
    inited: bool,
}

impl<'d, const MAX_EP_COUNT: usize> Bus<'d, MAX_EP_COUNT> {
    fn restore_irqs(&mut self) {
        self.instance.regs.gintmsk().write(|w| {
            w.set_usbrst(true);
            w.set_enumdnem(true);
            w.set_usbsuspm(true);
            w.set_wuim(true);
            w.set_iepint(true);
            w.set_oepint(true);
            // w.set_rxflvlm(true);
            w.set_srqim(true);
            w.set_otgint(true);
        });
    }

    /// Returns the PHY type.
    pub fn phy_type(&self) -> PhyType {
        self.instance.phy_type
    }

    /// Configures the PHY as a device.
    pub fn configure_as_device(&mut self) {
        let r = self.instance.regs;
        let phy_type = self.instance.phy_type;
        r.gusbcfg().write(|w| {
            // Force device mode
            w.set_fdmod(true);
            // Enable internal full-speed PHY
            w.set_physel(phy_type.internal() && !phy_type.high_speed());
        });
    }

    /// Applies configuration specific to
    /// Core ID 0x0000_1100 and 0x0000_1200
    pub fn config_v1(&mut self) {
        let r = self.instance.regs;
        let phy_type = self.instance.phy_type;
        assert!(phy_type != PhyType::InternalHighSpeed);

        r.gccfg_v1().modify(|w| {
            // Enable internal full-speed PHY, logic is inverted
            w.set_pwrdwn(phy_type.internal());
        });

        // F429-like chips have the GCCFG.NOVBUSSENS bit
        r.gccfg_v1().modify(|w| {
            w.set_novbussens(!self.config.vbus_detection);
            w.set_vbusasen(false);
            w.set_vbusbsen(self.config.vbus_detection);
            w.set_sofouten(false);
        });
    }

    /// Applies configuration specific to
    /// Core ID 0x0000_2000, 0x0000_2100, 0x0000_2300, 0x0000_3000 and 0x0000_3100
    pub fn config_v2v3(&mut self) {
        let r = self.instance.regs;
        let phy_type = self.instance.phy_type;

        // F446-like chips have the GCCFG.VBDEN bit with the opposite meaning
        r.gccfg_v2().modify(|w| {
            // Enable internal full-speed PHY, logic is inverted
            w.set_pwrdwn(phy_type.internal() && !phy_type.high_speed());
            w.set_phyhsen(phy_type.internal() && phy_type.high_speed());
        });

        r.gccfg_v2().modify(|w| {
            w.set_vbden(self.config.vbus_detection);
        });

        // Force B-peripheral session
        r.gotgctl().modify(|w| {
            w.set_bvaloen(!self.config.vbus_detection);
            w.set_bvaloval(true);
        });
    }

    /// Applies configuration specific to
    /// Core ID 0x0000_5000
    pub fn config_v5(&mut self) {
        let r = self.instance.regs;
        let phy_type = self.instance.phy_type;

        if phy_type == PhyType::InternalHighSpeed {
            r.gccfg_v3().modify(|w| {
                w.set_vbvaloven(!self.config.vbus_detection);
                w.set_vbvaloval(!self.config.vbus_detection);
                w.set_vbden(self.config.vbus_detection);
            });
        } else {
            r.gotgctl().modify(|w| {
                w.set_bvaloen(!self.config.vbus_detection);
                w.set_bvaloval(!self.config.vbus_detection);
            });
            r.gccfg_v3().modify(|w| {
                w.set_vbden(self.config.vbus_detection);
            });
        }
    }

    fn init(&mut self) {
        let r = self.instance.regs;
        let phy_type = self.instance.phy_type;

        // Soft disconnect.
        r.dctl().write(|w| w.set_sdis(true));

        // Set speed.
        r.dcfg().write(|w| {
            w.set_pfivl(vals::Pfivl::FRAME_INTERVAL_80);
            w.set_dspd(phy_type.to_dspd());
            if self.config.xcvrdly {
                w.set_xcvrdly(true);
            }
        });

        // USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM |
        //                                                 USB_OTG_DIEPMSK_XFRCM |
        //                                                 USB_OTG_DIEPMSK_EPDM;

        // Unmask transfer complete EP interrupt
        r.diepmsk().write(|w| {
            w.set_xfrcm(true);
            w.set_tom(true);
            w.set_epdm(true);
        });

        // USBx_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |
        //                                                 USB_OTG_DOEPMSK_XFRCM |
        //                                                 USB_OTG_DOEPMSK_EPDM |
        //                                                 USB_OTG_DOEPMSK_OTEPSPRM |
        //                                                 USB_OTG_DOEPMSK_NAKM;

        // Unmask SETUP received EP interrupt
        r.doepmsk().write(|w| {
            // TODO - there are some bits missing in the embassy-stm32 PAC
            w.set_stupm(true);
            w.set_xfrcm(true);
            w.set_epdm(true);
        });

        // Unmask and clear core interrupts
        self.restore_irqs();
        r.gintsts().write_value(regs::Gintsts(0xFFFF_FFFF));

        // Unmask global interrupt
        r.gahbcfg().write(|w| {
            // TODO(goodhoko): make this conditional?
            w.set_dmaen(true);

            // 0000 Single: Bus transactions use single 32 bit accesses (not recommended)
            // 0001 INCR: Bus transactions use unspecified length accesses (not recommended, uses the
            // INCR AHB bus command)
            // 0011 INCR4: Bus transactions target 4x 32 bit accesses
            // 0101 INCR8: Bus transactions target 8x 32 bit accesses
            // 0111 INCR16: Bus transactions based on 16x 32 bit accesses
            // Others: Reserved
            w.set_hbstlen(0b0011);

            // The recommended value for
            // RXTHRLEN is to be the same as the programmed AHB burst length (HBSTLEN bit in
            // OTG_GAHBCFG).
            // TODO(bschwind) - Maybe set RX Threshold Length here
            //                  Also set RXTHREN to true
            //                  Also set NONISOTHREN to true

            // More info for endpoint 0:
            //     Bit 15 STPKTRX: Setup packet received
            // Applicable for control OUT endpoints in only in the Buffer DMA Mode. Set by the OTG_HS,
            // this bit indicates that this buffer holds 8 bytes of setup data. There is only one setup packet
            // per buffer. On receiving a setup packet, the OTG_HS closes the buffer and disables the
            // corresponding endpoint after SETUP_COMPLETE status is seen in the Rx FIFO. OTG_HS
            // puts a SETUP_COMPLETE status into the Rx FIFO when it sees the first IN or OUT token
            // after the SETUP packet for that particular endpoint. The application must then re-enable the
            // endpoint to receive any OUT data for the control transfer and reprogram the buffer start
            // address. Because of the above behavior, OTG_HS can receive any number of back to back
            // setup packets and one buffer for every setup packet is used.

            w.set_gint(true); // unmask global interrupt
        });

        // Connect
        r.dctl().write(|w| w.set_sdis(false));
    }

    fn init_fifo(&mut self) {
        trace!("init_fifo");

        let regs = self.instance.regs;
        // ERRATA NOTE: Don't interrupt FIFOs being written to. The interrupt
        // handler COULD interrupt us here and do FIFO operations, so ensure
        // the interrupt does not occur.
        critical_section::with(|_| {
            // Configure RX fifo size. All endpoints share the same FIFO area.
            let rx_fifo_size_words = self.instance.extra_rx_fifo_words + ep_fifo_size(&self.ep_out);
            trace!("configuring rx fifo size={}", rx_fifo_size_words);

            regs.grxfsiz().modify(|w| w.set_rxfd(rx_fifo_size_words));

            // Configure TX (USB in direction) fifo size for each endpoint
            let mut fifo_top = rx_fifo_size_words;
            for i in 0..self.instance.endpoint_count {
                if let Some(ep) = self.ep_in[i] {
                    trace!(
                        "configuring tx fifo ep={}, offset={}, size={}",
                        i,
                        fifo_top,
                        ep.fifo_size_words
                    );

                    let dieptxf = if i == 0 { regs.dieptxf0() } else { regs.dieptxf(i - 1) };

                    dieptxf.write(|w| {
                        w.set_fd(ep.fifo_size_words);
                        w.set_sa(fifo_top);
                    });

                    fifo_top += ep.fifo_size_words;
                }
            }

            assert!(
                fifo_top <= self.instance.fifo_depth_words,
                "FIFO allocations exceeded maximum capacity"
            );

            // Flush fifos
            regs.grstctl().write(|w| {
                w.set_rxfflsh(true);
                w.set_txfflsh(true);
                w.set_txfnum(0x10);
            });
        });

        loop {
            let x = regs.grstctl().read();
            if !x.rxfflsh() && !x.txfflsh() {
                break;
            }
        }
    }

    fn configure_endpoints(&mut self) {
        trace!("configure_endpoints");

        let regs = self.instance.regs;

        // Configure IN endpoints
        for (index, ep) in self.ep_in.iter().enumerate() {
            if let Some(ep) = ep {
                critical_section::with(|_| {
                    regs.diepctl(index).write(|w| {
                        if index == 0 {
                            w.set_mpsiz(ep0_mpsiz(ep.max_packet_size));
                        } else {
                            w.set_mpsiz(ep.max_packet_size);
                            w.set_eptyp(to_eptyp(ep.ep_type));
                            w.set_sd0pid_sevnfrm(true);
                            w.set_txfnum(index as _);
                            w.set_snak(true);
                        }
                    });
                });
            }
        }

        // Configure OUT endpoints
        for (index, ep) in self.ep_out.iter().enumerate() {
            if let Some(ep) = ep {
                critical_section::with(|_| {
                    regs.doepctl(index).write(|w| {
                        if index == 0 {
                            w.set_mpsiz(ep0_mpsiz(ep.max_packet_size));
                        } else {
                            w.set_mpsiz(ep.max_packet_size);
                            w.set_eptyp(to_eptyp(ep.ep_type));
                            w.set_sd0pid_sevnfrm(true);
                        }
                    });

                    regs.doeptsiz(index).modify(|w| {
                        if index == 0 {
                            // C reference code:
                            // USBx_OUTEP(0U)->DOEPTSIZ = 0U;
                            // USBx_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1UL << 19));
                            // USBx_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
                            // USBx_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT; // 3 setup packets

                            // if (dma == 1U)
                            // {
                            //     USBx_OUTEP(0U)->DOEPDMA = (uint32_t)psetup;
                            //     /* EP enable */
                            //     USBx_OUTEP(0U)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
                            // }

                            w.set_pktcnt(1);
                            // TODO - STM32 code sets this to 24...
                            // w.set_xfrsiz(ep.max_packet_size as _);
                            w.set_xfrsiz(24);
                            w.set_rxdpid_stupcnt(3);
                        } else {
                            // w.set_pktcnt(1);
                            // Expect the user to call read() where we'll set_xfrsiz and set_pktcnt.
                        }
                    });

                    if index == 0 {
                        // TODO(bschwind) - Self should probably be in `Pin<>` so that it doesn't move.
                        regs.doepdma(index)
                            .write_value(self.instance.state.cp_state.setup_data.as_ptr() as u32);

                        // Probably not needed, but enable the endpoint
                        regs.doepctl(0).modify(|w| {
                            w.set_usbaep(true);
                            w.set_epena(true);
                        })
                    }
                });
            }
        }

        // Enable IRQs for allocated endpoints
        regs.daintmsk().modify(|w| {
            w.set_iepm(ep_irq_mask(&self.ep_in));
            w.set_oepm(ep_irq_mask(&self.ep_out));
        });
    }

    fn disable_all_endpoints(&mut self) {
        for i in 0..self.instance.endpoint_count {
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::In), false);
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::Out), false);
        }
    }
}

impl<'d, const MAX_EP_COUNT: usize> embassy_usb_driver::Bus for Bus<'d, MAX_EP_COUNT> {
    async fn poll(&mut self) -> Event {
        poll_fn(move |cx| {
            if !self.inited {
                self.init();
                self.inited = true;

                // If no vbus detection, just return a single PowerDetected event at startup.
                if !self.config.vbus_detection {
                    return Poll::Ready(Event::PowerDetected);
                }
            }

            let regs = self.instance.regs;
            self.instance.state.bus_waker.register(cx.waker());

            let ints = regs.gintsts().read();

            if ints.srqint() {
                trace!("vbus detected");

                regs.gintsts().write(|w| w.set_srqint(true)); // clear
                self.restore_irqs();

                if self.config.vbus_detection {
                    return Poll::Ready(Event::PowerDetected);
                }
            }

            if ints.otgint() {
                let otgints = regs.gotgint().read();
                regs.gotgint().write_value(otgints); // clear all
                self.restore_irqs();

                if otgints.sedet() {
                    trace!("vbus removed");
                    if self.config.vbus_detection {
                        self.disable_all_endpoints();
                        return Poll::Ready(Event::PowerRemoved);
                    }
                }
            }

            if ints.usbrst() {
                trace!("reset");

                self.init_fifo();
                self.configure_endpoints();

                // Reset address
                critical_section::with(|_| {
                    regs.dcfg().modify(|w| {
                        w.set_dad(0);
                    });
                });

                regs.gintsts().write(|w| w.set_usbrst(true)); // clear
                self.restore_irqs();
            }

            if ints.enumdne() {
                trace!("enumdne");

                let speed = regs.dsts().read().enumspd();
                let trdt = (self.instance.calculate_trdt_fn)(speed);
                trace!("  speed={} trdt={}", speed.to_bits(), trdt);
                regs.gusbcfg().modify(|w| w.set_trdt(trdt));

                // Set the MPS of the IN EP0 to 64 bytes
                regs.diepctl(0).modify(|w| {
                    w.set_mpsiz(ep0_mpsiz(64));
                });

                // OTG_DOEPCTL0
                // For USB OTG_HS in DMA mode, program the OTG_DOEPCTL0 register to enable
                // control OUT endpoint 0, to receive a SETUP packet.
                regs.doepctl(0).modify(|w| {
                    w.set_cnak(true);
                    w.set_epena(true);
                });

                regs.dctl().write(|w| {
                    w.set_cginak(true);
                });

                regs.gintsts().write(|w| w.set_enumdne(true)); // clear
                self.restore_irqs();

                // TODO - why do we return a Reset Event here?
                return Poll::Ready(Event::Reset);
            }

            if ints.usbsusp() {
                trace!("suspend");
                regs.gintsts().write(|w| w.set_usbsusp(true)); // clear
                self.restore_irqs();
                return Poll::Ready(Event::Suspend);
            }

            if ints.wkupint() {
                trace!("resume");
                regs.gintsts().write(|w| w.set_wkupint(true)); // clear
                self.restore_irqs();
                return Poll::Ready(Event::Resume);
            }

            Poll::Pending
        })
        .await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        trace!("endpoint_set_stalled ep={:?} en={}", ep_addr, stalled);

        assert!(
            ep_addr.index() < self.instance.endpoint_count,
            "endpoint_set_stalled index {} out of range",
            ep_addr.index()
        );

        let regs = self.instance.regs;
        let state = self.instance.state;
        match ep_addr.direction() {
            Direction::Out => {
                critical_section::with(|_| {
                    regs.doepctl(ep_addr.index()).modify(|w| {
                        w.set_stall(stalled);
                    });
                });

                state.ep_states[ep_addr.index()].out_waker.wake();
            }
            Direction::In => {
                critical_section::with(|_| {
                    regs.diepctl(ep_addr.index()).modify(|w| {
                        w.set_stall(stalled);
                    });
                });

                state.ep_states[ep_addr.index()].in_waker.wake();
            }
        }
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        assert!(
            ep_addr.index() < self.instance.endpoint_count,
            "endpoint_is_stalled index {} out of range",
            ep_addr.index()
        );

        let regs = self.instance.regs;
        match ep_addr.direction() {
            Direction::Out => regs.doepctl(ep_addr.index()).read().stall(),
            Direction::In => regs.diepctl(ep_addr.index()).read().stall(),
        }
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        trace!("endpoint_set_enabled ep={:?} en={}", ep_addr, enabled);

        assert!(
            ep_addr.index() < self.instance.endpoint_count,
            "endpoint_set_enabled index {} out of range",
            ep_addr.index()
        );

        let regs = self.instance.regs;
        let state = self.instance.state;
        match ep_addr.direction() {
            Direction::Out => {
                critical_section::with(|_| {
                    // cancel transfer if active
                    if !enabled && regs.doepctl(ep_addr.index()).read().epena() {
                        regs.doepctl(ep_addr.index()).modify(|w| {
                            w.set_snak(true);
                            w.set_epdis(true);
                        })
                    }

                    regs.doepctl(ep_addr.index()).modify(|w| {
                        w.set_usbaep(enabled);
                    });

                    // TODO(bschwind) - Do we need to flush the TX FIFO here at all?
                    //                  Should we flush the RX FIFO?
                    // Flush tx fifo
                    regs.grstctl().write(|w| {
                        w.set_txfflsh(true);
                        w.set_txfnum(ep_addr.index() as _);
                    });
                    loop {
                        let x = regs.grstctl().read();
                        if !x.txfflsh() {
                            break;
                        }
                    }
                });

                // Wake `Endpoint::wait_enabled()`
                state.ep_states[ep_addr.index()].out_waker.wake();
            }
            Direction::In => {
                critical_section::with(|_| {
                    // cancel transfer if active
                    if !enabled && regs.diepctl(ep_addr.index()).read().epena() {
                        regs.diepctl(ep_addr.index()).modify(|w| {
                            w.set_snak(true); // set NAK
                            w.set_epdis(true);
                        })
                    }

                    regs.diepctl(ep_addr.index()).modify(|w| {
                        w.set_usbaep(enabled);
                        w.set_cnak(enabled); // clear NAK that might've been set by SNAK above.
                    })
                });

                // Wake `Endpoint::wait_enabled()`
                state.ep_states[ep_addr.index()].in_waker.wake();
            }
        }
    }

    async fn enable(&mut self) {
        trace!("enable");
        // TODO: enable the peripheral once enable/disable semantics are cleared up in embassy-usb
    }

    async fn disable(&mut self) {
        trace!("disable");

        // TODO: disable the peripheral once enable/disable semantics are cleared up in embassy-usb
        //Bus::disable(self);
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

/// USB endpoint direction.
trait Dir {
    /// Returns the direction value.
    fn dir() -> Direction;
}

/// Marker type for the "IN" direction.
pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Marker type for the "OUT" direction.
pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

/// USB endpoint.
pub struct Endpoint<'d, D> {
    _phantom: PhantomData<D>,
    regs: Otg,
    info: EndpointInfo,
    state: &'d EpState,
}

impl<'d> embassy_usb_driver::Endpoint for Endpoint<'d, In> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        poll_fn(|cx| {
            let ep_index = self.info.addr.index();

            self.state.in_waker.register(cx.waker());

            if self.regs.diepctl(ep_index).read().usbaep() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d> embassy_usb_driver::Endpoint for Endpoint<'d, Out> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        poll_fn(|cx| {
            let ep_index = self.info.addr.index();

            self.state.out_waker.register(cx.waker());

            if self.regs.doepctl(ep_index).read().usbaep() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d> embassy_usb_driver::EndpointOut for Endpoint<'d, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        if self.state.out_transfer_done.load(Ordering::Acquire) {
            // somehow the teransfer already happened
        }

        trace!("DMA read start len={}", buf.len());

        let index = self.info.addr.index();

        let doepctl = self.regs.doepctl(index).read();
        trace!("DMA read ep={:?}: doepctl {:08x}", self.info.addr, doepctl.0,);
        if !doepctl.usbaep() {
            trace!("DMA read ep={:?} error disabled", self.info.addr);
            return Err(EndpointError::Disabled);
        }

        // 1. Configure the packet count and transfer size to "enable" the EP
        let packet_count =
            ((buf.len() as u32 + self.info.max_packet_size as u32 - 1) / self.info.max_packet_size as u32).max(1);
        self.regs.doeptsiz(index).modify(|w| {
            w.set_xfrsiz(buf.len() as u32);
            w.set_pktcnt(packet_count as u16);
        });

        // 2. Setup DMA
        assert_eq!(
            buf.len() % 4,
            0,
            "buffer must be aligned to DMA word-size which is 4 bytes"
        );
        // Set the destination address of the DMA transfer.
        self.regs.doepdma(index).write_value(buf.as_mut_ptr() as u32);

        info!("marking OUT transfer on EP#{} as NOT done", index);
        self.state.out_transfer_done.store(false, Ordering::Relaxed);
        self.state
            .out_transfer_requested_bytes
            .store(buf.len(), Ordering::Relaxed);

        // Enable the endpoint in case it was disabled.
        self.regs.doepctl(index).modify(|w| {
            w.set_cnak(true);
            w.set_epena(true);
        });

        poll_fn(|cx| {
            self.state.out_waker.register(cx.waker());

            if self.state.out_transfer_done.load(Ordering::Acquire) {
                let bytes_read = self.state.out_transferred_bytes.load(Ordering::Acquire);
                self.state.out_transferred_bytes.store(0, Ordering::Relaxed);

                Poll::Ready(Ok(bytes_read))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d> embassy_usb_driver::EndpointIn for Endpoint<'d, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        trace!("write ep={:?} data={:?}", self.info.addr, Bytes(buf));

        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let index = self.info.addr.index();
        // Wait for previous transfer to complete and check if endpoint is disabled
        poll_fn(|cx| {
            self.state.in_waker.register(cx.waker());

            let diepctl = self.regs.diepctl(index).read();
            let dtxfsts = self.regs.dtxfsts(index).read();
            trace!(
                "write ep={:?}: diepctl {:08x} ftxfsts {:08x}",
                self.info.addr,
                diepctl.0,
                dtxfsts.0
            );
            if !diepctl.usbaep() {
                trace!("write ep={:?} wait for prev: error disabled", self.info.addr);
                Poll::Ready(Err(EndpointError::Disabled))
            } else if !diepctl.epena() {
                trace!("write ep={:?} wait for prev: ready", self.info.addr);
                Poll::Ready(Ok(()))
            } else {
                trace!("write ep={:?} wait for prev: pending", self.info.addr);
                Poll::Pending
            }
        })
        .await?;

        // ERRATA: Transmit data FIFO is corrupted when a write sequence to the FIFO is interrupted with
        // accesses to certain OTG_FS registers.
        //
        // Prevent the interrupt (which might poke FIFOs) from executing while copying data to FIFOs.
        // TODO(goodhoko): do we still need critical section with DMA?
        critical_section::with(|_| {
            // Setup transfer size
            self.regs.dieptsiz(index).write(|w| {
                w.set_mcnt(1);
                w.set_pktcnt(1);
                w.set_xfrsiz(buf.len() as _);
            });

            if self.info.ep_type == EndpointType::Isochronous {
                // Isochronous endpoints must set the correct even/odd frame bit to
                // correspond with the next frame's number.
                let frame_number = self.regs.dsts().read().fnsof();
                let frame_is_odd = frame_number & 0x01 == 1;

                self.regs.diepctl(index).modify(|r| {
                    if frame_is_odd {
                        r.set_sd0pid_sevnfrm(true);
                    } else {
                        r.set_sd1pid_soddfrm(true);
                    }
                });
            }

            // Set the DMA address here
            self.regs.diepdma(index).write_value(buf.as_ptr() as u32);

            self.state.in_transfer_done.store(false, Ordering::Relaxed);
            // Enable endpoint
            self.regs.diepctl(index).modify(|w| {
                w.set_cnak(true);
                w.set_epena(true);
            });
        });

        poll_fn(|cx| {
            self.state.in_waker.register(cx.waker());

            if self.state.in_transfer_done.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        trace!("write done ep={:?}", self.info.addr);

        Ok(())
    }
}

/// USB control pipe.
pub struct ControlPipe<'d> {
    max_packet_size: u16,
    regs: Otg,
    setup_state: &'d ControlPipeSetupState,
    ep_in: Endpoint<'d, In>,
    ep_out: Endpoint<'d, Out>,
}

impl<'d> embassy_usb_driver::ControlPipe for ControlPipe<'d> {
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        // Clear NAK to indicate we are ready to receive more data
        // Re-enable the control 0 endpoint

        self.regs
            .doepdma(self.ep_out.info.addr.index())
            .write_value(self.setup_state.setup_data.as_ptr() as u32);

        self.setup_state.awaiting_setup_packet.store(true, Ordering::Release);
        self.regs.doepctl(self.ep_out.info.addr.index()).modify(|w| {
            w.set_cnak(true);
            w.set_usbaep(true);
            w.set_epena(true);
        });

        poll_fn(|cx| {
            self.ep_out.state.out_waker.register(cx.waker());

            if !self.setup_state.awaiting_setup_packet.load(Ordering::Relaxed) {
                let mut data = [0; 8];
                data[0..4].copy_from_slice(&self.setup_state.setup_data[0].to_ne_bytes());
                data[4..8].copy_from_slice(&self.setup_state.setup_data[1].to_ne_bytes());

                // EP0 should not be controlled by `Bus` so this RMW does not need a critical section
                self.regs.doeptsiz(self.ep_out.info.addr.index()).modify(|w| {
                    w.set_rxdpid_stupcnt(3);
                });

                info!("SETUP received: {:?}", Bytes(&data));
                Poll::Ready(data)
            } else {
                info!("SETUP waiting");
                Poll::Pending
            }
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        trace!("control: data_out");
        let len = self.ep_out.read(buf).await?;
        trace!("control: data_out read: {:?}", Bytes(&buf[..len]));
        Ok(len)
    }

    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        trace!("control: data_in write: {:?}", Bytes(data));
        self.ep_in.write(data).await?;

        // wait for status response from host after sending the last packet
        if last {
            trace!("control: data_in waiting for status");
            self.ep_out.read(&mut []).await?;
            trace!("control: complete");
        }

        Ok(())
    }

    async fn accept(&mut self) {
        trace!("control: accept");

        self.ep_in.write(&[]).await.ok();

        trace!("control: accept OK");
    }

    async fn reject(&mut self) {
        trace!("control: reject");

        // EP0 should not be controlled by `Bus` so this RMW does not need a critical section
        self.regs.diepctl(self.ep_in.info.addr.index()).modify(|w| {
            w.set_stall(true);
        });
        self.regs.doepctl(self.ep_out.info.addr.index()).modify(|w| {
            w.set_stall(true);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        trace!("setting addr: {}", addr);
        critical_section::with(|_| {
            self.regs.dcfg().modify(|w| {
                w.set_dad(addr);
            });
        });

        // synopsys driver requires accept to be sent after changing address
        self.accept().await
    }
}

/// Translates HAL [EndpointType] into PAC [vals::Eptyp]
fn to_eptyp(ep_type: EndpointType) -> vals::Eptyp {
    match ep_type {
        EndpointType::Control => vals::Eptyp::CONTROL,
        EndpointType::Isochronous => vals::Eptyp::ISOCHRONOUS,
        EndpointType::Bulk => vals::Eptyp::BULK,
        EndpointType::Interrupt => vals::Eptyp::INTERRUPT,
    }
}

/// Calculates total allocated FIFO size in words
fn ep_fifo_size(eps: &[Option<EndpointData>]) -> u16 {
    eps.iter().map(|ep| ep.map(|ep| ep.fifo_size_words).unwrap_or(0)).sum()
}

/// Generates IRQ mask for enabled endpoints
fn ep_irq_mask(eps: &[Option<EndpointData>]) -> u16 {
    eps.iter().enumerate().fold(
        0,
        |mask, (index, ep)| {
            if ep.is_some() {
                mask | (1 << index)
            } else {
                mask
            }
        },
    )
}

/// Calculates MPSIZ value for EP0, which uses special values.
fn ep0_mpsiz(max_packet_size: u16) -> u16 {
    match max_packet_size {
        8 => 0b11,
        16 => 0b10,
        32 => 0b01,
        64 => 0b00,
        other => panic!("Unsupported EP0 size: {}", other),
    }
}

/// Hardware-dependent USB IP configuration.
pub struct OtgInstance<'d, const MAX_EP_COUNT: usize> {
    /// The USB peripheral.
    pub regs: Otg,
    /// The USB state.
    pub state: &'d State<MAX_EP_COUNT>,
    /// FIFO depth in words.
    pub fifo_depth_words: u16,
    /// Number of used endpoints.
    pub endpoint_count: usize,
    /// The PHY type.
    pub phy_type: PhyType,
    /// Extra RX FIFO words needed by some implementations.
    pub extra_rx_fifo_words: u16,
    /// Function to calculate TRDT value based on some internal clock speed.
    pub calculate_trdt_fn: fn(speed: vals::Dspd) -> u8,
}

fn u32_to_u8(arr: &[u32]) -> &[u8] {
    let len = 4 * arr.len();
    let ptr = arr.as_ptr() as *const u8;
    unsafe { core::slice::from_raw_parts(ptr, len) }
}
