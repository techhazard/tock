//! X-MAC protocol layer for low power 802.15.4 reception, intended primarily
//! to manage an Atmel RF233 radio.
//!
//! TODO: Test no-preamble transmission with randomized backoff, requires 3
//!       devices.
//! TODO: Modifying sleep time with traffic load to optimize energy usage.
//! TODO: Remove expectation that radios cancel pending sleeps when receiving a
//!       new packet (see line 652).
//!
//! Original X-MAC paper, on which this implementation is heavily based:
//!     http://www.cs.cmu.edu/~andersoe/papers/xmac-sensys.pdf
//!
//! Nodes using this layer place their radios to sleep for the vast majority of
//! the time, thereby reducing power consumption. Transmitters wake and send a
//! stream of small, strobed `preamble` packets to the desired recipient. If a
//! receiver wakes and ACKS a relevant preamble, the receiver waits for a data
//! packet before returning to sleep. See comments below for implementation
//! details.
//!
//! Additional notes:
//!
//!   * Since much of a node's time is spent sleeping, transmission latency is
//!     much higher than using a radio that is always powered on.
//!   * ReturnCode::ENOACKs may be generated when transmitting, if the
//!     destination node cannot acknowledge within the maximum retry interval.
//!   * Since X-MAC relies on proper sleep/wake behavior for all nodes, any
//!     node with this implementation will not be able to communicate correctly
//!     with non-XMAC-wrapped radios.
//!
//! Usage
//! -----
//! This capsule implements the `kernel::hil::radio::Radio` interface while
//! wrapping an actual radio driver with the same interface, and can be used
//! as the backend for a `capsules::ieee802154::mac::MacDevice`. Given a radio
//! driver `RF233Device` and `kernel::hil::time::Alarm` and
//! `kernel::hil::rng::RNG` implementations, the necessary modifications to a
//! board configuration are shown below (for `imix`):
//!
//! ```rust
//! use capsules::xmac::XMAC;
//! type XMACDevice = capsules::xmac::XMAC<'static, RF233Device, Alarm>;
//! // ...
//! // XMAC needs two buffers in addition to those provided to the RF233 driver.
//! //   1. stores actual packet contents to free the SPI buffers used by the
//! //      radio for transmitting preamble packets
//! //   2. buffer to receive packets into before passing them up the stack
//! static mut XMAC_BUF: [u8; radio::MAX_BUF_SIZE] = [0x00; radio::MAX_BUF_SIZE];
//! static mut XMAC_RX_BUF: [u8; radio::MAX_BUF_SIZE] = [0x00; radio::MAX_BUF_SIZE];
//! // ...
//! let xmac: &XMACDevice = static_init!(XMACDevice, XMAC::new(rf233, alarm, rng));
//! rng.set_client(xmac);
//!
//! // Hook up the radio to the XMAC implementation. We can now treat `xmac` as
//! // the radio driver.
//! rf233.set_transmit_client(xmac);
//! rf233.set_receive_client(xmac, &mut RF233_RX_BUF);
//! rf233.set_power_client(xmac);
//!
//! xmac.initialize(&mut RF233_BUF, &mut RF233_REG_WRITE, &mut RF233_REG_READ);
//! xmac.initialize_xmac(&mut XMAC_BUF);
//!
//! // We can now use the XMAC driver to instantiate a MacDevice
//! let xmac_mac = static_init!(
//!     capsules::ieee802154::mac::MacDevice<'static, XMACDevice>,
//!     capsules::ieee802154::mac::MacDevice::new(xmac));
//! xmac.set_transmit_client(xmac_mac);
//! xmac.set_receive_client(xmac_mac, &mut XMAC_RX_BUF);
//! xmac.set_config_client(xmac_mac);
//! ```
//
// Author: Jean-Luc Watson
// Date: Nov 21 2017
//

use core::cell::Cell;
use kernel::ReturnCode;
use kernel::common::take_cell::TakeCell;
use kernel::hil::radio;
use kernel::hil::rng::{self, RNG};
use kernel::hil::spi;
use kernel::hil::time::{self, Alarm, Time, Frequency};
use net::ieee802154::*;

// Time the radio will remain awake listening for packets before sleeping.
// Observing the RF233, receive callbacks for preambles are generated only after
// having been awake for more than 4-6 ms; 10 ms is a safe amount of time where
// we are very likely to pick up any incoming preambles, and is half as much
// as the 20 ms lower bound in Buettner et al.
const WAKE_TIME_MS: u32 = 10;
// Time the radio will sleep between wakes. Configurable to any desired value
// less than or equal to the max time the transmitter sends preambles before
// abandoning the transmission.
const SLEEP_TIME_MS: u32 = 250;
// Time the radio will continue to send preamble packets before aborting the
// transmission and returning ENOACK. Should be at least as large as the maximum
// sleep time for any node in the network.
const PREAMBLE_TX_MS: u32 = 250;

// Maximum backoff for a transmitter attempting to send a data packet, when the
// node has detected a data packet sent to the same destination from another
// transmitter. This is an optimization that eliminates the need for any
// preambles when the receiving node is shown to already be awake.
const MAX_TX_BACKOFF_MS: u32 = 10;
// After receiving a data packet, maximum time a node will stay awake to receive
// any additional incoming packets before going to sleep.
const MAX_RX_SLEEP_DELAY_MS: u32 = MAX_TX_BACKOFF_MS;

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, PartialEq)]
enum XMACState {
    // The primary purpose of these states is to manage the timer that runs the
    // protocol and determines the state of the radio (e.g. if in SLEEP, a fired
    // timer indicates we should transition to AWAKE).
    AWAKE, // Awake and listening for incoming preambles
    DELAY_SLEEP, // Receiving done; waiting for any other incoming data packets
    SLEEP, // Asleep and not receiving or transmitting
    STARTUP, // Radio waking up, PowerClient::on() transitions to next state
    TX_PREAMBLE, // Transmitting preambles and waiting for an ACK
    TX, // Transmitting data packet to the destination node
    TX_DELAY, // Backing off to send data directly without preamble
}

// Information extracted for each packet from the data buffer provided to
// transmit(), used to generate preamble packets and detect when a delayed
// direct transmission (described above) is appropriate.
#[derive(Copy, Clone, Eq, PartialEq)]
pub struct XMACHeaderInfo {
    pub dst_pan: Option<PanID>,
    pub dst_addr: Option<MacAddress>,
    pub src_pan: Option<PanID>,
    pub src_addr: Option<MacAddress>,
}

// The X-MAC `driver` consists primarily of a backend radio driver, an alarm for
// transitioning between different portions of the protocol, and a source of
// randomness for transmit backoffs. In addition, we maintain two packet buffers
// (one for transmit, one for receive) that cycle without copying between XMAC,
// the tx/rx client, and the underlying radio driver. The transmit buffer can
// also hold the actual data packet contents while preambles are being
// transmitted.
pub struct XMAC<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> {
    radio: &'a R,
    alarm: &'a A,
    rng: &'a RNG,
    tx_client: Cell<Option<&'static radio::TxClient>>,
    rx_client: Cell<Option<&'static radio::RxClient>>,
    state: Cell<XMACState>,
    delay_sleep: Cell<bool>,

    tx_header: Cell<Option<XMACHeaderInfo>>,
    tx_payload: TakeCell<'static, [u8]>,
    tx_len: Cell<usize>,

    tx_preamble_pending: Cell<bool>,
    tx_preamble_seq_num: Cell<u8>,
    tx_preamble_buf: TakeCell<'static, [u8]>,

    rx_buf: TakeCell<'static, [u8]>,
    rx_pending: Cell<bool>,
}

impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> XMAC<'a, R, A> {
    pub fn new(radio: &'a R, alarm: &'a A, rng: &'a RNG) -> XMAC<'a, R, A> {
        XMAC {
            radio: radio,
            alarm: alarm,
            rng: rng,
            tx_client: Cell::new(None),
            rx_client: Cell::new(None),
            state: Cell::new(XMACState::STARTUP),
            delay_sleep: Cell::new(false),
            tx_header: Cell::new(None),
            tx_payload: TakeCell::empty(),
            tx_len: Cell::new(0),
            tx_preamble_pending: Cell::new(false),
            tx_preamble_seq_num: Cell::new(0),
            tx_preamble_buf: TakeCell::empty(),
            rx_buf: TakeCell::empty(),
            rx_pending: Cell::new(false),
        }
    }

    // Should be called directly after kernel::hil::Radio::initialize(...). Sets
    // up needed transmit buffer and, since the radio is starting up, sets the
    // startup timer.
    pub fn initialize_xmac(&self, preamble_buf: &'static mut [u8]) {
        self.tx_preamble_buf.replace(preamble_buf);
        self.state.set(XMACState::STARTUP);
    }

    fn sleep_time(&self) -> u32 {
        // TODO (ongoing) modify based on traffic load to efficiently schedule
        // sleep. Currently sleeps for a constant amount of time.
        SLEEP_TIME_MS
    }

    fn sleep(&self) {
        // If transmitting/delaying sleep, we don't want to try to sleep (again)
        if self.state.get() == XMACState::AWAKE {
            // If we should delay sleep (completed RX), set timer accordingly
            if self.delay_sleep.get() {
                self.state.set(XMACState::DELAY_SLEEP);
                self.set_timer_ms::<A>(MAX_RX_SLEEP_DELAY_MS);

                // Otherwise, don't sleep if expecting a data packet or transmitting
            } else if !self.rx_pending.get() {
                self.radio.stop();
                self.state.set(XMACState::SLEEP);
                self.set_timer_ms::<A>(self.sleep_time());
            }
        }
    }

    // Sets the timer to fire a set number of milliseconds in the future based
    // on the current tick value.
    fn set_timer_ms<T: Time>(&self, ms: u32) {
        self.alarm.set_alarm(self.alarm
            .now()
            .wrapping_add(((ms as f32 / 1000.0) * <T::Frequency>::frequency() as f32) as u32));
    }

    fn transmit_preamble(&self) {

        let mut result: (ReturnCode, Option<&'static mut [u8]>) = (ReturnCode::SUCCESS, None);
        let buf = self.tx_preamble_buf.take().unwrap();
        let tx_header = self.tx_header.get().unwrap();

        // If we're not currently sending preambles, skip transmission
        if let XMACState::TX_PREAMBLE = self.state.get() {

            // Generate preamble frame. We use a reserved frame type (0b101) to
            // distinguish from regular data frames, increment a sequence
            // number for each consecutive packet sent, and send with no
            // security.
            let header = Header {
                frame_type: FrameType::Multipurpose,
                frame_pending: false,
                ack_requested: true,
                version: FrameVersion::V2015,
                seq: Some(self.tx_preamble_seq_num.get()),
                dst_pan: tx_header.dst_pan,
                dst_addr: tx_header.dst_addr,
                src_pan: tx_header.src_pan,
                src_addr: tx_header.src_addr,
                security: None,
                header_ies: Default::default(),
                header_ies_len: 0,
                payload_ies: Default::default(),
                payload_ies_len: 0,
            };

            self.tx_preamble_seq_num.set(self.tx_preamble_seq_num.get() + 1);

            match header.encode(&mut buf[radio::PSDU_OFFSET..], true).done() {
                // If we can successfully encode the preamble, transmit.
                Some((data_offset, _)) => {
                    result = self.radio.transmit(buf, data_offset + radio::PSDU_OFFSET);
                }
                None => {
                    self.tx_preamble_buf.replace(buf);
                    self.call_tx_client(self.tx_payload.take().unwrap(), false, ReturnCode::FAIL);
                    return;
                }
            }
        }

        // If the transmission fails, callback directly back into the client
        if result.0 != ReturnCode::SUCCESS {
            self.call_tx_client(result.1.unwrap(), false, result.0);
        }
    }

    fn transmit_packet(&self) {

        // If we have actual data to transmit, send it and report errors to
        // client.
        if self.tx_payload.is_some() {

            let result: (ReturnCode, Option<&'static mut [u8]>);
            let tx_buf = self.tx_payload.take().unwrap();

            result = self.radio.transmit(tx_buf, self.tx_len.get());

            if result.0 != ReturnCode::SUCCESS {
                self.call_tx_client(result.1.unwrap(), false, result.0);
            }
        }
    }

    // Reports back to client that transmission is complete, radio can turn off
    // if not kept awake by other portions of the protocol.
    fn call_tx_client(&self, buf: &'static mut [u8], acked: bool, result: ReturnCode) {
        self.state.set(XMACState::AWAKE);
        self.sleep();
        self.tx_client
            .get()
            .map(move |c| { c.send_done(buf, acked, result); });
    }

    // Reports any received packet back to the client and starts going to sleep.
    // Does not propagate preamble packets up to the RxClient.
    fn call_rx_client(&self,
                      buf: &'static mut [u8],
                      len: usize,
                      crc_valid: bool,
                      result: ReturnCode) {
        self.delay_sleep.set(true);
        self.sleep();

        self.rx_client
            .get()
            .map(move |c| { c.receive(buf, len, crc_valid, result); });
    }
}

impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> rng::Client
    for XMAC<'a, R, A> {
    fn randomness_available(&self, randomness: &mut Iterator<Item = u32>) -> rng::Continue {
        match randomness.next() {
            Some(random) => {
                if self.state.get() == XMACState::TX_DELAY {
                    // When another data packet to our desired destination is
                    // detected, we backoff a random amount before sending our
                    // own data with no preamble. This assumes that the reciever
                    // will remain awake long enough to receive our transmission,
                    // as it should with this implementation. Since RNG is
                    // asynchronous, we account for the time spent waiting for
                    // the callback and randomly determine the remaining time
                    // spent backing off.
                    let time_remaining_ms =
                        (((self.alarm.get_alarm().wrapping_sub(self.alarm.now())) as f32 /
                          <A::Frequency>::frequency() as f32) *
                         1000.0) as u32;
                    self.set_timer_ms::<A>(random % time_remaining_ms);
                }
                rng::Continue::Done
            }
            None => rng::Continue::More,
        }
    }
}

impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> radio::Radio
    for XMAC<'a, R, A> {
}

// The vast majority of these calls pass through to the underlying radio driver.
impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm> radio::RadioConfig
    for XMAC<'a, R, A> {
    fn initialize(&self,
                  spi_buf: &'static mut [u8],
                  reg_write: &'static mut [u8],
                  reg_read: &'static mut [u8])
                  -> ReturnCode {
        self.radio.start();
        self.radio.initialize(spi_buf, reg_write, reg_read)
    }

    fn reset(&self) -> ReturnCode {
        self.radio.reset()
    }

    fn start(&self) -> ReturnCode {
        if !self.radio.is_on() {
            self.state.set(XMACState::STARTUP);
            self.radio.start();
            ReturnCode::SUCCESS
        } else {
            ReturnCode::EALREADY
        }
    }

    fn stop(&self) -> ReturnCode {
        if self.state.get() == XMACState::SLEEP {
            ReturnCode::EALREADY
        } else {
            self.sleep();
            ReturnCode::SUCCESS
        }
    }

    // Always lie and say the radio is on when sleeping, as XMAC will wake up
    // itself to send preambles if necessary.
    fn is_on(&self) -> bool {
        if self.state.get() == XMACState::SLEEP {
            return true;
        }
        self.radio.is_on()
    }

    fn busy(&self) -> bool {
        self.radio.busy() || self.state.get() == XMACState::TX_PREAMBLE ||
        self.state.get() == XMACState::TX
    }

    fn set_config_client(&self, client: &'static radio::ConfigClient) {
        self.radio.set_config_client(client)
    }

    fn set_power_client(&self, client: &'static radio::PowerClient) {
        self.radio.set_power_client(client)
    }

    fn set_address(&self, addr: u16) {
        self.radio.set_address(addr)
    }

    fn set_address_long(&self, addr: [u8; 8]) {
        self.radio.set_address_long(addr)
    }

    fn set_pan(&self, id: u16) {
        self.radio.set_pan(id)
    }

    fn set_tx_power(&self, power: i8) -> ReturnCode {
        self.radio.set_tx_power(power)
    }

    fn set_channel(&self, chan: u8) -> ReturnCode {
        self.radio.set_channel(chan)
    }

    fn get_address(&self) -> u16 {
        self.radio.get_address()
    }

    fn get_address_long(&self) -> [u8; 8] {
        self.radio.get_address_long()
    }

    fn get_pan(&self) -> u16 {
        self.radio.get_pan()
    }

    fn get_tx_power(&self) -> i8 {
        self.radio.get_tx_power()
    }

    fn get_channel(&self) -> u8 {
        self.radio.get_channel()
    }

    fn config_commit(&self) {
        self.radio.config_commit()
    }
}

impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> radio::RadioData
    for
    XMAC<'a, R, A> {
    fn set_transmit_client(&self, client: &'static radio::TxClient) {
        self.tx_client.set(Some(client));
    }

    fn set_receive_client(&self, client: &'static radio::RxClient, buffer: &'static mut [u8]) {
        self.rx_client.set(Some(client));
        self.rx_buf.replace(buffer);
    }

    fn set_receive_buffer(&self, buffer: &'static mut [u8]) {
        self.rx_buf.replace(buffer);
    }

    fn transmit(&self,
                spi_buf: &'static mut [u8],
                frame_len: usize)
                -> (ReturnCode, Option<&'static mut [u8]>) {

        // If the radio is busy, we already have data to transmit, or the buffer
        // size is wrong, fail before attempting to send any preamble packets
        // (and waking up the radio).
        let frame_len = frame_len + radio::MFR_SIZE;
        if self.radio.busy() || self.tx_payload.is_some() {
            return (ReturnCode::EBUSY, Some(spi_buf));
        } else if radio::PSDU_OFFSET + frame_len >= spi_buf.len() {
            return (ReturnCode::ESIZE, Some(spi_buf));
        }


        match Header::decode(&spi_buf[radio::PSDU_OFFSET..], false).done() {
            Some((_, (header, _))) => {
                self.tx_len.set(frame_len - radio::PSDU_OFFSET);
                self.tx_header.set(Some(XMACHeaderInfo {
                    dst_addr: header.dst_addr,
                    dst_pan: header.dst_pan,
                    src_addr: header.src_addr,
                    src_pan: header.src_pan,
                }));
            }
            None => {
                self.tx_header.set(None);
            }
        }

        match self.tx_header.get() {
            Some(_) => {
                self.tx_payload.replace(spi_buf);
            }
            None => {
                return (ReturnCode::FAIL, Some(spi_buf));
            }
        }

        self.tx_preamble_seq_num.set(0);

        // If the radio is on, start the preamble timer and start transmitting
        if self.radio.is_on() {
            self.state.set(XMACState::TX_PREAMBLE);
            self.set_timer_ms::<A>(PREAMBLE_TX_MS);
            self.transmit_preamble();

            // If the radio is currently sleeping, wake it and indicate that when
            // ready, it should begin transmitting preambles
        } else {
            self.state.set(XMACState::STARTUP);
            self.tx_preamble_pending.set(true);
            self.radio.start();
        }

        (ReturnCode::SUCCESS, None)
    }
}

impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> spi::SpiMasterClient
    for
    XMAC<'a, R, A> {
    fn read_write_done(&self,
                       mut _write: &'static mut [u8],
                       read: Option<&'static mut [u8]>,
                       _len: usize) {
        self.radio.read_write_done(_write, read, _len)
    }
}

// Core of the XMAC protocol - when the timer fires, the protocol state
// indicates the next state/action to take.
impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> time::Client
    for XMAC<'a, R, A> {
    fn fired(&self) {
        match self.state.get() {
            XMACState::SLEEP => {
                // If asleep, start the radio and wait for the PowerClient to
                // indicate that the radio is ready
                if !self.radio.is_on() {
                    self.state.set(XMACState::STARTUP);
                    self.radio.start();
                } else {
                    self.state.set(XMACState::AWAKE);
                }
            }
            // If we've been delaying sleep or haven't heard any incoming
            // preambles, turn the radio off.
            XMACState::AWAKE => {
                self.sleep();
            }
            XMACState::DELAY_SLEEP => {
                self.delay_sleep.set(false);
                self.state.set(XMACState::AWAKE);
                self.sleep();
            }
            // If we've sent preambles for longer than the maximum sleep time of
            // any node in the network, then our destination is non-responsive;
            // return ENOACK to the client.
            XMACState::TX_PREAMBLE => {
                self.call_tx_client(self.tx_payload.take().unwrap(), false, ReturnCode::ENOACK);
            }
            // After a randomized backoff period, transmit the data directly.
            XMACState::TX_DELAY => {
                self.state.set(XMACState::TX);
                self.transmit_packet();
            }
            _ => {}
        }
    }
}

impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> radio::PowerClient
    for
    XMAC<'a, R, A> {
    fn changed(&self, on: bool) {
        // If the radio turns on and we're in STARTUP, then either transition to
        // listening for incoming preambles or start transmitting preambles if
        // the radio was turned on for a transmission.
        if on {
            if let XMACState::STARTUP = self.state.get() {
                if self.tx_preamble_pending.get() {
                    self.state.set(XMACState::TX_PREAMBLE);
                    self.set_timer_ms::<A>(PREAMBLE_TX_MS);
                    self.transmit_preamble();
                } else {
                    self.state.set(XMACState::AWAKE);
                    self.set_timer_ms::<A>(WAKE_TIME_MS);
                }
            }
        }
    }
}

impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> radio::TxClient
    for XMAC<'a, R, A> {
    fn send_done(&self, buf: &'static mut [u8], acked: bool, result: ReturnCode) {
        match self.state.get() {
            // Completed a data transmission to the destination node
            XMACState::TX => {
                self.call_tx_client(buf, acked, result);
            }
            // Completed a preamble transmission
            XMACState::TX_PREAMBLE => {
                self.tx_preamble_buf.replace(buf);
                if acked {
                    // Destination signals ready to receive data
                    self.state.set(XMACState::TX);
                    self.transmit_packet();
                } else {
                    // Continue resending preambles
                    self.transmit_preamble();
                }
            }
            XMACState::TX_DELAY |
            XMACState::SLEEP => {
                // If, while sending preambles, we switch to TX_DELAY mode, the
                // last preamble sent will complete afterwards. If no ACK, the
                // radio may have fallen sleep before the callback is processed.
                self.tx_preamble_buf.replace(buf);
            }
            _ => {}
        }
    }
}

// The receive callback is complicated by the fact that, to determine when a
// destination node is receiving packets/awake while we are attempting a
// transmission, we put the radio in promiscuous mode. Not a huge issue, but
// we need to be wary of incoming packets not actually addressed to our node.
impl<'a, R: radio::Radio + spi::SpiMasterClient + 'a, A: Alarm + 'a> radio::RxClient
    for XMAC<'a, R, A> {
    fn receive(&self,
               buf: &'static mut [u8],
               frame_len: usize,
               crc_valid: bool,
               result: ReturnCode) {

        let mut data_received: bool = false;
        let mut continue_sleep: bool = true;

        // First, check to make sure we can decode the MAC header (especially
        // the destination address) to see if we can backoff/send pending
        // transmission.
        if let Some((_, (header, _))) = Header::decode(&buf[radio::PSDU_OFFSET..], false).done() {
            if let Some(dst_addr) = header.dst_addr {

                let addr_match = match dst_addr {
                    MacAddress::Short(addr) => addr == self.radio.get_address(),
                    MacAddress::Long(long_addr) => long_addr == self.radio.get_address_long(),
                };
                // The destination doesn't match our address, check to see if we
                // can backoff a pending transmission if it exists rather than
                // continue sending preambles.
                if !addr_match {
                    if self.state.get() == XMACState::TX_PREAMBLE {
                        if let Some(tx_dst_addr) =
                            self.tx_header.get().and_then(|hdr| hdr.dst_addr) {
                            if tx_dst_addr == dst_addr {
                                // Randomize backoff - since the callback is asynchronous, set the
                                // timer for the max and adjust later. As a result, we can't
                                // backoff for more than the RNG generation time.
                                self.state.set(XMACState::TX_DELAY);
                                self.rng.get();
                                self.set_timer_ms::<A>(MAX_TX_BACKOFF_MS);
                                continue_sleep = false;
                            }
                        }
                    }
                } else {
                    // We've received either a preamble or data packet
                    match header.frame_type {
                        FrameType::Multipurpose => {
                            continue_sleep = false;
                            self.rx_pending.set(true);
                        }
                        FrameType::Data => {
                            continue_sleep = false;
                            data_received = true;
                        }
                        _ => {}
                    }
                }
            }
        }

        // TODO: this currently assumes that upon receiving a packet, the radio
        // will cancel a pending sleep, and an additional call to Radio::stop()
        // is required to shut down the radio. This works specifically for the
        // RF233 with the added line at rf233.rs:744. In progress: it might be
        // possible to remove this requirement.
        if self.state.get() == XMACState::SLEEP {
            self.state.set(XMACState::AWAKE);
        }

        if data_received {
            self.radio.set_receive_buffer(self.rx_buf.take().unwrap());
            self.rx_pending.set(false);
            self.call_rx_client(buf, frame_len, crc_valid, result);
        } else {
            self.radio.set_receive_buffer(buf);
        }

        // If we should go to sleep (i.e. not waiting up for any additional data
        // packets), shut the radio down. If a prior sleep was pending, it was
        // cancelled as the result of the RX (see above).
        if continue_sleep {
            self.sleep();
        }
    }
}
