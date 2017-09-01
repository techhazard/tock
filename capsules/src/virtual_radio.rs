//! Virtualizes the 802.15.4 sending interface to multiple clients.

// System call interface for sending and receiving 802.15.4 packets.
//
// Author: Philip Levis
// Date: Jan 12 2017
//

use core::cell::Cell;
use core::cmp;
use kernel::ReturnCode;
use kernel::common::take_cell::{MapCell, TakeCell};
use kernel::common::virtualizer::{QueuedCall,CallQueue, Dequeued};
use mac;
use net::ieee802154::{MacAddress, Header};

pub struct RadioMux<'a, M: mac::Mac + 'a> {
    mac: &'a M,
    queue: CallQueue<'a>
}

pub struct VirtualRadioDevice<'a, M: mac::Mac + 'a> {
    tx_buffer: Option<&'a mac::Frame>,
    queued_call: QueuedCall<'a>,
    mux: &'a RadioMux<'a, M>,
}

impl<'a, M: mac::Mac> RadioMux<'a, M> {
    pub fn new(mac: &'a M) -> RadioMux<'a, M> {
        RadioMux {
            mac: mac,
            queue: CallQueue::new()
        }
    }
}

impl <'a, M: mac::Mac> VirtualRadioDevice<'a, M> {
    pub fn new(mux: &'a RadioMux<'a, M>) -> VirtualRadioDevice<'a, M> {
        let mut v = VirtualRadioDevice {
            tx_buffer: None,
            queued_call: QueuedCall::new(&mux.queue),
            mux: mux,
        };
        v.queued_call.set_callback(&v);
        return v
    }
    

}

impl <'a, M: mac::Mac> Dequeued for VirtualRadioDevice<'a, M> {
    fn id(&self) -> u32 {
      0 
    }
    fn dequeued(&self) {
        //print!("Dequeued\n");
    }
}
