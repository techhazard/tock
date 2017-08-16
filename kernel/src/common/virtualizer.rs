//! Provide a queue-based callback for virtualizing OS abstractions.

use core::cell::Cell;
use common::list::{List, ListLink, ListNode};

pub trait Dequeued {
    fn dequeued(&self);
    fn id(&self) -> u32;
}

pub struct QueuedCall<'a> {
    next: ListLink<'a, QueuedCall<'a>>,
    callback: Cell<Option<&'a Dequeued>>,
    active: Cell<bool>,
    queue: &'a CallQueue<'a>,
}

impl<'a> ListNode<'a, QueuedCall<'a>> for QueuedCall<'a> {
    fn next(&self) -> &'a ListLink<QueuedCall<'a>> {
        &self.next
    }
}

impl<'a> QueuedCall<'a> {
    pub fn new(queue: &'a CallQueue<'a>) -> QueuedCall<'a> {
        QueuedCall {
            next: ListLink::empty(),
            callback: Cell::new(None),
            active: Cell::new(false),
            queue: queue,
        }
    }

    pub fn set_callback(&'a self, callback: &'a Dequeued) {
        self.callback.set(Some(callback));
        self.queue.queued_calls.push_head(self);
    }

    pub fn set_active(&'a self) {
        self.active.set(true);
    }

    pub fn set_inactive(&'a self) {
        self.active.set(false);
    }
}

pub struct CallQueue<'a> {
    queued_calls: List<'a, QueuedCall<'a>>,
    next: Cell<Option<&'a QueuedCall<'a>>>,
}

impl<'a> CallQueue<'a> {
    pub const fn new() -> CallQueue<'a> {
        CallQueue {
            queued_calls: List::new(),
            next: Cell::new(None),
        }
    }

    // This triggers the next queued element by performing
    // a linear scan of the list, starting at the front. It
    // keeps a reference to the 'next' element after the last one
    // triggered (or None, if it was the tail of the queue).
    // It keeps track of the 'last' active element before
    // the 'next' element. It then walks forward from next,
    // triggering the first active element. If it reaches the
    // end of the queue, it triggers 'last' if there is one,
    // or returns false if there is no 'last' (no element in the
    // queue was marked active).
    pub fn dequeue_and_trigger(&self) -> bool {
        // If the last scan reached the end of the queue,
        // set the first element to look at to be the first element
        // of the queue (could be None if queue is empty).
        if self.next.get().is_none() {
            self.next.set(self.queued_calls.head());
        }
        let mut next = false; // Do we need to set next element?
        let mut passed = false; // Have we passed next element?
        let mut last = self.queued_calls.head(); // Last element 
        for call in self.queued_calls.iter() {
            if !passed {
                if call as *const QueuedCall == self.next.get().unwrap() as *const QueuedCall {
                    passed = true;
                    self.next.set(None);
                    if call.active.get() {
                       next = true;
                       call.active.set(false);
                       call.callback.get().map(|c| c.dequeued());
                    }
                } else if call.active.get() {
                    last = Some(call);
                }
            }
            else if next {
                self.next.set(Some(call));
                return true;
            } else if call.active.get() {
                call.active.set(false);
                call.callback.get().map(|c| c.dequeued());
                next = true;
                self.next.set(None);
            }
        }
        if last.is_some() {
            let val = last.unwrap();
            if val.active.get() {
                val.active.set(false);
                val.callback.get().map(|c| c.dequeued());
                return true;
            }
        }
        next
    }
}
