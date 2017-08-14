//! Provide a queue-based callback for virtualizing OS abstractions.

use core::cell::Cell;
use common::list::{List, ListLink, ListNode};

pub trait Dequeued {
    fn dequeued(&self);
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

    pub fn dequeue_and_trigger(&self) -> bool {
        if self.next.get().is_none() {
            let mut next = false;
            for call in self.queued_calls.iter() {
                if next {
                    self.next.set(Some(call));
                    return true;
                } else if call.active.get() {
                    call.active.set(false);
                    call.callback.get().map(|c| c.dequeued());
                    next = true;
                }
            }
            return next;
        } else {
            let mut next = false;
            for call in self.next.get().iter() {
                if next {
                    self.next.set(Some(call));
                    return true;
                } else if call.active.get() {
                    call.active.set(false);
                    call.callback.get().map(|c| c.dequeued());
                    next = true;
                }
            }
            self.next.set(None);
            // Fired, but it was the last element of the list
            // Since self.next is None we'll start at beginning next time.
            if next == true {
                return true;
            }
            // Didn't fire, so try wrapping around to beginning of list.
            if next == false {
                return self.dequeue_and_trigger();
            }
        }
        false
    }

}
