//! Inter-process communication mechanism for Tock.
//!
//! This is a special syscall driver that allows userspace applications to
//! share memory.

/// Syscall number
pub const DRIVER_NUM: usize = 0x00010000;

use {AppId, AppSlice, Callback, Driver, Grant, Shared};
use process;
use returncode::ReturnCode;

struct IPCData {
    shared_memory: [Option<AppSlice<Shared, u8>>; 8],
    client_callbacks: [Option<Callback>; 8],
    callback: Option<Callback>,
}

impl Default for IPCData {
    fn default() -> IPCData {
        IPCData {
            shared_memory: [None, None, None, None, None, None, None, None],
            client_callbacks: [None, None, None, None, None, None, None, None],
            callback: None,
        }
    }
}

pub struct IPC {
    data: Grant<IPCData>,
}

impl IPC {
    pub unsafe fn new() -> IPC {
        IPC {
            data: Grant::create(),
        }
    }

    pub unsafe fn schedule_callback(
        &self,
        appid: AppId,
        otherapp: AppId,
        cb_type: process::IPCType,
    ) {
        self.data
            .enter(appid, |mydata, _| {
                let callback = match cb_type {
                    process::IPCType::Service => mydata.callback,
                    process::IPCType::Client => {
                        *mydata.client_callbacks.get(otherapp.idx()).unwrap_or(&None)
                    }
                };
                callback
                    .map(|mut callback| {
                        self.data
                            .enter(otherapp, |otherdata, _| {
                                if appid.idx() >= otherdata.shared_memory.len() {
                                    return;
                                }
                                match otherdata.shared_memory[appid.idx()] {
                                    Some(ref slice) => {
                                        slice.expose_to(appid);
                                        callback.schedule(
                                            otherapp.idx() + 1,
                                            slice.len(),
                                            slice.ptr() as usize,
                                        );
                                    }
                                    None => {
                                        callback.schedule(otherapp.idx() + 1, 0, 0);
                                    }
                                }
                            })
                            .unwrap_or(());
                    })
                    .unwrap_or(());
            })
            .unwrap_or(());
    }
}

impl Driver for IPC {
    /// subscribe enables processes using IPC to register callbacks that fire
    /// when notify() is called.
    fn subscribe(&self, subscribe_num: usize, callback: Callback) -> ReturnCode {
        match subscribe_num {
            // subscribe(0)
            //
            // Subscribe with subscribe_num == 0 is how a process registers
            // itself as an IPC service. Each process can only register as a
            // single IPC service. The identifier for the IPC service is the
            // application name stored in the TBF header of the application.
            // The callback that is passed to subscribe is called when another
            // process notifies the server process.
            0 => self.data
                .enter(callback.app_id(), |data, _| {
                    data.callback = Some(callback);
                    ReturnCode::SUCCESS
                })
                .unwrap_or(ReturnCode::EBUSY),

            // subscribe(>=1)
            //
            // Subscribe with subscribe_num >= 1 is how a client registers
            // a callback for a given service. The service number (passed
            // here as subscribe_num) is returned from the allow() call.
            // Once subscribed, the client will receive callbacks when the
            // service process calls notify_client().
            svc_id => {
                if svc_id - 1 >= 8 {
                    ReturnCode::EINVAL /* Maximum of 8 IPC's exceeded */
                } else {
                    self.data
                        .enter(callback.app_id(), |data, _| {
                            data.client_callbacks[svc_id - 1] = Some(callback);
                            ReturnCode::SUCCESS
                        })
                        .unwrap_or(ReturnCode::EBUSY)
                }
            }
        }
    }

    /// command is how notify() is implemented.
    /// Notifying an IPC service is done by setting client_or_svc to 0,
    /// and notifying an IPC client is done by setting client_or_svc to 1.
    /// In either case, the target_id is the same number as provided in a notify
    /// callback or as returned by allow.
    fn command(
        &self,
        target_id: usize,
        client_or_svc: usize,
        _: usize,
        appid: AppId,
    ) -> ReturnCode {
        let procs = unsafe { &mut process::PROCS };
        if target_id == 0 || target_id > procs.len() {
            return ReturnCode::EINVAL; /* Request to IPC to impossible process */
        }

        let cb_type = if client_or_svc == 0 {
            process::IPCType::Service
        } else {
            process::IPCType::Client
        };

        procs[target_id - 1]
            .as_mut()
            .map(|target| {
                target.schedule_ipc(appid, cb_type);
                ReturnCode::SUCCESS
            })
            .unwrap_or(ReturnCode::EINVAL) /* Request to IPC to unknown process */
    }

    /// allow enables processes to discover IPC services on the platform or
    /// share buffers with existing services.
    ///
    /// If allow is called with target_id == 0, it is an IPC service discover
    /// call. The contents of the slice should be the string name of the IPC
    /// service. If this mechanism can find that service, allow will return
    /// an ID that can be used to notify that service. Otherwise an error will
    /// be returned.
    ///
    /// If allow is called with target_id >= 1, it is a share command where the
    /// application is explicitly sharing a slice with an IPC service (as
    /// specified by the target_id). allow() simply allows both processes to
    /// access the buffer, it does not signal the service.
    fn allow(&self, appid: AppId, target_id: usize, slice: AppSlice<Shared, u8>) -> ReturnCode {
        if target_id == 0 {
            if slice.len() > 0 {
                let procs = unsafe { &mut process::PROCS };
                for (i, process) in procs.iter().enumerate() {
                    match process {
                        &Some(ref p) => {
                            let s = p.package_name.as_bytes();
                            // are slices equal?
                            if s.len() == slice.len()
                                && s.iter().zip(slice.iter()).all(|(c1, c2)| c1 == c2)
                            {
                                return ReturnCode::SuccessWithValue {
                                    value: (i as usize) + 1,
                                };
                            }
                        }
                        &None => {}
                    }
                }
            }
            return ReturnCode::EINVAL; /* AppSlice must have non-zero length */
        }
        return self.data
            .enter(appid, |data, _| {
                data.shared_memory
                    .get_mut(target_id - 1)
                    .map(|smem| {
                        *smem = Some(slice);
                        ReturnCode::SUCCESS
                    })
                    .unwrap_or(ReturnCode::EINVAL) /* Target process does not exist */
            })
            .unwrap_or(ReturnCode::EBUSY);
    }
}
