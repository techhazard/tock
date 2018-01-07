//! Tock core scheduler.

use core::nonzero::NonZero;
use memop;
use platform::{Chip, Platform};
use platform::mpu::MPU;
use platform::systick::SysTick;
use process;
use process::{Process, Task};
use returncode::ReturnCode;
use syscall::Syscall;

/// The time a process is permitted to run before being pre-empted
const KERNEL_TICK_DURATION_US: u32 = 10000;
/// Skip re-scheduling a process if its quanta is nearly exhausted
const MIN_QUANTA_THRESHOLD_US: u32 = 500;

pub unsafe fn do_process<P: Platform, C: Chip>(
    platform: &P,
    chip: &mut C,
    process: &mut Process,
    appid: ::AppId,
    ipc: &::ipc::IPC,
) {
    let systick = chip.systick();
    systick.reset();
    systick.set_timer(KERNEL_TICK_DURATION_US);
    systick.enable(true);

    loop {
        if chip.has_pending_interrupts() || systick.overflowed()
            || systick.value() <= MIN_QUANTA_THRESHOLD_US
        {
            break;
        }

        match process.current_state() {
            process::State::Running => {
                process.setup_mpu(chip.mpu());
                chip.mpu().enable_mpu();
                systick.enable(true);
                process.switch_to();
                systick.enable(false);
                chip.mpu().disable_mpu();
            }
            process::State::Yielded => match process.dequeue_task() {
                None => break,
                Some(cb) => {
                    match cb {
                        Task::FunctionCall(ccb) => {
                            process.push_function_call(ccb);
                        }
                        Task::IPC((otherapp, ipc_type)) => {
                            ipc.schedule_callback(appid, otherapp, ipc_type);
                        }
                    }
                    continue;
                }
            },
            process::State::Fault => {
                // we should never be scheduling a process in fault
                panic!("Attempted to schedule a faulty process");
            }
        }

        if !process.syscall_fired() {
            break;
        }

        // check if the app had a fault
        if process.app_fault() {
            // let process deal with it as appropriate
            process.fault_state();
            continue;
        }

        // process had a system call, count it
        process.incr_syscall_count();
        match process.svc_number() {
            Some(Syscall::MEMOP) => {
                let res = memop::memop(process);
                process.set_return_code(res);
            }
            Some(Syscall::YIELD) => {
                process.yield_state();
                process.pop_syscall_stack();

                // There might be already enqueued callbacks
                continue;
            }
            Some(Syscall::SUBSCRIBE) => {
                let driver_num = process.r0();
                let subdriver_num = process.r1();
                let callback_ptr_raw = process.r2() as *mut ();
                let appdata = process.r3();

                let res = if callback_ptr_raw as usize == 0 {
                    ReturnCode::EINVAL
                } else {
                    let callback_ptr = NonZero::new_unchecked(callback_ptr_raw);

                    let callback = ::Callback::new(appid, appdata, callback_ptr);
                    platform.with_driver(driver_num, |driver| match driver {
                        Some(d) => d.subscribe(subdriver_num, callback),
                        None => ReturnCode::ENODEVICE,
                    })
                };
                process.set_return_code(res);
            }
            Some(Syscall::COMMAND) => {
                let res = platform.with_driver(process.r0(), |driver| match driver {
                    Some(d) => d.command(process.r1(), process.r2(), process.r3(), appid),
                    None => ReturnCode::ENODEVICE,
                });
                process.set_return_code(res);
            }
            Some(Syscall::ALLOW) => {
                let res = platform.with_driver(process.r0(), |driver| {
                    match driver {
                        Some(d) => {
                            let start_addr = process.r2() as *mut u8;
                            let size = process.r3();
                            if process.in_exposed_bounds(start_addr, size) {
                                let slice = ::AppSlice::new(start_addr as *mut u8, size, appid);
                                d.allow(appid, process.r1(), slice)
                            } else {
                                ReturnCode::EINVAL /* memory not allocated to process */
                            }
                        }
                        None => ReturnCode::ENODEVICE,
                    }
                });
                process.set_return_code(res);
            }
            _ => {}
        }
    }
    systick.reset();
}
