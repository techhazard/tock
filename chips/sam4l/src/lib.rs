//! Peripheral implementations for the SAM4L MCU.
//!
//! http://www.atmel.com/microsite/sam4l/default.aspx

#![crate_name = "sam4l"]
#![crate_type = "rlib"]
#![feature(repr_align, attr_literals, const_cell_new)]
#![feature(const_atomic_usize_new, const_ptr_null_mut, integer_atomics, try_from)]
#![feature(asm, core_intrinsics, concat_idents, const_fn)]
#![no_std]

extern crate cortexm4;
#[allow(unused_imports)]
#[macro_use(debug)]
extern crate kernel;

#[macro_use]
mod helpers;

pub mod chip;
pub mod ast;
pub mod bpm;
pub mod bscif;
pub mod dma;
pub mod i2c;
pub mod spi;
pub mod nvic;
pub mod pm;
pub mod gpio;
pub mod usart;
pub mod scif;
pub mod adc;
pub mod flashcalw;
pub mod wdt;
pub mod trng;
pub mod crccu;
pub mod dac;
pub mod aes;
pub mod usbc;

use cortexm4::{generic_isr, systick_handler, SVC_Handler};

unsafe extern "C" fn unhandled_interrupt() {
    let mut interrupt_number: u32;

    // IPSR[8:0] holds the currently active interrupt
    asm!(
        "mrs    r0, ipsr                    "
        : "={r0}"(interrupt_number)
        :
        : "r0"
        :
        );

    interrupt_number = interrupt_number & 0x1ff;

    panic!("Unhandled Interrupt. ISR {} is active.", interrupt_number);
}

extern "C" {
    // _estack is not really a function, but it makes the types work
    // You should never actually invoke it!!
    fn _estack();

    // Defined by platform
    fn reset_handler();

    static mut _szero: u32;
    static mut _ezero: u32;
    static mut _etext: u32;
    static mut _srelocate: u32;
    static mut _erelocate: u32;
}

#[link_section=".vectors"]
#[cfg_attr(rustfmt, rustfmt_skip)]
// no_mangle Ensures that the symbol is kept until the final binary
#[no_mangle]
pub static BASE_VECTORS: [unsafe extern fn(); 16] = [
    _estack, reset_handler,
    /* NMI */           unhandled_interrupt,
    /* Hard Fault */    hard_fault_handler,
    /* MemManage */     unhandled_interrupt,
    /* BusFault */      unhandled_interrupt,
    /* UsageFault*/     unhandled_interrupt,
    unhandled_interrupt, unhandled_interrupt, unhandled_interrupt,
    unhandled_interrupt,
    /* SVC */           SVC_Handler,
    /* DebugMon */      unhandled_interrupt,
    unhandled_interrupt,
    /* PendSV */        unhandled_interrupt,
    /* SysTick */       systick_handler
];

#[link_section = ".vectors"]
#[no_mangle] // Ensures that the symbol is kept until the final binary
pub static IRQS: [unsafe extern "C" fn(); 80] = [generic_isr; 80];

pub unsafe fn init() {
    // Relocate data segment.
    // Assumes data starts right after text segment as specified by the linker
    // file.
    let mut pdest = &mut _srelocate as *mut u32;
    let pend = &mut _erelocate as *mut u32;
    let mut psrc = &_etext as *const u32;

    if psrc != pdest {
        while (pdest as *const u32) < pend {
            *pdest = *psrc;
            pdest = pdest.offset(1);
            psrc = psrc.offset(1);
        }
    }

    // Clear the zero segment (BSS)
    let pzero = &_ezero as *const u32;
    pdest = &mut _szero as *mut u32;

    while (pdest as *const u32) < pzero {
        *pdest = 0;
        pdest = pdest.offset(1);
    }

    cortexm4::nvic::disable_all();
    cortexm4::nvic::clear_all_pending();
    cortexm4::nvic::enable_all();
}

unsafe extern "C" fn hard_fault_handler() {
    use core::intrinsics::offset;

    let faulting_stack: *mut u32;
    let kernel_stack: bool;

    asm!(
        "mov    r1, 0                       \n\
         tst    lr, #4                      \n\
         itte   eq                          \n\
         mrseq  r0, msp                     \n\
         addeq  r1, 1                       \n\
         mrsne  r0, psp                     "
        : "={r0}"(faulting_stack), "={r1}"(kernel_stack)
        :
        : "r0", "r1"
        :
        );

    if kernel_stack {
        let stacked_r0: u32 = *offset(faulting_stack, 0);
        let stacked_r1: u32 = *offset(faulting_stack, 1);
        let stacked_r2: u32 = *offset(faulting_stack, 2);
        let stacked_r3: u32 = *offset(faulting_stack, 3);
        let stacked_r12: u32 = *offset(faulting_stack, 4);
        let stacked_lr: u32 = *offset(faulting_stack, 5);
        let stacked_pc: u32 = *offset(faulting_stack, 6);
        let stacked_xpsr: u32 = *offset(faulting_stack, 7);

        let mode_str = "Kernel";

        let shcsr: u32 = core::ptr::read_volatile(0xE000ED24 as *const u32);
        let cfsr: u32 = core::ptr::read_volatile(0xE000ED28 as *const u32);
        let hfsr: u32 = core::ptr::read_volatile(0xE000ED2C as *const u32);
        let mmfar: u32 = core::ptr::read_volatile(0xE000ED34 as *const u32);
        let bfar: u32 = core::ptr::read_volatile(0xE000ED38 as *const u32);

        let iaccviol = (cfsr & 0x01) == 0x01;
        let daccviol = (cfsr & 0x02) == 0x02;
        let munstkerr = (cfsr & 0x08) == 0x08;
        let mstkerr = (cfsr & 0x10) == 0x10;
        let mlsperr = (cfsr & 0x20) == 0x20;
        let mmfarvalid = (cfsr & 0x80) == 0x80;

        let ibuserr = ((cfsr >> 8) & 0x01) == 0x01;
        let preciserr = ((cfsr >> 8) & 0x02) == 0x02;
        let impreciserr = ((cfsr >> 8) & 0x04) == 0x04;
        let unstkerr = ((cfsr >> 8) & 0x08) == 0x08;
        let stkerr = ((cfsr >> 8) & 0x10) == 0x10;
        let lsperr = ((cfsr >> 8) & 0x20) == 0x20;
        let bfarvalid = ((cfsr >> 8) & 0x80) == 0x80;

        let undefinstr = ((cfsr >> 16) & 0x01) == 0x01;
        let invstate = ((cfsr >> 16) & 0x02) == 0x02;
        let invpc = ((cfsr >> 16) & 0x04) == 0x04;
        let nocp = ((cfsr >> 16) & 0x08) == 0x08;
        let unaligned = ((cfsr >> 16) & 0x100) == 0x100;
        let divbysero = ((cfsr >> 16) & 0x200) == 0x200;

        let vecttbl = (hfsr & 0x02) == 0x02;
        let forced = (hfsr & 0x40000000) == 0x40000000;

        let ici_it = (((stacked_xpsr >> 25) & 0x3) << 6) | ((stacked_xpsr >> 10) & 0x3f);
        let thumb_bit = ((stacked_xpsr >> 24) & 0x1) == 1;
        let exception_number = (stacked_xpsr & 0x1ff) as usize;

        panic!(
            "{} HardFault.\n\
             \tKernel version {}\n\
             \tr0  0x{:x}\n\
             \tr1  0x{:x}\n\
             \tr2  0x{:x}\n\
             \tr3  0x{:x}\n\
             \tr12 0x{:x}\n\
             \tlr  0x{:x}\n\
             \tpc  0x{:x}\n\
             \tprs 0x{:x} [ N {} Z {} C {} V {} Q {} GE {}{}{}{} ; ICI.IT {} T {} ; Exc {}-{} ]\n\
             \tsp  0x{:x}\n\
             \ttop of stack     0x{:x}\n\
             \tbottom of stack  0x{:x}\n\
             \tSHCSR 0x{:x}\n\
             \tCFSR  0x{:x}\n\
             \tHSFR  0x{:x}\n\
             \tInstruction Access Violation:       {}\n\
             \tData Access Violation:              {}\n\
             \tMemory Management Unstacking Fault: {}\n\
             \tMemory Management Stacking Fault:   {}\n\
             \tMemory Management Lazy FP Fault:    {}\n\
             \tInstruction Bus Error:              {}\n\
             \tPrecise Data Bus Error:             {}\n\
             \tImprecise Data Bus Error:           {}\n\
             \tBus Unstacking Fault:               {}\n\
             \tBus Stacking Fault:                 {}\n\
             \tBus Lazy FP Fault:                  {}\n\
             \tUndefined Instruction Usage Fault:  {}\n\
             \tInvalid State Usage Fault:          {}\n\
             \tInvalid PC Load Usage Fault:        {}\n\
             \tNo Coprocessor Usage Fault:         {}\n\
             \tUnaligned Access Usage Fault:       {}\n\
             \tDivide By Zero:                     {}\n\
             \tBus Fault on Vector Table Read:     {}\n\
             \tForced Hard Fault:                  {}\n\
             \tFaulting Memory Address: (valid: {}) {:#010X}\n\
             \tBus Fault Address:       (valid: {}) {:#010X}\n\
             ",
            mode_str,
            env!("TOCK_KERNEL_VERSION"),
            stacked_r0,
            stacked_r1,
            stacked_r2,
            stacked_r3,
            stacked_r12,
            stacked_lr,
            stacked_pc,
            stacked_xpsr,
            (stacked_xpsr >> 31) & 0x1,
            (stacked_xpsr >> 30) & 0x1,
            (stacked_xpsr >> 29) & 0x1,
            (stacked_xpsr >> 28) & 0x1,
            (stacked_xpsr >> 27) & 0x1,
            (stacked_xpsr >> 19) & 0x1,
            (stacked_xpsr >> 18) & 0x1,
            (stacked_xpsr >> 17) & 0x1,
            (stacked_xpsr >> 16) & 0x1,
            ici_it,
            thumb_bit,
            exception_number,
            kernel::process::ipsr_isr_number_to_str(exception_number),
            faulting_stack as u32,
            (_estack as *const ()) as u32,
            (&_ezero as *const u32) as u32,
            shcsr,
            cfsr,
            hfsr,
            iaccviol,
            daccviol,
            munstkerr,
            mstkerr,
            mlsperr,
            ibuserr,
            preciserr,
            impreciserr,
            unstkerr,
            stkerr,
            lsperr,
            undefinstr,
            invstate,
            invpc,
            nocp,
            unaligned,
            divbysero,
            vecttbl,
            forced,
            mmfarvalid,
            mmfar,
            bfarvalid,
            bfar
        );
    } else {
        // hard fault occurred in an app, not the kernel. The app should be
        //  marked as in an error state and handled by the kernel
        asm!(
            "ldr r0, =SYSCALL_FIRED
              mov r1, #1
              str r1, [r0, #0]

              ldr r0, =APP_FAULT
              str r1, [r0, #0]

              /* Read the SCB registers. */
              ldr r0, =SCB_REGISTERS
              ldr r1, =0xE000ED14
              ldr r2, [r1, #0] /* CCR */
              str r2, [r0, #0]
              ldr r2, [r1, #20] /* CFSR */
              str r2, [r0, #4]
              ldr r2, [r1, #24] /* HFSR */
              str r2, [r0, #8]
              ldr r2, [r1, #32] /* MMFAR */
              str r2, [r0, #12]
              ldr r2, [r1, #36] /* BFAR */
              str r2, [r0, #16]

              /* Set thread mode to privileged */
              mov r0, #0
              msr CONTROL, r0

              movw LR, #0xFFF9
              movt LR, #0xFFFF"
        );
    }
}
