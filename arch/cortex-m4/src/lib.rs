//! Shared implementations for ARM Cortex-M4 MCUs.

#![crate_name = "cortexm4"]
#![crate_type = "rlib"]
#![feature(asm, const_fn, naked_functions)]
#![no_std]

#[allow(unused_imports)]
#[macro_use(debug, debug_gpio)]
extern crate kernel;

pub mod mpu;
pub mod nvic;
pub mod systick;
pub mod scb;

#[cfg(not(target_os = "none"))]
pub unsafe extern "C" fn systick_handler() {}

#[cfg(target_os = "none")]
#[naked]
pub unsafe extern "C" fn systick_handler() {
    asm!(
        "
        /* Skip saving process state if not coming from user-space */
        cmp lr, #0xfffffffd
        bne _systick_handler_no_stacking

        /* We need the most recent kernel's version of r1, which points */
        /* to the Process struct's stored registers field. The kernel's r1 */
        /* lives in the second word of the hardware stacked registers on MSP */
        mov r1, sp
        ldr r1, [r1, #4]
        stmia r1, {r4-r11}
    _systick_handler_no_stacking:
        ldr r0, =OVERFLOW_FIRED
        mov r1, #1
        str r1, [r0, #0]

        /* Set thread mode to privileged */
        mov r0, #0
        msr CONTROL, r0

        movw LR, #0xFFF9
        movt LR, #0xFFFF
         "
    );
}

#[cfg(not(target_os = "none"))]
pub unsafe extern "C" fn generic_isr() {}

#[cfg(target_os = "none")]
#[naked]
/// All ISRs are caught by this handler which disables the NVIC and switches to the kernel.
pub unsafe extern "C" fn generic_isr() {
    asm!(
        "
    /* Skip saving process state if not coming from user-space */
    cmp lr, #0xfffffffd
    bne _ggeneric_isr_no_stacking

    /* We need the most recent kernel's version of r1, which points */
    /* to the Process struct's stored registers field. The kernel's r1 */
    /* lives in the second word of the hardware stacked registers on MSP */
    mov r1, sp
    ldr r1, [r1, #4]
    stmia r1, {r4-r11}

    /* Set thread mode to privileged */
    mov r0, #0
    msr CONTROL, r0

    movw LR, #0xFFF9
    movt LR, #0xFFFF
_ggeneric_isr_no_stacking:
    /* Find the ISR number by looking at the low byte of the IPSR registers */
    mrs r0, IPSR
    and r0, #0xff
    /* ISRs start at 16, so substract 16 to get zero-indexed */
    sub r0, #16

    /*
     * High level:
     *    NVIC.ICER[r0 / 32] = 1 << (r0 & 31)
     * */
	lsrs	r2, r0, #5 /* r2 = r0 / 32 */

    /* r0 = 1 << (r0 & 31) */
	movs r3, #1        /* r3 = 1 */
	and	r0, r0, #31    /* r0 = r0 & 31 */
	lsl	r0, r3, r0     /* r0 = r3 << r0 */

    /* r3 = &NVIC.ICER */
    mov r3, #0xe180
    movt r3, #0xe000

    /* here:
     *
     *  `r2` is r0 / 32
     *  `r3` is &NVIC.ICER
     *  `r0` is 1 << (r0 & 31)
     *
     * So we just do:
     *
     *  `*(r3 + r2 * 4) = r0`
     *
     *  */
	str	r0, [r3, r2, lsl #2]"
    );
}

#[cfg(not(target_os = "none"))]
#[allow(non_snake_case)]
pub unsafe extern "C" fn SVC_Handler() {}

#[cfg(target_os = "none")]
#[naked]
#[allow(non_snake_case)]
pub unsafe extern "C" fn SVC_Handler() {
    asm!(
        "
  cmp lr, #0xfffffff9
  bne to_kernel

  /* Set thread mode to unprivileged */
  mov r0, #1
  msr CONTROL, r0

  movw lr, #0xfffd
  movt lr, #0xffff
  bx lr
to_kernel:
  ldr r0, =SYSCALL_FIRED
  mov r1, #1
  str r1, [r0, #0]

  /* Set thread mode to privileged */
  mov r0, #0
  msr CONTROL, r0

  movw LR, #0xFFF9
  movt LR, #0xFFFF"
    );
}

#[cfg(not(target_os = "none"))]
pub unsafe extern "C" fn switch_to_user(user_stack: *const u8, process_got: *const u8) -> *mut u8 {
    user_stack as *mut u8
}

#[cfg(target_os = "none")]
#[no_mangle]
/// r0 is top of user stack, r1 Process GOT
pub unsafe extern "C" fn switch_to_user(
    mut user_stack: *const u8,
    process_regs: &mut [usize; 8],
) -> *mut u8 {
    asm!("
    /* Load bottom of stack into Process Stack Pointer */
    msr psp, $0

    /* Load non-hardware-stacked registers from Process stack */
    /* Ensure that $2 is stored in a callee saved register */
    ldmia $2, {r4-r11}

    /* SWITCH */
    svc 0xff /* It doesn't matter which SVC number we use here */

    /* Push non-hardware-stacked registers into Process struct's */
    /* regs field */
    stmia $2, {r4-r11}

    mrs $0, PSP /* PSP into r0 */"
    : "={r0}"(user_stack)
    : "{r0}"(user_stack), "{r1}"(process_regs)
    : "r4","r5","r6","r7","r8","r9","r10","r11");
    user_stack as *mut u8
}
