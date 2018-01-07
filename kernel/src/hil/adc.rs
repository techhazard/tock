//! Interfaces for analog to digital converter peripherals.

use returncode::ReturnCode;

// *** Interfaces for low-speed, single-sample ADCs ***

/// Simple interface for reading an ADC sample on any channel.
pub trait Adc {
    /// The chip-dependent type of an ADC channel.
    type Channel;

    /// Initialize must be called before taking a sample.
    fn initialize(&self) -> ReturnCode;

    /// Request a single ADC sample on a particular channel.
    /// Used for individual samples that have no timing requirements.
    fn sample(&self, channel: &Self::Channel) -> ReturnCode;

    /// Request repeated ADC samples on a particular channel.
    /// Callbacks will occur at the given frequency with low jitter and can be
    /// set to any frequency supported by the chip implementation. However
    /// callbacks may be limited based on how quickly the system can service
    /// individual samples, leading to missed samples at high frequencies.
    fn sample_continuous(&self, channel: &Self::Channel, frequency: u32) -> ReturnCode;

    /// Stop a sampling operation.
    /// Can be used to stop any simple or high-speed sampling operation. No
    /// further callbacks will occur.
    fn stop_sampling(&self) -> ReturnCode;
}

/// Trait for handling callbacks from simple ADC calls.
pub trait Client {
    /// Called when a sample is ready.
    fn sample_ready(&self, sample: u16);
}

// *** Interfaces for high-speed, buffered ADC sampling ***

/// Interface for continuously sampling at a given frequency on a channel.
/// Requires the AdcSimple interface to have been implemented as well.
pub trait AdcHighSpeed: Adc {
    /// Start sampling continuously into buffers.
    /// Samples are double-buffered, going first into `buffer1` and then into
    /// `buffer2`. A callback is performed to the client whenever either buffer
    /// is full, which expects either a second buffer to be sent via the
    /// `provide_buffer` call. Length fields correspond to the number of
    /// samples that should be collected in each buffer. If an error occurs,
    /// the buffers will be returned.
    fn sample_highspeed(
        &self,
        channel: &Self::Channel,
        frequency: u32,
        buffer1: &'static mut [u16],
        length1: usize,
        buffer2: &'static mut [u16],
        length2: usize,
    ) -> (
        ReturnCode,
        Option<&'static mut [u16]>,
        Option<&'static mut [u16]>,
    );

    /// Provide a new buffer to fill with the ongoing `sample_continuous`
    /// configuration.
    /// Expected to be called in a `buffer_ready` callback. Note that if this
    /// is not called before the second buffer is filled, samples will be
    /// missed. Length field corresponds to the number of samples that should
    /// be collected in the buffer. If an error occurs, the buffer will be
    /// returned.
    fn provide_buffer(
        &self,
        buf: &'static mut [u16],
        length: usize,
    ) -> (ReturnCode, Option<&'static mut [u16]>);

    /// Reclaim ownership of buffers.
    /// Can only be called when the ADC is inactive, which occurs after a
    /// successful `stop_sampling`. Used to reclaim buffers after a sampling
    /// operation is complete. Returns success if the ADC was inactive, but
    /// there may still be no buffers that are `some` if the driver had already
    /// returned all buffers.
    fn retrieve_buffers(
        &self,
    ) -> (
        ReturnCode,
        Option<&'static mut [u16]>,
        Option<&'static mut [u16]>,
    );
}

/// Trait for handling callbacks from high-speed ADC calls.
pub trait HighSpeedClient {
    /// Called when a buffer is full.
    /// The length provided will always be less than or equal to the length of
    /// the buffer. Expects an additional call to either provide another buffer
    /// or stop sampling
    fn samples_ready(&self, buf: &'static mut [u16], length: usize);
}
