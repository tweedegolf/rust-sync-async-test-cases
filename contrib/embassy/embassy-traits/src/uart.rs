use core::future::Future;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    Other,
}

pub trait Read {
    type ReadFuture<'a>: Future<Output = Result<(), Error>>
    where
        Self: 'a;

    fn read<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadFuture<'a>;
}

pub trait ReadUntilIdle {
    type ReadUntilIdleFuture<'a>: Future<Output = Result<usize, Error>>
    where
        Self: 'a;

    /// Receive into the buffer until the buffer is full or the line is idle after some bytes are received
    /// Return the number of bytes received
    fn read_until_idle<'a>(&'a mut self, buf: &'a mut [u8]) -> Self::ReadUntilIdleFuture<'a>;
}

pub trait Write {
    type WriteFuture<'a>: Future<Output = Result<(), Error>>
    where
        Self: 'a;

    fn write<'a>(&'a mut self, buf: &'a [u8]) -> Self::WriteFuture<'a>;
}
