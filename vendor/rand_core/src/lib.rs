#![no_std]

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct Error;

pub trait RngCore {
    fn next_u32(&mut self) -> u32;
    fn next_u64(&mut self) -> u64;
    fn fill_bytes(&mut self, dest: &mut [u8]);
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Error>;
}

pub mod impls {
    use super::RngCore;

    pub fn fill_bytes_via_next<R: RngCore + ?Sized>(rng: &mut R, dest: &mut [u8]) {
        let mut left = dest;
        while left.len() >= 8 {
            let chunk = rng.next_u64().to_le_bytes();
            left[..8].copy_from_slice(&chunk);
            left = &mut left[8..];
        }
        if !left.is_empty() {
            let chunk = rng.next_u64().to_le_bytes();
            left.copy_from_slice(&chunk[..left.len()]);
        }
    }
}
