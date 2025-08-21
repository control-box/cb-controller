#![no_std]

pub mod fixed_point;
pub mod pid;

pub type Fp9_7 = fixed_point::Fixed<i16, 7>;
pub type Fp25_7 = fixed_point::Fixed<i32, 7>;
pub type Fp39_15 = fixed_point::Fixed<i64, 15>;
