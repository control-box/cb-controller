use core::fmt::{self, Debug, Display};
use core::ops::*;
use num_traits::{Bounded, PrimInt, Signed};

#[allow(unused_imports)]
use num_traits::float::FloatCore; // for round() function call

/// Generic Fixed Point Data Type
///
/// - `T`: Basetype (e.g.. i16, i32, i64)
/// - `FRAC`: Number of bits after the point
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Fixed<T: PrimInt + Signed + Bounded, const FRAC: usize> {
    raw: T,
}

impl<T: PrimInt + Signed + Bounded, const FRAC: usize> Fixed<T, FRAC> {
    pub const SCALE: i128 = 1i128 << FRAC;

    pub const fn from_raw(raw: T) -> Self {
        Self { raw }
    }

    pub fn raw(self) -> T {
        self.raw
    }

    pub fn from_int(v: T) -> Self {
        let wide = (v.to_i128().unwrap()) * Self::SCALE;
        Self::sat_from_i128(wide)
    }

    pub fn from_f32(v: f32) -> Self {
        let wide = (v * (Self::SCALE as f32)).round() as i128;
        Self::sat_from_i128(wide)
    }

    pub fn from_f64(v: f64) -> Self {
        let wide = (v * (Self::SCALE as f64)).round() as i128;
        Self::sat_from_i128(wide)
    }

    pub fn to_int(self) -> T {
        let val = self.raw.to_i128().unwrap() >> FRAC;
        // Clamp to T's range
        let min = T::min_value().to_i128().unwrap();
        let max = T::max_value().to_i128().unwrap();
        let clamped = val.clamp(min, max);
        T::from(clamped).unwrap()
    }

    pub fn to_f32(self) -> f32 {
        (self.raw.to_i128().unwrap() as f32) / (Self::SCALE as f32)
    }

    pub fn to_f64(self) -> f64 {
        (self.raw.to_i128().unwrap() as f64) / (Self::SCALE as f64)
    }

    fn sat_from_i128(v: i128) -> Self {
        let min = T::min_value().to_i128().unwrap();
        let max = T::max_value().to_i128().unwrap();
        let clamped = v.clamp(min, max);
        Self {
            raw: T::from(clamped).unwrap(),
        }
    }
}

// --- saturierende Operatoren ---
impl<T: PrimInt + Signed + Bounded, const FRAC: usize> Add for Fixed<T, FRAC> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        let res = (self.raw.to_i128().unwrap()) + (rhs.raw.to_i128().unwrap());
        Self::sat_from_i128(res)
    }
}
impl<T: PrimInt + Signed + Bounded, const FRAC: usize> Sub for Fixed<T, FRAC> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        let res = (self.raw.to_i128().unwrap()) - (rhs.raw.to_i128().unwrap());
        Self::sat_from_i128(res)
    }
}
impl<T: PrimInt + Signed + Bounded, const FRAC: usize> Mul for Fixed<T, FRAC> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        let prod = (self.raw.to_i128().unwrap()) * (rhs.raw.to_i128().unwrap());
        let res = prod >> FRAC;
        Self::sat_from_i128(res)
    }
}
impl<T: PrimInt + Signed + Bounded, const FRAC: usize> Div for Fixed<T, FRAC> {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        if rhs.raw.is_zero() {
            return if self.raw.is_negative() {
                Self::from_raw(T::min_value())
            } else {
                Self::from_raw(T::max_value())
            };
        }
        let num = (self.raw.to_i128().unwrap()) << FRAC;
        let res = num / (rhs.raw.to_i128().unwrap());
        Self::sat_from_i128(res)
    }
}

impl<T: PrimInt + Signed + Bounded + Debug, const FRAC: usize> fmt::Debug for Fixed<T, FRAC> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Fixed(raw={:?}, val={:.6})", self.raw, self.to_f64())
    }
}

impl<T: PrimInt + Signed + Bounded + Display, const FRAC: usize> fmt::Display for Fixed<T, FRAC> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.3}", self.to_f64())
    }
}

// --- Demo ---
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn demo_i16_frac7() {
        type Fix16_7 = Fixed<i16, 7>;
        let a = Fix16_7::from_f32(3.5);
        let b = Fix16_7::from_f32(2.0);
        assert!(((a * b).to_f32() - 7.0).abs() < 1e-3);
        assert_eq!((a + b).to_f32(), 5.5);
    }

    #[test]
    fn demo_i32_frac7() {
        type Fix32_7 = Fixed<i32, 7>;
        let a = Fix32_7::from_f64(123.25);
        assert_eq!(a.to_int(), 123);
    }

    #[test]
    fn saturating_add() {
        type Fix16_7 = Fixed<i16, 7>;
        let big = Fix16_7::from_f32(255.0);
        let res = big + Fix16_7::from_f32(10.0);
        assert_eq!(res.raw(), i16::MAX); // Saturation
    }
}
