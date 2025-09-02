use core::cmp::PartialOrd;
use core::default::Default;
// use core::ops::{Add, Div, Mul, Sub};
use num_traits::{FromPrimitive, Num, Signed};

/// PID controller with generic numeric type `T`.
///
/// `T` might be `f32` or any other fixed point type, that implements the
/// traits `Num + Signed + PartialOrd + Copy + FromPrimitive + Default`.
///
/// # Example
/// ```
/// use cb_controller::pid::{PidCoreBuilder, PidOutputLimit, PidSetpointRange};
///
/// // The essential PID controller configuration is done via builder
/// let mut pid = PidCoreBuilder::<f32>::default()
///     .kp(2.0).ki(0.5).kd(1.0)
///     .sampling_interval(0.1_f32)
///     .build();
///
/// // Optional features are injected after the build any time
/// pid.set_output_limit(PidOutputLimit::default().range(0.0, 10.0).invert_range().anti_windup(true));
/// pid.set_setpoint_range(PidSetpointRange::default().range(0.0, 50.0).out_of_band_output(0.0));
///
/// let mut y = 0.0;
/// let linear_plant = 2.0;
/// let setpoints = [10.0, 20.0, 60.0]; // 60 outside range -> PID off
///
/// for &sp in &setpoints {
///     let u = pid.update(y, sp);
///     if sp > 50.0 {
///         assert!(u == 0.0);
///     } else {
///         y = u * linear_plant;
///     }
/// }
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct PidController<T>
where
    T: Num + Signed + PartialOrd + Copy + Default,
{
    // Configuration items
    kp: T,
    ki: T,
    kd: T,
    dt: f32,
    output_limit: Option<PidOutputLimit<T>>, // output mapping and limitation
    setpoint_range: Option<PidSetpointRange<T>>, // Input validation modells on/off of the controller
    dead_band_tolerance: Option<T>,              // <-- Dead band - tolerance

    // working data
    integral: T,
    last_error: Option<T>,
    last_output: Option<T>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PidOutputLimit<T>
where
    T: Num + Signed + PartialOrd + Copy + Default,
{
    minimum: T,
    maximum: T,    // assured the condition: minimum =< maximum
    pub invert: T, // either 1 (no invert) or -1 (invert) for easy numeric operation
    pub anti_windup: bool,
}

impl<T> PidOutputLimit<T>
where
    T: Num + Signed + PartialOrd + Copy + Default,
{
    pub fn range(self, first: T, second: T) -> Self {
        let (minimum, maximum, invert) = if first < second {
            (first, second, T::one())
        } else {
            (second, first, T::zero() - T::one())
        };
        PidOutputLimit::<T> {
            minimum,
            maximum,
            invert,
            ..self
        }
    }

    pub fn invert_range(self) -> Self {
        PidOutputLimit::<T> {
            invert: T::zero() - T::one(),
            ..self
        }
    }

    /// Anti windup strategy: Conditional integration
    pub fn anti_windup(self, anti_windup: bool) -> Self {
        PidOutputLimit::<T> {
            anti_windup,
            ..self
        }
    }

    // public getter, since minium and maximum must be private due to robust setter
    pub fn minimum(&self) -> T {
        self.minimum
    }
    pub fn maximum(&self) -> T {
        self.maximum
    }
}

impl<T> Default for PidOutputLimit<T>
where
    T: Num + Signed + PartialOrd + Copy + Default,
{
    fn default() -> Self {
        PidOutputLimit::<T> {
            minimum: T::zero(),
            maximum: T::one(),
            invert: T::one(),
            anti_windup: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PidSetpointRange<T>
where
    T: Num + Signed + PartialOrd + Copy + Default,
{
    minimum: T,
    maximum: T,
    pub off_band_output: T,
}

impl<T> PidSetpointRange<T>
where
    T: Num + Signed + PartialOrd + Copy + Default,
{
    pub fn range(self, first: T, second: T) -> Self {
        let (minimum, maximum) = if first < second {
            (first, second)
        } else {
            (second, first)
        };
        PidSetpointRange::<T> {
            minimum,
            maximum,
            ..self
        }
    }

    pub fn out_of_band_output(self, off_band_output: T) -> Self {
        PidSetpointRange::<T> {
            off_band_output,
            ..self
        }
    }

    // public getter, since minium and maximum must be private due to robust setter
    pub fn minimum(&self) -> T {
        self.minimum
    }
    pub fn maximum(&self) -> T {
        self.maximum
    }
}

impl<T> Default for PidSetpointRange<T>
where
    T: Num + Signed + PartialOrd + Copy + Default + FromPrimitive,
{
    fn default() -> Self {
        PidSetpointRange::<T> {
            minimum: T::zero(),
            maximum: T::one(),
            off_band_output: T::zero(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PidCoreBuilder<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default + Clone,
{
    pub kp: T,
    ki: Option<T>,
    kd: Option<T>,
    ti: Option<T>,
    td: Option<T>,
    pub dt: f32,
}

impl<T> Default for PidCoreBuilder<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default,
{
    fn default() -> Self {
        PidCoreBuilder::<T> {
            kp: T::one(),
            ki: Some(T::zero()),
            kd: Some(T::zero()),
            ti: None,
            td: None,
            dt: 1.0_f32,
        }
    }
}

impl<T> Default for PidController<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default + Clone,
{
    fn default() -> Self {
        PidCoreBuilder::<T>::default().build()
    }
}

impl<T> PidController<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default,
{
    /// Enforce output limits
    ///
    /// if output limits are set, there is also an output leveling by $$ maximum + minium / 2 $$
    /// if minimum is less than maximum the output and the leveling is inverted as well
    ///
    /// This output leveling only happens on ON operation.
    /// Switching anti-windup on, prevents the integral portion to windup if out of band.
    pub fn set_output_limit(&mut self, ol: PidOutputLimit<T>) {
        self.output_limit = Some(ol);
    }
    pub fn reset_output_limit(&mut self) {
        self.output_limit = None;
    }
    pub fn is_output_limited(&self) -> bool {
        self.output_limit.is_some()
    }

    /// Enforce allowed setpoint range
    ///
    /// - **ON operation**: if setpoint is within range the pid controller operates as designed
    /// - **OFF operation**: if setpoint is outside this range the pid controller is off and produced
    ///   a constant out of band output
    pub fn set_setpoint_range(&mut self, sr: PidSetpointRange<T>) {
        self.setpoint_range = Some(sr);
    }
    pub fn reset_setpoint_range(&mut self) {
        self.setpoint_range = None;
    }
    pub fn is_setpoint_range(&self) -> bool {
        self.setpoint_range.is_some()
    }

    /// Enforce dead band tolerance
    ///
    /// if the absolute difference between setpoint and process variable is less than the tolerance,
    /// the pid controller output is frozen (to the previous output)
    pub fn set_dead_band_tolerance(&mut self, t: T) {
        self.dead_band_tolerance = Some(t)
    }
    pub fn reset_dead_band_tolerance(&mut self) {
        self.dead_band_tolerance = None;
    }
    pub fn is_dead_band_tolerance(&self) -> bool {
        self.dead_band_tolerance.is_some()
    }

    /// Reset the pid controller - all internal processing states are reset
    /// Any configuration like *Kp*, *Ki*, etc are not changed
    pub fn reset(&mut self) {
        self.integral = T::zero();
        self.last_error = None;
        self.last_output = None;
    }

    /// The control loop update function
    pub fn update(&mut self, input: T, setpoint: T) -> T {
        let dt = T::from_f32(self.dt).unwrap_or(T::one());

        // Setpoint range check
        if let Some(sp) = self.setpoint_range.clone() {
            if setpoint < sp.minimum || setpoint > sp.maximum {
                // controller is OFF
                self.reset();
                return sp.off_band_output;
            }
        }
        // controller is ON from hereafter

        let error = input - setpoint;

        // Deadband tolerance check
        if let Some(tol) = self.dead_band_tolerance {
            if let Some(last_output) = self.last_output {
                if error.abs() <= tol {
                    return last_output;
                } // inside dead band
            }
        }

        // Integral term
        self.integral = self.integral + error * dt;
        // Derivative term
        let derivative = if let Some(last) = self.last_error {
            (error - last) / dt
        } else {
            T::zero()
        };
        let mut output = self.kp * error + self.ki * self.integral + self.kd * derivative;

        // Output Adjustment
        if let Some(ol) = &self.output_limit {
            // Output level shift and inverstion based on output range
            output = output * ol.invert + (ol.maximum + ol.minimum) / (T::one() + T::one());

            let mut limit_correction = false;
            if output < ol.minimum {
                output = ol.minimum;
                limit_correction = true;
            }
            if output > ol.maximum {
                output = ol.maximum;
                limit_correction = true;
            }
            // Anti windup correction by resetting the integral accumulation
            // method is called conditional integration
            // simple, small stationary errors are possible
            if ol.anti_windup && limit_correction {
                self.integral = T::zero();
            }
        }

        self.last_error = Some(error);
        self.last_output = Some(output); // store for dead band tolerance
        output
    }
}

impl<T> PidCoreBuilder<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default,
{
    pub fn kp(mut self, kp: T) -> Self {
        self.kp = kp;
        self
    }
    pub fn ki(mut self, ki: T) -> Self {
        self.ki = Some(ki);
        self.ti = None;
        self
    }
    pub fn kd(mut self, kd: T) -> Self {
        self.kd = Some(kd);
        self.td = None;
        self
    }
    // (German Nachstellzeit)
    pub fn reset_time(mut self, ti: T) -> Self {
        self.ti = Some(ti);
        self.ki = None;
        self
    }
    // (German Vorhaltezeit)
    pub fn hold_time(mut self, td: T) -> Self {
        self.td = Some(td);
        self.kd = None;
        self
    }
    pub fn sampling_interval(mut self, dt: f32) -> Self {
        self.dt = dt;
        self
    }

    pub fn get_ki(&self) -> T {
        match (self.ki, self.ti) {
            (Some(k), _) => k,
            (None, Some(t)) => self.kp / t,
            (None, None) => T::default(),
        }
    }

    pub fn get_kd(&self) -> T {
        match (self.kd, self.td) {
            (Some(k), _) => k,
            (None, Some(t)) => self.kp * t,
            (None, None) => T::default(),
        }
    }

    pub fn get_reset_time(&self) -> T {
        match (self.ki, self.ti) {
            (Some(k), _) => self.kp / k,
            (None, Some(t)) => t,
            (None, None) => T::default(),
        }
    }

    pub fn get_hold_time(&self) -> T {
        match (self.kd, self.td) {
            (Some(k), _) => k / self.kp,
            (None, Some(t)) => t,
            (None, None) => T::default(),
        }
    }

    /// Determine if time parameter (reset_time, hold_time) are preferred over
    /// ki and kd amplification parameter
    ///
    /// if there is no clear indication amplification (return is false)
    pub fn is_time_parameterized(&self) -> bool {
        if self.ti.is_some() && self.kd.is_none() {
            return true;
        }
        if self.td.is_some() && self.ki.is_none() {
            return true;
        }
        false
    }

    pub fn build(self) -> PidController<T> {
        PidController::<T> {
            kp: self.kp,
            ki: self.get_ki(),
            kd: self.get_kd(),
            dt: self.dt,
            integral: T::default(),
            last_error: None,
            last_output: None,
            output_limit: None,
            setpoint_range: None,
            dead_band_tolerance: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pid_basic_update() {
        let mut pid = PidCoreBuilder::<f32>::default()
            .kp(2.0)
            .ki(0.5)
            .kd(1.0)
            .sampling_interval(0.1)
            .build();

        let input = 0.0;
        let setpoint = 10.0;
        let output = pid.update(input, setpoint);
        // Output should be positive and non-zero for positive error
        assert!(output > 0.0);
    }

    #[test]
    fn pid_output_limit() {
        let mut pid = PidCoreBuilder::<f32>::default()
            .kp(1.0)
            .ki(0.0)
            .kd(0.0)
            .sampling_interval(1.0)
            .build();

        let limit = PidOutputLimit::<f32>::default()
            .range(-1.0, 1.0)
            .anti_windup(true);
        pid.set_output_limit(limit);

        let output = pid.update(0.0, 10.0);
        // Output should be limited to maximum
        assert!(output <= 1.0);
    }

    #[test]
    fn pid_output_limit_range_function() {
        let expected = PidOutputLimit::<f32>::default()
            .range(-1.0, 1.0)
            .invert_range();
        let sut = PidOutputLimit::<f32>::default().range(1.0, -1.0);
        assert_eq!(sut, expected);
    }

    #[test]
    fn pid_setpoint_range() {
        let mut pid = PidCoreBuilder::<f32>::default()
            .kp(1.0)
            .ki(0.0)
            .kd(0.0)
            .sampling_interval(1.0)
            .build();

        let range = PidSetpointRange::<f32>::default()
            .range(0.0, 5.0)
            .out_of_band_output(-99.0);
        pid.set_setpoint_range(range);

        // Setpoint outside allowed range
        let output = pid.update(0.0, 10.0);
        assert_eq!(output, -99.0);
    }

    #[test]
    fn pid_setpoint_range_range_function() {
        let expected = PidSetpointRange::<f32>::default()
            .range(0.0, 5.0)
            .out_of_band_output(-99.0);
        let sut = PidSetpointRange::<f32>::default()
            .range(5.0, 0.0)
            .out_of_band_output(-99.0);
        assert_eq!(sut, expected);
    }

    #[test]
    fn pid_dead_band() {
        let mut pid = PidCoreBuilder::<f32>::default()
            .kp(1.0)
            .ki(0.0)
            .kd(0.0)
            .sampling_interval(1.0)
            .build();

        pid.set_dead_band_tolerance(0.5);
        let first = pid.update(0.0, 0.4); // within dead band
        let second = pid.update(0.0, 0.4); // should return last output
        assert_eq!(second, first);
    }

    #[test]
    fn test_kd_and_hold_time_setters_getters() {
        let builder = PidCoreBuilder::<f64>::default();

        // Set Kd and check get_kd and get_hold_time
        let builder_kd = builder.clone().kd(2.5);
        assert_eq!(builder_kd.get_kd(), 2.5);
        // get_hold_time should return the corresponding time value for Kd
        let expected_hold_time = builder_kd.get_hold_time();
        // Setting hold_time should update Kd accordingly
        let builder_hold = builder_kd.clone().hold_time(expected_hold_time);
        assert!((builder_hold.get_kd() - builder_kd.get_kd()).abs() < 1e-10);

        // Set hold_time and check get_hold_time and get_kd
        let builder_hold2 = builder.clone().hold_time(0.8);
        assert!((builder_hold2.get_hold_time() - 0.8).abs() < 1e-10);
        // get_kd should return the corresponding amplification value for hold_time
        let expected_kd = builder_hold2.get_kd();
        let builder_kd2 = builder_hold2.clone().kd(expected_kd);
        assert!((builder_kd2.get_hold_time() - builder_hold2.get_hold_time()).abs() < 1e-10);
    }

    #[test]
    fn test_ki_and_reset_time_setters_getters() {
        let builder = PidCoreBuilder::<f64>::default();

        // Set Ki and check get_ki and get_reset_time
        let builder_ki = builder.clone().ki(1.5);
        assert_eq!(builder_ki.get_ki(), 1.5);
        let expected_reset_time = builder_ki.get_reset_time();
        let builder_reset = builder_ki.clone().reset_time(expected_reset_time);
        assert!((builder_reset.get_ki() - builder_ki.get_ki()).abs() < 1e-10);

        // Set reset_time and check get_reset_time and get_ki
        let builder_reset2 = builder.clone().reset_time(0.6);
        assert!((builder_reset2.get_reset_time() - 0.6).abs() < 1e-10);
        let expected_ki = builder_reset2.get_ki();
        let builder_ki2 = builder_reset2.clone().ki(expected_ki);
        assert!((builder_ki2.get_reset_time() - builder_reset2.get_reset_time()).abs() < 1e-10);
    }
}
