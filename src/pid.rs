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
/// use cb_controller::pid::PID;
///
/// let sample_time =0.1_f32;
/// let mut pid = PID::<f32>::builder()
///     .kp(2.0).ki(0.5).kd(1.0)
///     .dt(sample_time)
///     .output_limits(0.0, 100.0)
///     .setpoint_range(0.0, 50.0)
///     .anti_windup(true)
///     .build();
///
/// let mut input = 0.0;
/// let setpoints = [10.0, 20.0, 60.0]; // 60 outside range -> PID off
///
/// for &sp in &setpoints {
///     let output = pid.update(input, sp);
///     if sp > 50.0 {
///         assert!(output.is_none());
///     } else {
///         let val = output.unwrap();
///         input += val * sample_time;
///     }
/// }
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct PID<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default,
{
    kp: T,
    ki: T,
    kd: T,
    dt: f32,

    integral: T,
    last_error: Option<T>,
    last_output: Option<T>,

    output_min: Option<T>,
    output_max: Option<T>,

    setpoint_min: Option<T>,
    setpoint_max: Option<T>,

    anti_windup: bool,

    tolerance: Option<T>, // <-- Dead band - tolerance
}

impl <T> Default for PID<T>
where T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default + Clone,
{

    fn default() -> Self {
        PIDBuilder::<T>::default().build()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PIDBuilder<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default + Clone,
{
    pub kp: T,
    pub ki: Option<T>,
    pub kd: Option<T>,
    pub ti: Option<T>,
    pub td: Option<T>,
    pub dt: f32,
    pub output_min: Option<T>,
    pub output_max: Option<T>,
    setpoint_min: Option<T>,
    setpoint_max: Option<T>,
    pub anti_windup: bool,
    tolerance: Option<T>,
}

impl<T> Default for PIDBuilder<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default,
{
    fn default() -> Self {
        PIDBuilder::<T> {
            kp: T::one(),
            ki: None,
            kd: None,
            ti: None,
            td: None,
            dt: 1.0_f32,
            output_min: None,
            output_max: None,
            setpoint_min: None,
            setpoint_max: None,
            anti_windup: false,
            tolerance: None,
        }
    }
}
impl<T> PID<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default,
{
    pub fn builder() -> PIDBuilder<T> {
        PIDBuilder::default()
    }

    pub fn reset(&mut self) {
        self.integral = T::default();
        self.last_error = None;
        self.last_output = None;
    }

    pub fn update(&mut self, input: T, setpoint: T) -> Option<T> {
        let dt = T::from_f32(self.dt).unwrap_or(T::one());

        // Setpoint range check (on/off)
        let on = match (self.setpoint_min, self.setpoint_max) {
            (Some(min), Some(max)) => setpoint >= min && setpoint <= max,
            (Some(min), None) => setpoint >= min,
            (None, Some(max)) => setpoint <= max,
            (None, None) => true,
        };

        if !on && self.anti_windup {
            self.integral = T::default();
        }

        if !on {
            return None;
        }

        let error = setpoint - input;

        // Deadband tolerance check
        if let Some(tol) = self.tolerance {
            if error.abs() <= tol {
                // inside dead band
                return self.last_output;
            }
        }

        // Integral term with mode-dependent anti-windup
        if self.anti_windup {
            let unsat_output = self.kp * error + self.ki * self.integral;
            if self.output_min.map_or(true, |min| unsat_output > min)
                && self.output_max.map_or(true, |max| unsat_output < max)
            {
                self.integral = self.integral + error * dt;
            }
        } else {
            self.integral = self.integral + error * dt;
        }

        let derivative = if let Some(last) = self.last_error {
            (error - last) / dt
        } else {
            T::default()
        };

        self.last_error = Some(error);

        let mut output = self.kp * error + self.ki * self.integral + self.kd * derivative;

        if let Some(min) = self.output_min {
            if output < min {
                output = min;
            }
        }
        if let Some(max) = self.output_max {
            if output > max {
                output = max;
            }
        }

        self.last_output = Some(output);
        Some(output)
    }
}

impl<T> PIDBuilder<T>
where
    T: Num + Signed + PartialOrd + Copy + FromPrimitive + Default,
{
    pub fn kp(mut self, kp: T) -> Self { self.kp = kp; self }
    pub fn ki(mut self, ki: T) -> Self { self.ki = Some(ki); self.ti = None; self }
    pub fn kd(mut self, kd: T) -> Self { self.kd = Some(kd); self.td = None; self }
    // (German Nachstellzeit)
    pub fn reset_time(mut self, ti: T) -> Self { self.ti = Some(ti); self.ki = None; self }
    // (German Vorhaltezeit)
    pub fn hold_time(mut self, td: T) -> Self { self.td = Some(td); self.kd = None; self }
    pub fn sampling_interval(mut self, dt: f32) -> Self { self.dt = dt; self }
    pub fn output_limits(mut self, min: T, max: T) -> Self { self.output_min = Some(min); self.output_max = Some(max); self }
    pub fn anti_windup(mut self, enabled: bool) -> Self { self.anti_windup = enabled; self }
    pub fn setpoint_range(mut self, min: T, max: T) -> Self { self.setpoint_min = Some(min); self.setpoint_max = Some(max); self }
    pub fn tolerance(mut self, tol: T) -> Self { self.tolerance = Some(tol); self }

    pub fn reset_output_limits(self) ->  Self {
        PIDBuilder { output_min: None, output_max: None, anti_windup: false, ..self }
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
            (Some(k), _) =>  self.kp / k,
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
        if self.ti.is_some() && self.kd.is_none() { return true; }
        if self.td.is_some() && self.ki.is_none() { return true; }
        false
    }

    /// Check if output limitting is configured
    pub fn is_output_limited(&self) -> bool {
        match (self.output_max, self.output_min) {
            (None, None) => true,
            (Some(_), Some(_)) => false,
            _ => true, // either upper or lower limit is set - no use case for now
        }
    }

    pub fn build(self) -> PID<T> {
        PID {
            kp: self.kp,
            ki: self.get_ki(),
            kd: self.get_kd(),
            dt: self.dt,
            integral: T::default(),
            last_error: None,
            last_output: None,
            output_min: self.output_min,
            output_max: self.output_max,
            setpoint_min: self.setpoint_min,
            setpoint_max: self.setpoint_max,
            anti_windup: self.anti_windup,
            tolerance: self.tolerance,
        }
    }
}