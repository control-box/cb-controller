# PID Controller (no-std compatible)

A generic **PID controller** implementation in Rust, designed for embedded and real-time systems.  
Supports both `no_std` and `std` environments.  

Fixed point numbers are provided as well and can be used with the PID-Controller.
They are usefull at embedded systems where no floating point unit is available.

## Features

- **no-std** compatible (no allocations, only `core`)
- Generic over the numeric type `T`
  - Use `f32`/`f64` for floating-point
  - Use your own **fixed-point types** for deterministic embedded applications
- Configurable via a **Builder Pattern**
- Optional **output limits** (saturation)
- Optional **anti-windup** handling
- Optional **tolerance band** (deadband around the setpoint)
- Supports **Ti / Td** parameterization in addition to `Ki / Kd`
- Optional **setpoint range** to enable/disable the controller automatically

## Example

PID controller using fixed point arithmetics.

```rust
use cb_controller::pid::PIDBuilder;
use cb_controller::Fp9_7;  // fixed point number type; 9 bit before and 7 bit after the point

fn main() {
    // PID controller with floating point
    let mut pid = PIDBuilder::<Fp9_7>default()
        .kp(Fp9_7::from_num(2.0))
        .ti(1.0)               // use Ti -> internally computes Ki
        .td(0.1)               // use Td -> internally computes Kd
        .dt(0.01)              // fixed sampling time (10 ms)
        .output_limits(-10.0, 10.0)
        .anti_windup(true)
        .tolerance(0.05)       // deadband around setpoint
        .build();

    let setpoint = 1.0;
    let mut process_variable = 0.0;

    for _ in 0..100 {
        let output = pid.update(setpoint, process_variable);
        // feed output to plant (here: simple example)
        let time_delay_in_sec = 0.01_f32;
        process_variable += output *time_delay_in_ms;
        println!("PV: {process_variable:.3}, OUT: {output:.3}");
    }
}

```

### Development

- main branch is used to build the app and publish it at gh-pages
- ci only checks for code sanity (build, test, doc/doc-test)

## License

MIT — see [`LICENSE.md`](LICENSE.md)


