#![no_std]

const PI: f32 = core::f32::consts::PI;
const TAU: f32 = core::f32::consts::TAU;

pub fn sqrtf(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut guess = if x > 1.0 { x } else { 1.0 };
    for _ in 0..8 {
        guess = 0.5 * (guess + x / guess);
    }
    guess
}

pub fn ceilf(x: f32) -> f32 {
    let i = x as i32;
    if x > 0.0 && x != i as f32 {
        (i + 1) as f32
    } else {
        i as f32
    }
}

pub fn sinf(x: f32) -> f32 {
    let x = wrap_pi(x);
    let x2 = x * x;
    let x3 = x2 * x;
    let x5 = x3 * x2;
    let x7 = x5 * x2;
    x - x3 * (1.0 / 6.0) + x5 * (1.0 / 120.0) - x7 * (1.0 / 5040.0)
}

pub fn cosf(x: f32) -> f32 {
    let x = wrap_pi(x);
    let x2 = x * x;
    let x4 = x2 * x2;
    let x6 = x4 * x2;
    1.0 - x2 * 0.5 + x4 * (1.0 / 24.0) - x6 * (1.0 / 720.0)
}

fn wrap_pi(mut x: f32) -> f32 {
    while x > PI {
        x -= TAU;
    }
    while x < -PI {
        x += TAU;
    }
    x
}
