//! Human-like timing and trajectory shaping for HID input.
//!
//! Shapes injected input after measured human movement instead of fixed
//! schedules: minimum-jerk position profiles for reaches (Flash & Hogan 1985),
//! Fitts-style duration pull toward human movement times, corrective
//! submovements with an undershoot bias (Meyer et al. 1988; Elliott et al.
//! 2004), flick velocity profiles with early lift-off for swipes and scrolls,
//! curvature-weighted cadence in the spirit of the two-thirds power law,
//! endpoint scatter that scales with movement speed (Schmidt et al. 1979),
//! log-normal press and inter-key intervals with faster same-key bursts, and
//! per-frame deadline jitter so event cadence is never uniform.
//!
//! Every plan function returns the exact legacy values when disabled, so
//! callers keep one code path and tests stay deterministic.

use serde_json::json;

const TAP_PRESS_MEDIAN_MS: f64 = 85.0;
const TAP_PRESS_SIGMA: f64 = 0.30;
const TAP_PRESS_MIN_MS: u64 = 55;
const TAP_PRESS_MAX_MS: u64 = 220;
/// Touch endpoint scatter: ~2 px at 3x (54 HID units per px), clamped to 8 px.
const TAP_START_SIGMA_UNITS: f64 = 110.0;
const TAP_START_MAX_UNITS: f64 = 430.0;
/// Fingertip micro-drift while a tap contact is held down.
const TAP_DRIFT_MIN_UNITS: f64 = 10.0;
const TAP_DRIFT_MAX_UNITS: f64 = 90.0;
const DOUBLE_TAP_GAP_MEDIAN_MS: f64 = 110.0;
const DOUBLE_TAP_GAP_SIGMA: f64 = 0.30;
const DOUBLE_TAP_GAP_MIN_MS: u64 = 70;
const DOUBLE_TAP_GAP_MAX_MS: u64 = 240;

const DRAG_TOTAL_MIN_FACTOR: f64 = 0.88;
const DRAG_TOTAL_MAX_FACTOR: f64 = 1.18;
const DRAG_HOLD_MIN_FACTOR: f64 = 0.9;
const DRAG_HOLD_MAX_FACTOR: f64 = 1.15;
const DRAG_END_HOLD_MIN_FACTOR: f64 = 0.85;
const DRAG_END_HOLD_MAX_FACTOR: f64 = 1.3;
const DRAG_START_SIGMA_UNITS: f64 = 80.0;
const DRAG_FRAME_DT_MIN_FACTOR: f64 = 0.65;
const DRAG_FRAME_DT_MAX_FACTOR: f64 = 1.35;
/// Slow curved path: lateral arc of 0.4%-2% of distance plus layered drift.
const DRAG_ARC_MIN_FRACTION: f64 = 0.004;
const DRAG_ARC_MAX_FRACTION: f64 = 0.02;
const DRAG_MICRO_SIGMA_UNITS: f64 = 9.0;
/// Schmidt's linear speed-accuracy tradeoff: scatter grows with peak speed.
const END_SCATTER_BASE_UNITS: f64 = 30.0;
const END_SCATTER_PER_SPEED_UNITS: f64 = 25.0;
const END_SCATTER_MAX_UNITS: f64 = 500.0;

/// Fitts-style aiming pull: touch movement time grows roughly linearly with
/// log2(1 + distance/D0). The requested duration stays authoritative; the pull
/// only nudges it toward the time a human would need for this distance.
const FITTS_A_MS: f64 = 150.0;
const FITTS_B_MS: f64 = 180.0;
const FITTS_D0_UNITS: f64 = 3000.0;
const FITTS_PULL_EXPONENT: f64 = 0.5;
const FITTS_PULL_MIN: f64 = 0.72;
const FITTS_PULL_MAX: f64 = 1.7;
/// A reach with a hold or a long duration is an aiming drag; a short, fast,
/// unhitched movement is a flick: the finger lifts early and OS inertia
/// finishes the scroll (measured flicks reach peak velocity quickly and then
/// slow down sharply before lift-off).
const FLICK_MAX_DURATION_MS: u64 = 500;
const FLICK_MIN_DISTANCE_UNITS: f64 = 4000.0;
const FLICK_LIFT_MIN_FRACTION: f64 = 0.78;
const FLICK_LIFT_MAX_FRACTION: f64 = 0.95;
const FLICK_PEAK_MIN_FRACTION: f64 = 0.12;
const FLICK_PEAK_MAX_FRACTION: f64 = 0.22;
const FLICK_DECAY_MIN_TAU: f64 = 0.35;
const FLICK_DECAY_MAX_TAU: f64 = 0.55;
const FLICK_ARC_MIN_FRACTION: f64 = 0.002;
const FLICK_ARC_MAX_FRACTION: f64 = 0.008;
/// In-flight corrections after the primary submovement (Meyer et al. 1988;
/// Elliott et al. 2004: humans undershoot more often than they overshoot).
/// The correction covers most but not all of the miss, so the final error
/// stays within the release scatter.
const CORRECT_UNDERSHOOT_P: f64 = 0.25;
const CORRECT_OVERSHOOT_P: f64 = 0.10;
const CORRECT_UNDERSHOOT_MIN: f64 = 0.010;
const CORRECT_UNDERSHOOT_MAX: f64 = 0.050;
const CORRECT_OVERSHOOT_MIN: f64 = 0.004;
const CORRECT_OVERSHOOT_MAX: f64 = 0.020;
const CORRECT_COVER_MIN: f64 = 0.9;
const CORRECT_COVER_MAX: f64 = 1.0;
const CORRECT_LATENCY_MIN_MS: f64 = 50.0;
const CORRECT_LATENCY_MAX_MS: f64 = 120.0;
const CORRECT_MOVE_MIN_MS: f64 = 40.0;
const CORRECT_MOVE_MAX_MS: f64 = 90.0;
/// Two-thirds power law flavor: cadence slows on the tightest path segments
/// (curvature-weighted frame deadlines, capped so minimum-jerk stays dominant).
const CURVATURE_WARP_MAX: f64 = 0.30;
/// Repeated presses of the same key speed up (burst behavior).
const KEY_REPEAT_MEDIAN_FACTOR: f64 = 0.72;
const KEY_REPEAT_DECAY: f64 = 0.94;
const KEY_REPEAT_MIN_MS: u64 = 70;

const PINCH_TOTAL_MIN_FACTOR: f64 = 0.9;
const PINCH_TOTAL_MAX_FACTOR: f64 = 1.15;
const PINCH_NOISE_FRACTION: f64 = 0.01;
const PINCH_CONTACT_SIGMA_UNITS: f64 = 20.0;

const KEY_HOLD_MEDIAN_MS: f64 = 90.0;
const KEY_HOLD_SIGMA: f64 = 0.35;
const KEY_HOLD_MIN_MS: u64 = 55;
const KEY_HOLD_MAX_MS: u64 = 200;
const KEY_GAP_MEDIAN_MS: f64 = 130.0;
const KEY_GAP_SIGMA: f64 = 0.40;
const KEY_GAP_MIN_MS: u64 = 60;
const KEY_GAP_MAX_MS: u64 = 420;
const MODIFIER_LEAD_MIN_MS: u64 = 30;
const MODIFIER_LEAD_MAX_MS: u64 = 70;
const MODIFIER_TAIL_MIN_MS: u64 = 40;
const MODIFIER_TAIL_MAX_MS: u64 = 120;
const BUTTON_INTERVAL_MIN_FACTOR: f64 = 0.8;
const BUTTON_INTERVAL_MAX_FACTOR: f64 = 1.35;
const BUTTON_HOLD_MIN_FACTOR: f64 = 0.85;
const BUTTON_HOLD_MAX_FACTOR: f64 = 1.25;

/// Shared RNG and debug sink for one request. `poki-native` handles a single
/// request per process, so configuration comes from the environment once.
pub(crate) struct Humanize {
    enabled: bool,
    state: u64,
    spare_normal: Option<f64>,
    debug_path: Option<String>,
}

impl Humanize {
    pub(crate) fn off() -> Self {
        Self { enabled: false, state: 0, spare_normal: None, debug_path: None }
    }

    /// Fixed-seed instance for tests that assert on sampled values.
    pub(crate) fn seeded(seed: u64) -> Self {
        Self { enabled: true, state: seed, spare_normal: None, debug_path: None }
    }

    /// Enabled unless POKI_HUMANIZE=off|0. POKI_HUMANIZE_SEED pins the RNG for
    /// reproducible runs; POKI_HUMANIZE_DEBUG=<path> appends plan summaries.
    pub(crate) fn from_env() -> Self {
        let enabled = !matches!(
            std::env::var("POKI_HUMANIZE").as_deref(),
            Ok("off") | Ok("0")
        );
        let state = std::env::var("POKI_HUMANIZE_SEED")
            .ok()
            .and_then(|seed| seed.parse::<u64>().ok())
            .unwrap_or_else(|| {
                let nanos = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos() as u64)
                    .unwrap_or(0x9E3779B97F4A7C15);
                nanos ^ (std::process::id() as u64).rotate_left(32)
            });
        let debug_path = std::env::var("POKI_HUMANIZE_DEBUG").ok().filter(|p| !p.is_empty());
        Self { enabled, state, spare_normal: None, debug_path }
    }

    pub(crate) fn enabled(&self) -> bool {
        self.enabled
    }

    fn next_u64(&mut self) -> u64 {
        // SplitMix64: small, dependency-free, well distributed.
        self.state = self.state.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }

    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / 9_007_199_254_740_992.0)
    }

    fn uniform_range(&mut self, min: f64, max: f64) -> f64 {
        min + (max - min) * self.uniform()
    }

    fn normal(&mut self) -> f64 {
        if let Some(spare) = self.spare_normal.take() {
            return spare;
        }
        // Box-Muller; u1 must be > 0 to avoid log(0). The spare partner uses
        // the SAME angle: sin(angle) is symmetric, while sin(angle / 2) on
        // [0, 2*pi) is always non-negative and would skew every distribution.
        let u1 = self.uniform().max(f64::MIN_POSITIVE);
        let u2 = self.uniform();
        let magnitude = (-2.0 * u1.ln()).sqrt();
        let angle = std::f64::consts::TAU * u2;
        self.spare_normal = Some(magnitude * angle.sin());
        magnitude * angle.cos()
    }

    fn jitter_point(&mut self, x: u16, y: u16, sigma: f64, max_offset: f64) -> (u16, u16) {
        if !self.enabled {
            return (x, y);
        }
        let dx = (self.normal() * sigma).clamp(-max_offset, max_offset);
        let dy = (self.normal() * sigma).clamp(-max_offset, max_offset);
        (clamp_coord(x as f64 + dx), clamp_coord(y as f64 + dy))
    }

    fn lognormal_ms(&mut self, median: f64, sigma: f64, min_ms: u64, max_ms: u64) -> u64 {
        let ms = median * (sigma * self.normal()).exp();
        ms.clamp(min_ms as f64, max_ms as f64).round() as u64
    }

    fn record(&self, summary: serde_json::Value) {
        if let Some(path) = &self.debug_path {
            if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                use std::io::Write as _;
                let _ = writeln!(file, "{summary}");
            }
        }
    }
}

fn clamp_coord(value: f64) -> u16 {
    value.round().clamp(0.0, 65535.0) as u16
}

fn scale_duration(h: &mut Humanize, requested_ms: u64, min: f64, max: f64) -> u64 {
    (requested_ms as f64 * h.uniform_range(min, max))
        .round()
        .clamp(1.0, 60000.0) as u64
}

pub(crate) struct TapPlan {
    pub contact: (u16, u16),
    pub release: (u16, u16),
    pub press_ms: u64,
}

/// A tap or long press: scatter the contact point around the target, hold for
/// a sampled duration, and drift a little before releasing.
pub(crate) fn tap_plan(h: &mut Humanize, x: u16, y: u16, requested_ms: u64, long_press: bool) -> TapPlan {
    if !h.enabled {
        return TapPlan { contact: (x, y), release: (x, y), press_ms: requested_ms };
    }
    let contact = h.jitter_point(x, y, TAP_START_SIGMA_UNITS, TAP_START_MAX_UNITS);
    let drift_angle = h.uniform() * std::f64::consts::TAU;
    let drift = h.uniform_range(TAP_DRIFT_MIN_UNITS, TAP_DRIFT_MAX_UNITS);
    let release = clamp_coord(contact.0 as f64 + drift * drift_angle.cos());
    let release = (release, clamp_coord(contact.1 as f64 + drift * drift_angle.sin()));
    let press_ms = if long_press {
        scale_duration(h, requested_ms, 0.92, 1.10)
    } else {
        h.lognormal_ms(TAP_PRESS_MEDIAN_MS, TAP_PRESS_SIGMA, TAP_PRESS_MIN_MS, TAP_PRESS_MAX_MS)
    };
    h.record(json!({
        "op": if long_press { "long_press" } else { "tap" },
        "request": [x, y, requested_ms],
        "contact": [contact.0, contact.1],
        "release": [release.0, release.1],
        "press_ms": press_ms,
    }));
    TapPlan { contact, release, press_ms }
}

pub(crate) struct DragPlan {
    pub contact: (u16, u16),
    pub release: (u16, u16),
    pub hold_ms: u64,
    pub end_hold_ms: u64,
    /// Contact frames as (x, y, milliseconds after the movement starts).
    pub frames: Vec<(u16, u16, u64)>,
}

/// Minimum-jerk position profile: 10t^3 - 15t^4 + 6t^5 (Flash & Hogan 1985).
fn min_jerk(t: f64) -> f64 {
    let t2 = t * t;
    t2 * t * (10.0 - 15.0 * t + 6.0 * t2)
}

/// Jittered per-step deadlines. `weights` skews the cadence (curvature
/// slowdown); per-frame jitter breaks uniformity; renormalization keeps the
/// last deadline at total_ms with strictly increasing values.
fn jittered_deadlines(h: &mut Humanize, total_ms: u64, steps: usize, weights: &[f64]) -> Vec<u64> {
    let mut dts = Vec::with_capacity(steps);
    let mut weighted_sum = 0.0;
    for index in 0..steps {
        let weight = weights.get(index).copied().unwrap_or(1.0);
        let dt = total_ms as f64 / steps as f64
            * weight
            * h.uniform_range(DRAG_FRAME_DT_MIN_FACTOR, DRAG_FRAME_DT_MAX_FACTOR);
        weighted_sum += dt;
        dts.push(dt);
    }
    let mut deadlines = Vec::with_capacity(steps);
    let mut elapsed = 0.0;
    for dt in &mut dts {
        *dt *= total_ms as f64 / weighted_sum;
        elapsed += *dt;
        deadlines.push((elapsed.round().max(1.0) as u64).min(total_ms));
    }
    // Restore strict monotonicity after rounding (defensive; per-frame gaps
    // are >= ~10 ms so rounding cannot collapse them).
    for index in 1..deadlines.len() {
        if deadlines[index] <= deadlines[index - 1] {
            deadlines[index] = deadlines[index - 1] + 1;
        }
    }
    deadlines
}

/// Curvature-weighted cadence approximation of the two-thirds power law
/// (Lacquaniti et al. 1983): tangential speed drops on tighter segments.
/// Weights stay in [1, 1 + CURVATURE_WARP_MAX], so straight paths keep the
/// plain minimum-jerk cadence.
fn curvature_weights(points: &[(f64, f64)]) -> Vec<f64> {
    if points.len() < 3 {
        return Vec::new();
    }
    let mut curvatures = Vec::with_capacity(points.len());
    for window in points.windows(3) {
        let (a, b, c) = (window[0], window[1], window[2]);
        let ux = b.0 - a.0;
        let uy = b.1 - a.1;
        let vx = c.0 - b.0;
        let vy = c.1 - b.1;
        let d1 = ux.hypot(uy);
        let d2 = vx.hypot(vy);
        let k = if d1 > 1e-9 && d2 > 1e-9 {
            2.0 * (ux * vy - uy * vx).abs() / (d1 * d2 * (d1 + d2))
        } else {
            0.0
        };
        curvatures.push(k);
    }
    let max = curvatures.iter().cloned().fold(0.0, f64::max).max(1e-9);
    let mut weights = vec![1.0];
    for k in &curvatures {
        let normalized = (k / max).clamp(0.0, 1.0);
        weights.push(1.0 + CURVATURE_WARP_MAX * normalized.powf(1.0 / 3.0));
    }
    weights.push(1.0);
    weights
}

fn end_scatter_sigma(speed_units_per_s: f64) -> f64 {
    (END_SCATTER_BASE_UNITS + END_SCATTER_PER_SPEED_UNITS * speed_units_per_s)
        .min(END_SCATTER_MAX_UNITS)
}

/// Aiming reach: Fitts-pulled duration, minimum-jerk chord progress with a
/// lateral arc and drift, curvature-weighted cadence, an optional corrective
/// submovement after the primary reach, and a Schmidt-scattered release.
#[allow(clippy::type_complexity)]
fn aiming_drag(
    h: &mut Humanize,
    x1: u16,
    y1: u16,
    dx: f64,
    dy: f64,
    distance: f64,
    duration_ms: u64,
) -> (u64, Vec<(u16, u16, u64)>, (u16, u16), &'static str, serde_json::Value) {
    let fitts_ms = FITTS_A_MS + FITTS_B_MS * (1.0 + distance / FITTS_D0_UNITS).log2();
    let pull = (fitts_ms / (duration_ms.max(1) as f64))
        .powf(FITTS_PULL_EXPONENT)
        .clamp(FITTS_PULL_MIN, FITTS_PULL_MAX);
    let base_ms = ((duration_ms as f64 * pull) as u64).clamp(80, 60000);
    let total_ms = scale_duration(h, base_ms, DRAG_TOTAL_MIN_FACTOR, DRAG_TOTAL_MAX_FACTOR);

    let arc = distance * h.uniform_range(DRAG_ARC_MIN_FRACTION, DRAG_ARC_MAX_FRACTION)
        * if h.uniform() < 0.5 { -1.0 } else { 1.0 };
    let noise_slow_amplitude = distance * h.uniform_range(0.002, 0.005);
    let noise_fast_amplitude = distance * h.uniform_range(0.001, 0.003);
    let noise_slow_freq = h.uniform_range(0.5, 1.8);
    let noise_fast_freq = h.uniform_range(0.8, 2.6);
    let noise_slow_phase = h.uniform() * std::f64::consts::TAU;
    let noise_fast_phase = h.uniform() * std::f64::consts::TAU;
    // Perpendicular direction of travel.
    let (perp_x, perp_y) = (-dy / distance, dx / distance);

    // Primary submovement plus optional in-flight correction. The correction
    // covers most (undershoot) or part (overshoot pull-back) of the miss, so
    // the residual error stays inside the release scatter.
    let roll = h.uniform();
    let (main_fraction, correction_units) = if roll < CORRECT_UNDERSHOOT_P {
        let miss = distance * h.uniform_range(CORRECT_UNDERSHOOT_MIN, CORRECT_UNDERSHOOT_MAX);
        (1.0 - miss / distance, miss * h.uniform_range(CORRECT_COVER_MIN, CORRECT_COVER_MAX))
    } else if roll < CORRECT_UNDERSHOOT_P + CORRECT_OVERSHOOT_P {
        let over = distance * h.uniform_range(CORRECT_OVERSHOOT_MIN, CORRECT_OVERSHOOT_MAX);
        (1.0 + over / distance, -over * h.uniform_range(0.5, 0.9))
    } else {
        (1.0, 0.0)
    };

    let steps = total_ms.div_ceil(16).max(1) as usize;
    // Idealized path (chord progress + arc) for curvature weighting.
    let ideal: Vec<(f64, f64)> = (0..=steps)
        .map(|index| {
            let t = index as f64 / steps as f64;
            let lateral = arc * (std::f64::consts::PI * t).sin();
            (x1 as f64 + dx * min_jerk(t) + perp_x * lateral, y1 as f64 + dy * min_jerk(t) + perp_y * lateral)
        })
        .collect();
    let weights = curvature_weights(&ideal);
    let deadlines = jittered_deadlines(h, total_ms, steps, &weights);

    // Peak speed of the minimum-jerk profile is 1.875 * amplitude / duration.
    let peak_speed = 1.875 * distance / (total_ms as f64 / 1000.0);

    let mut frames = Vec::with_capacity(steps + 8);
    for (index, &at_ms) in deadlines.iter().enumerate() {
        let t = (index + 1) as f64 / steps as f64;
        let lateral = arc * (std::f64::consts::PI * t).sin()
            + noise_slow_amplitude * (std::f64::consts::TAU * noise_slow_freq * t + noise_slow_phase).sin()
            + noise_fast_amplitude * (std::f64::consts::TAU * noise_fast_freq * t + noise_fast_phase).sin();
        let jitter_x = h.normal() * DRAG_MICRO_SIGMA_UNITS;
        let jitter_y = h.normal() * DRAG_MICRO_SIGMA_UNITS;
        let x = x1 as f64 + dx * min_jerk(t) * main_fraction + perp_x * lateral + jitter_x;
        let y = y1 as f64 + dy * min_jerk(t) * main_fraction + perp_y * lateral + jitter_y;
        frames.push((clamp_coord(x), clamp_coord(y), at_ms));
    }

    let release;
    let correction_record;
    if correction_units.abs() >= 1.0 {
        // Corrective submovement: slower, hence tighter (Schmidt tradeoff).
        let latency = h.uniform_range(CORRECT_LATENCY_MIN_MS, CORRECT_LATENCY_MAX_MS);
        let corr_ms = h.uniform_range(CORRECT_MOVE_MIN_MS, CORRECT_MOVE_MAX_MS);
        let corr_steps = (corr_ms / 16.0).ceil().max(2.0) as usize;
        let corr_peak_speed = 1.875 * correction_units.abs() / (corr_ms / 1000.0);
        let corr_sigma = end_scatter_sigma(corr_peak_speed);
        let mut elapsed = total_ms as f64 + latency;
        let mut last_at = total_ms;
        for step in 1..=corr_steps {
            let t = step as f64 / corr_steps as f64;
            elapsed += corr_ms / corr_steps as f64
                * h.uniform_range(DRAG_FRAME_DT_MIN_FACTOR, DRAG_FRAME_DT_MAX_FACTOR);
            let at = (elapsed.round() as u64).max(last_at + 1);
            last_at = at;
            let jitter_x = h.normal() * DRAG_MICRO_SIGMA_UNITS;
            let jitter_y = h.normal() * DRAG_MICRO_SIGMA_UNITS;
            let x = x1 as f64 + dx * (main_fraction + correction_units / distance * min_jerk(t))
                + jitter_x;
            let y = y1 as f64 + dy * (main_fraction + correction_units / distance * min_jerk(t))
                + jitter_y;
            frames.push((clamp_coord(x), clamp_coord(y), at));
        }
        let (last_x, last_y) = {
            let frame = *frames.last().expect("correction frames appended");
            (frame.0, frame.1)
        };
        release = h.jitter_point(last_x, last_y, corr_sigma, END_SCATTER_MAX_UNITS);
        correction_record = json!({
            "kind": if correction_units > 0.0 { "undershoot" } else { "overshoot" },
            "units": correction_units.round(),
            "latency_ms": latency.round(),
            "move_ms": corr_ms.round(),
        });
    } else {
        // Release lands near the final frame with speed-scaled scatter.
        let (last_x, last_y, _) = frames.last().copied().unwrap_or((x1, y1, total_ms));
        release = h.jitter_point(last_x, last_y, end_scatter_sigma(peak_speed), END_SCATTER_MAX_UNITS);
        correction_record = serde_json::Value::Null;
    }
    (total_ms, frames, release, "aiming", correction_record)
}

/// Flick: contact time is short and roughly independent of distance; velocity
/// peaks early and decays before lift-off, so the release happens mid-gesture
/// and OS inertia carries the content the rest of the way.
#[allow(clippy::type_complexity)]
fn flick_drag(
    h: &mut Humanize,
    x1: u16,
    y1: u16,
    dx: f64,
    dy: f64,
    distance: f64,
    duration_ms: u64,
) -> (u64, Vec<(u16, u16, u64)>, (u16, u16), &'static str, serde_json::Value) {
    let contact_ms = scale_duration(h, duration_ms, 0.85, 1.05);
    let lift = h.uniform_range(FLICK_LIFT_MIN_FRACTION, FLICK_LIFT_MAX_FRACTION);
    let peak_t = h.uniform_range(FLICK_PEAK_MIN_FRACTION, FLICK_PEAK_MAX_FRACTION);
    let tau = h.uniform_range(FLICK_DECAY_MIN_TAU, FLICK_DECAY_MAX_TAU);
    let arc = distance * h.uniform_range(FLICK_ARC_MIN_FRACTION, FLICK_ARC_MAX_FRACTION)
        * if h.uniform() < 0.5 { -1.0 } else { 1.0 };
    let noise_amplitude = distance * h.uniform_range(0.001, 0.003);
    let noise_freq = h.uniform_range(0.8, 2.2);
    let noise_phase = h.uniform() * std::f64::consts::TAU;
    let (perp_x, perp_y) = (-dy / distance, dx / distance);
    let steps = contact_ms.div_ceil(16).max(4) as usize;
    // Normalized velocity shape: smooth rise to peak, exponential decay.
    let mut shape = Vec::with_capacity(steps + 1);
    let mut cumulative = 0.0;
    for index in 0..=steps {
        let t = index as f64 / steps as f64;
        let rise_x = (t / peak_t).min(1.0);
        let rise = rise_x * rise_x * (3.0 - 2.0 * rise_x);
        let decay = (-(t - peak_t).max(0.0) / tau).exp();
        cumulative += rise * decay;
        shape.push(cumulative);
    }
    let total_shape = *shape.last().expect("shape always has steps + 1 entries");
    for value in &mut shape {
        *value /= total_shape;
    }
    let deadlines = jittered_deadlines(h, contact_ms, steps, &[]);
    let mut frames = Vec::with_capacity(steps);
    for (index, &at_ms) in deadlines.iter().enumerate() {
        let t = (index + 1) as f64 / steps as f64;
        let progress = shape[index + 1] * lift;
        let lateral = arc * (std::f64::consts::PI * t).sin()
            + noise_amplitude * (std::f64::consts::TAU * noise_freq * t + noise_phase).sin();
        let jitter_x = h.normal() * DRAG_MICRO_SIGMA_UNITS;
        let jitter_y = h.normal() * DRAG_MICRO_SIGMA_UNITS;
        let x = x1 as f64 + dx * progress + perp_x * lateral + jitter_x;
        let y = y1 as f64 + dy * progress + perp_y * lateral + jitter_y;
        frames.push((clamp_coord(x), clamp_coord(y), at_ms));
    }
    // Numerical peak speed (per-step displacement) for Schmidt scatter.
    let peak_step = shape.windows(2).map(|pair| pair[1] - pair[0]).fold(0.0, f64::max);
    let peak_speed = peak_step * lift * distance / ((contact_ms / steps as u64) as f64 / 1000.0);
    let (last_x, last_y, _) = frames.last().copied().unwrap_or((x1, y1, contact_ms));
    let release = h.jitter_point(last_x, last_y, end_scatter_sigma(peak_speed), END_SCATTER_MAX_UNITS);
    let record = json!({
        "lift_fraction": (lift * 100.0).round() / 100.0,
        "peak_t": (peak_t * 100.0).round() / 100.0,
        "tau": (tau * 100.0).round() / 100.0,
    });
    (contact_ms, frames, release, "flick", record)
}

/// A drag or scroll. Aiming reaches (hold, long duration, or short distance)
/// use minimum-jerk timing with Fitts-style duration pull, a lateral arc,
/// curvature-weighted cadence, and corrective submovements. Fast unhitched
/// movements are flicks: early velocity peak, decay, and lift before the
/// gesture ends. Endpoint scatter grows with peak speed (Schmidt et al. 1979).
pub(crate) fn drag_plan(
    h: &mut Humanize,
    x1: u16,
    y1: u16,
    x2: u16,
    y2: u16,
    duration_ms: u64,
    hold_ms: u64,
    end_hold_ms: u64,
) -> DragPlan {
    if !h.enabled {
        return DragPlan {
            contact: (x1, y1),
            release: (x2, y2),
            hold_ms,
            end_hold_ms,
            frames: Vec::new(),
        };
    }
    let hold = scale_duration(h, hold_ms, DRAG_HOLD_MIN_FACTOR, DRAG_HOLD_MAX_FACTOR);
    let end_hold = scale_duration(h, end_hold_ms, DRAG_END_HOLD_MIN_FACTOR, DRAG_END_HOLD_MAX_FACTOR);
    let contact = h.jitter_point(x1, y1, DRAG_START_SIGMA_UNITS, TAP_START_MAX_UNITS);

    let dx = f64::from(x2) - f64::from(x1);
    let dy = f64::from(y2) - f64::from(y1);
    let distance = dx.hypot(dy).max(1.0);
    let aiming = hold_ms > 0
        || duration_ms > FLICK_MAX_DURATION_MS
        || distance < FLICK_MIN_DISTANCE_UNITS;
    let (total_ms, frames, release, profile, correction) = if aiming {
        aiming_drag(h, x1, y1, dx, dy, distance, duration_ms)
    } else {
        flick_drag(h, x1, y1, dx, dy, distance, duration_ms)
    };

    h.record(json!({
        "op": "drag",
        "request": [x1, y1, x2, y2, duration_ms],
        "profile": profile,
        "contact": [contact.0, contact.1],
        "release": [release.0, release.1],
        "hold_ms": hold,
        "end_hold_ms": end_hold,
        "total_ms": total_ms,
        "frames": frames.len(),
        "correction": correction,
    }));
    DragPlan { contact, release, hold_ms: hold, end_hold_ms: end_hold, frames }
}

pub(crate) struct PinchPlan {
    /// (distance, milliseconds after the gesture starts) per contact frame.
    pub frames: Vec<(f64, u64)>,
}

/// A pinch: minimum-jerk distance progression with smooth noise and jittered
/// frame deadlines; finger positions keep the requested angle and center.
pub(crate) fn pinch_plan(
    h: &mut Humanize,
    start_distance: f64,
    end_distance: f64,
    duration_ms: u64,
) -> PinchPlan {
    if !h.enabled {
        return PinchPlan { frames: Vec::new() };
    }
    let total_ms = scale_duration(h, duration_ms, PINCH_TOTAL_MIN_FACTOR, PINCH_TOTAL_MAX_FACTOR);
    let steps = total_ms.div_ceil(16).max(1) as usize;
    let mut deadlines = Vec::with_capacity(steps);
    let mut sum = 0.0;
    for _ in 0..steps {
        let dt = total_ms as f64 / steps as f64
            * h.uniform_range(DRAG_FRAME_DT_MIN_FACTOR, DRAG_FRAME_DT_MAX_FACTOR);
        sum += dt;
        deadlines.push(dt);
    }
    let mut elapsed = 0.0;
    for dt in &mut deadlines {
        *dt *= total_ms as f64 / sum;
        elapsed += *dt;
        *dt = elapsed.round().max(1.0).min(total_ms as f64);
    }
    let noise_amplitude = (end_distance - start_distance).abs() * PINCH_NOISE_FRACTION
        + start_distance.abs() * PINCH_NOISE_FRACTION * 0.5;
    let noise_freq = h.uniform_range(0.5, 1.8);
    let noise_phase = h.uniform() * std::f64::consts::TAU;
    let mut frames = Vec::with_capacity(steps);
    for (index, &at_ms) in deadlines.iter().enumerate() {
        let t = (index + 1) as f64 / steps as f64;
        let s = 10.0 * t * t * t - 15.0 * t * t * t * t + 6.0 * t * t * t * t * t;
        let noise = noise_amplitude
            * (std::f64::consts::TAU * noise_freq * t + noise_phase).sin();
        let distance = start_distance + (end_distance - start_distance) * s + noise;
        frames.push((distance, at_ms.round() as u64));
    }
    h.record(json!({
        "op": "pinch",
        "request": [start_distance, end_distance, duration_ms],
        "total_ms": total_ms,
        "frames": frames.len(),
    }));
    PinchPlan { frames }
}

/// Sub-pixel nudge for one contact finger of a pinch frame.
pub(crate) fn pinch_contact_jitter(h: &mut Humanize, x: u16, y: u16) -> (u16, u16) {
    if !h.enabled {
        return (x, y);
    }
    h.jitter_point(x, y, PINCH_CONTACT_SIGMA_UNITS, PINCH_CONTACT_SIGMA_UNITS * 3.0)
}

pub(crate) fn key_hold_ms(h: &mut Humanize) -> u64 {
    if !h.enabled { 50 } else { h.lognormal_ms(KEY_HOLD_MEDIAN_MS, KEY_HOLD_SIGMA, KEY_HOLD_MIN_MS, KEY_HOLD_MAX_MS) }
}

pub(crate) fn key_gap_ms(h: &mut Humanize) -> u64 {
    if !h.enabled { 50 } else { h.lognormal_ms(KEY_GAP_MEDIAN_MS, KEY_GAP_SIGMA, KEY_GAP_MIN_MS, KEY_GAP_MAX_MS) }
}

/// Gap before a repeated press of the same key (`press_key` with count > 1):
/// bursts start faster than isolated keystrokes and settle slightly quicker.
pub(crate) fn key_repeat_gap_ms(h: &mut Humanize, repeat_index: usize) -> u64 {
    if !h.enabled {
        50
    } else {
        let shrink = KEY_REPEAT_MEDIAN_FACTOR
            * KEY_REPEAT_DECAY.powi(repeat_index.saturating_sub(1) as i32);
        let median = (KEY_GAP_MEDIAN_MS * shrink).max(KEY_REPEAT_MIN_MS as f64);
        h.lognormal_ms(median, KEY_GAP_SIGMA, KEY_REPEAT_MIN_MS, KEY_GAP_MAX_MS)
    }
}

pub(crate) fn modifier_lead_ms(h: &mut Humanize) -> u64 {
    if !h.enabled { 50 } else { h.uniform_range(MODIFIER_LEAD_MIN_MS as f64, MODIFIER_LEAD_MAX_MS as f64).round() as u64 }
}

pub(crate) fn modifier_tail_ms(h: &mut Humanize) -> u64 {
    if !h.enabled { 50 } else { h.uniform_range(MODIFIER_TAIL_MIN_MS as f64, MODIFIER_TAIL_MAX_MS as f64).round() as u64 }
}

pub(crate) fn double_tap_gap_ms(h: &mut Humanize) -> u64 {
    if !h.enabled {
        80
    } else {
        h.lognormal_ms(DOUBLE_TAP_GAP_MEDIAN_MS, DOUBLE_TAP_GAP_SIGMA, DOUBLE_TAP_GAP_MIN_MS, DOUBLE_TAP_GAP_MAX_MS)
    }
}

pub(crate) fn button_interval_ms(h: &mut Humanize, requested_ms: u64) -> u64 {
    if !h.enabled {
        requested_ms
    } else {
        scale_duration(h, requested_ms, BUTTON_INTERVAL_MIN_FACTOR, BUTTON_INTERVAL_MAX_FACTOR)
    }
}

pub(crate) fn button_hold_ms(h: &mut Humanize, requested_ms: u64) -> u64 {
    if !h.enabled {
        requested_ms
    } else {
        scale_duration(h, requested_ms, BUTTON_HOLD_MIN_FACTOR, BUTTON_HOLD_MAX_FACTOR)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_between(value: u64, min: u64, max: u64, label: &str) {
        assert!(
            (min..=max).contains(&value),
            "{label}: {value} not in [{min}, {max}]"
        );
    }

    #[test]
    fn disabled_plan_is_exact_legacy_passthrough() {
        let mut h = Humanize::off();
        let tap = tap_plan(&mut h, 100, 200, 50, false);
        assert_eq!((tap.contact, tap.release, tap.press_ms), ((100, 200), (100, 200), 50));
        let drag = drag_plan(&mut h, 1, 2, 3, 4, 500, 20, 30);
        assert_eq!(
            (drag.contact, drag.release, drag.hold_ms, drag.end_hold_ms, drag.frames.is_empty()),
            ((1, 2), (3, 4), 20, 30, true)
        );
        let pinch = pinch_plan(&mut h, 100.0, 500.0, 500);
        assert_eq!(pinch.frames.is_empty(), true);
        assert_eq!(key_hold_ms(&mut h), 50);
        assert_eq!(double_tap_gap_ms(&mut h), 80);
    }

    #[test]
    fn seeded_generation_is_deterministic() {
        let plan_a = {
            let mut h = Humanize::seeded(7);
            tap_plan(&mut h, 5000, 5000, 50, false)
        };
        let plan_b = {
            let mut h = Humanize::seeded(7);
            tap_plan(&mut h, 5000, 5000, 50, false)
        };
        assert_eq!(plan_a.contact, plan_b.contact);
        assert_eq!(plan_a.release, plan_b.release);
        assert_eq!(plan_a.press_ms, plan_b.press_ms);
    }

    #[test]
    fn tap_press_ms_stays_in_human_range_and_scatter_stays_bounded() {
        let mut h = Humanize::seeded(11);
        for _ in 0..200 {
            let plan = tap_plan(&mut h, 32768, 32768, 50, false);
            assert_between(plan.press_ms, TAP_PRESS_MIN_MS, TAP_PRESS_MAX_MS, "tap press_ms");
            let offset = (f64::from(plan.contact.0) - 32768.0).hypot(f64::from(plan.contact.1) - 32768.0);
            assert!(offset <= TAP_START_MAX_UNITS * 2.0f64.sqrt(), "scatter {offset} too large");
            let drift = (f64::from(plan.release.0) - f64::from(plan.contact.0))
                .hypot(f64::from(plan.release.1) - f64::from(plan.contact.1));
            assert!(drift <= TAP_DRIFT_MAX_UNITS + 2.0, "drift {drift} too large");
        }
        // A long press keeps the requested magnitude but is never exact.
        let mut seen_varied = false;
        let mut durations = std::collections::HashSet::new();
        for _ in 0..20 {
            let plan = tap_plan(&mut h, 100, 100, 1000, true);
            assert_between(plan.press_ms, 900, 1120, "long press_ms");
            durations.insert(plan.press_ms);
            seen_varied |= durations.len() > 1;
        }
        assert!(seen_varied, "long press durations never vary");
    }

    #[test]
    fn drag_frames_follow_s_curve_with_monotonic_deadlines() {
        let mut h = Humanize::seeded(23);
        let plan = drag_plan(&mut h, 1000, 5000, 20000, 5000, 800, 20, 30);
        assert!(plan.frames.len() >= 40, "too few frames: {}", plan.frames.len());
        // Deadlines are strictly increasing and end near the jittered total.
        let mut last = 0;
        for &(_, _, at_ms) in &plan.frames {
            assert!(at_ms > last, "deadline {at_ms} not after {last}");
            last = at_ms;
        }
        // Fitts pull for 19000 units shortens the 800 ms request toward ~730;
        // the correction appends frames past the main movement.
        assert_between(last, 630, 1000, "drag total");
        // Progress crosses the midpoint near t=0.5 (minimum-jerk is symmetric).
        let half_frames = plan
            .frames
            .iter()
            .filter(|&&(x, _, _)| x >= 10000 && x <= 11000)
            .count();
        assert!(half_frames > 0, "path never crosses the midpoint");
        // All coordinates stay on screen.
        for &(x, y, _) in &plan.frames {
            assert!(x <= 65535 && y <= 65535);
        }
        // Release stays near the requested endpoint: residual correction error
        // (<= 0.5% of distance after coverage), lateral drift at t=1, and the
        // per-axis clamped Schmidt scatter.
        let release_offset =
            (f64::from(plan.release.0) - 20000.0).hypot(f64::from(plan.release.1) - 5000.0);
        assert!(
            release_offset <= 1100.0,
            "release {release_offset} units from the endpoint"
        );
    }

    #[test]
    fn flick_profile_peaks_early_and_lifts_before_the_end() {
        let mut h = Humanize::seeded(67);
        // Swipe shape: 300 ms, no hold, long distance.
        let plan = drag_plan(&mut h, 1000, 30000, 1000, 2000, 300, 0, 0);
        assert!(plan.frames.len() >= 10, "too few flick frames: {}", plan.frames.len());
        let mut last = 0;
        for &(_, _, at_ms) in &plan.frames {
            assert!(at_ms > last, "flick deadline {at_ms} not after {last}");
            last = at_ms;
        }
        assert_between(last, 250, 380, "flick contact time");
        let final_progress = f64::from(30000 - plan.frames.last().unwrap().1) / 28000.0;
        assert!(
            (0.7..=1.0).contains(&final_progress),
            "flick must lift before covering the full distance: {final_progress}"
        );
        // Velocity peaks early: the first third covers more than the last third.
        let third = plan.frames.len() / 3;
        let travelled = |frame: (u16, u16, u64)| f64::from(30000 - frame.1);
        let early = travelled(plan.frames[third]) - travelled(plan.frames[0]);
        let late = travelled(*plan.frames.last().unwrap())
            - travelled(plan.frames[plan.frames.len() - 1 - third]);
        assert!(early > late, "flick should decelerate: early {early} vs late {late}");
    }

    #[test]
    fn aiming_drags_correct_with_an_undershoot_bias() {
        // Count correction records from the debug dump: ~25% undershoot plus
        // ~10% overshoot across seeded runs, undershoot always dominant.
        let dir = std::env::temp_dir().join(format!("poki-humanize-corr-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("debug.jsonl");
        let runs = 120;
        {
            let mut h = Humanize {
                enabled: true,
                state: 71,
                spare_normal: None,
                debug_path: Some(path.to_string_lossy().into_owned()),
            };
            for _ in 0..runs {
                let plan = drag_plan(&mut h, 0, 5000, 30000, 5000, 800, 600, 0);
                let mut last = 0;
                for &(_, _, at_ms) in &plan.frames {
                    assert!(at_ms > last, "deadline {at_ms} not after {last}");
                    last = at_ms;
                }
                // Every plan ends within a small band of the target: the
                // correction covers most of the miss and scatter is clamped.
                let release_offset =
                    (f64::from(plan.release.0) - 30000.0).hypot(f64::from(plan.release.1) - 5000.0);
                assert!(release_offset <= 1100.0, "release {release_offset} units off target");
            }
        }
        let content = std::fs::read_to_string(&path).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        let mut undershoots = 0usize;
        let mut overshoots = 0usize;
        for line in content.lines() {
            let record: serde_json::Value = serde_json::from_str(line).unwrap();
            match record["correction"]["kind"].as_str() {
                Some("undershoot") => undershoots += 1,
                Some("overshoot") => overshoots += 1,
                _ => {}
            }
        }
        let corrections = undershoots + overshoots;
        assert!(
            corrections >= runs / 4 && corrections <= runs / 2,
            "correction rate {corrections}/{runs} outside the expected band"
        );
        assert!(undershoots > overshoots, "humans undershoot more than they overshoot");
    }

    #[test]
    fn fitts_pull_orders_long_reaches_after_short_ones() {
        let mut short_totals = Vec::new();
        let mut long_totals = Vec::new();
        for seed in 100..140 {
            let mut h = Humanize::seeded(seed);
            let short = drag_plan(&mut h, 0, 5000, 5000, 5000, 800, 600, 0);
            let mut h = Humanize::seeded(seed);
            let long = drag_plan(&mut h, 0, 5000, 45000, 5000, 800, 600, 0);
            short_totals.push(short.frames.last().unwrap().2);
            long_totals.push(long.frames.last().unwrap().2);
        }
        let mean = |values: &[u64]| values.iter().sum::<u64>() as f64 / values.len() as f64;
        assert!(
            mean(&long_totals) > mean(&short_totals) + 80.0,
            "Fitts pull did not separate distances: short {} vs long {}",
            mean(&short_totals),
            mean(&long_totals)
        );
    }

    #[test]
    fn curvature_weights_slow_tight_segments_and_keep_straight_paths_plain() {
        let straight = vec![(0.0, 0.0), (100.0, 0.0), (200.0, 0.0), (300.0, 0.0)];
        let weights = curvature_weights(&straight);
        assert_eq!(weights, vec![1.0, 1.0, 1.0, 1.0]);
        let arced = vec![(0.0, 0.0), (100.0, 8.0), (200.0, 24.0), (300.0, 30.0)];
        let weights = curvature_weights(&arced);
        assert_eq!(weights.len(), 4);
        let max_weight = weights.iter().cloned().fold(1.0, f64::max);
        assert!(max_weight > 1.05, "arc weight {max_weight} did not slow the curve");
        let min_weight = weights.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((1.0..=1.0 + CURVATURE_WARP_MAX).contains(&min_weight));
    }

    #[test]
    fn repeat_bursts_start_faster_and_stay_in_bounds() {
        let mut h = Humanize::seeded(83);
        let sample_median = |samples: &mut Vec<u64>| {
            samples.sort_unstable();
            samples[samples.len() / 2]
        };
        // Log-normal gaps are heavy-tailed; medians, not means, carry the
        // ordering signal.
        let mut isolated = Vec::new();
        let mut first_repeat = Vec::new();
        let mut settled_repeat = Vec::new();
        for _ in 0..240 {
            isolated.push(key_gap_ms(&mut h));
            first_repeat.push(key_repeat_gap_ms(&mut h, 1));
            settled_repeat.push(key_repeat_gap_ms(&mut h, 4));
        }
        for &(index, value) in &[
            ("isolated", sample_median(&mut isolated)),
            ("first repeat", sample_median(&mut first_repeat)),
            ("settled repeat", sample_median(&mut settled_repeat)),
        ] {
            assert_between(value, KEY_REPEAT_MIN_MS, KEY_GAP_MAX_MS, index);
        }
        let isolated_median = sample_median(&mut isolated);
        let first_median = sample_median(&mut first_repeat);
        let settled_median = sample_median(&mut settled_repeat);
        assert!(
            first_median < isolated_median,
            "first repeat median {first_median} not faster than isolated {isolated_median}"
        );
        assert!(
            settled_median <= first_median,
            "repeat bursts should accelerate: settled {settled_median} > first {first_median}"
        );
    }

    #[test]
    fn drag_s_curve_is_not_linear_in_time() {
        // A linear ramp at t=0.25 covers 25% of distance; minimum-jerk covers
        // ~10.4% (10*0.25^3 - 15*0.25^4 + 6*0.25^5 = 0.1035). Assert the
        // sampled early progress stays clearly below the linear share.
        let mut h = Humanize::seeded(31);
        let plan = drag_plan(&mut h, 0, 5000, 30000, 5000, 800, 0, 0);
        let quarter_index = plan.frames.len() / 4;
        let (x, _, _) = plan.frames[quarter_index];
        let progress = f64::from(x) / 30000.0;
        assert!(
            progress < 0.20,
            "early progress {progress} looks linear, not minimum-jerk"
        );
    }

    #[test]
    fn pinch_plan_produces_monotonic_deadlines_and_bounded_noise() {
        let mut h = Humanize::seeded(41);
        let plan = pinch_plan(&mut h, 400.0, 1600.0, 500);
        let mut last = 0;
        let mut min_distance = f64::INFINITY;
        let mut max_distance = f64::NEG_INFINITY;
        for &(distance, at_ms) in &plan.frames {
            assert!(at_ms > last, "pinch deadline {at_ms} not after {last}");
            last = at_ms;
            min_distance = min_distance.min(distance);
            max_distance = max_distance.max(distance);
        }
        assert_between(last, 430, 600, "pinch total");
        // Noise stays within a small band around the start-end span.
        assert!(min_distance >= 350.0, "pinch distance {min_distance} too low");
        assert!(max_distance <= 1650.0, "pinch distance {max_distance} too high");
    }

    #[test]
    fn key_timing_stays_in_human_ranges() {
        let mut h = Humanize::seeded(53);
        for _ in 0..100 {
            assert_between(key_hold_ms(&mut h), KEY_HOLD_MIN_MS, KEY_HOLD_MAX_MS, "key hold");
            assert_between(key_gap_ms(&mut h), KEY_GAP_MIN_MS, KEY_GAP_MAX_MS, "key gap");
            assert_between(
                double_tap_gap_ms(&mut h),
                DOUBLE_TAP_GAP_MIN_MS,
                DOUBLE_TAP_GAP_MAX_MS,
                "double tap gap",
            );
        }
        assert_between(
            button_interval_ms(&mut h, 120),
            (120.0 * BUTTON_INTERVAL_MIN_FACTOR) as u64,
            (120.0 * BUTTON_INTERVAL_MAX_FACTOR).ceil() as u64,
            "button interval",
        );
    }

    #[test]
    fn normal_samples_are_symmetric_around_zero() {
        // Guards the Box-Muller spare pairing: a non-symmetric spare (for
        // example sin(angle/2)) shifts every log-normal median upward.
        let mut h = Humanize::seeded(97);
        let mut values = Vec::new();
        for _ in 0..4000 {
            values.push(h.normal());
        }
        values.sort_by(f64::total_cmp);
        let median = values[values.len() / 2];
        assert!(median.abs() < 0.08, "normal median {median} not centered on zero");
        let mean: f64 = values.iter().sum::<f64>() / values.len() as f64;
        assert!(mean.abs() < 0.08, "normal mean {mean} not centered on zero");
        // Positive and negative draws must be roughly balanced.
        let positive = values.iter().filter(|&&v| v > 0.0).count();
        assert!(
            (1800..=2200).contains(&positive),
            "positive share {positive}/4000 not balanced"
        );
    }

    #[test]
    fn debug_dump_writes_json_line() {
        let dir = std::env::temp_dir().join(format!("poki-humanize-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("debug.jsonl");
        let mut h = Humanize { enabled: true, state: 99, spare_normal: None, debug_path: Some(path.to_string_lossy().into_owned()) };
        let _ = tap_plan(&mut h, 100, 100, 50, false);
        let content = std::fs::read_to_string(&path).unwrap();
        assert!(content.contains("\"op\":\"tap\""), "missing tap record: {content}");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}

