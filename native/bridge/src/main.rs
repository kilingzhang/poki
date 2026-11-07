mod apps;
mod ax;
mod configuration;
mod gesture;
mod humanize;
mod installation;
mod pasteboard;
mod remote;
mod stream;

use gesture::{Drag, Pinch, pinch_contacts};
use idevice::{
    IdeviceService, ReadWrite, RsdService,
    core_device::hid::{
        ButtonState, IndigoHidClient, KeyboardUsage, MainKeyboardService,
        TOUCHSCREEN_STATE_CONTACT, TOUCHSCREEN_STATE_RELEASE, UniversalHidServiceClient,
    },
    core_device_proxy::CoreDeviceProxy,
    rsd::RsdHandshake,
    usbmuxd::{UsbmuxdAddr, UsbmuxdConnection},
};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{io::Read, time::Duration};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;
type Touch = UniversalHidServiceClient<Box<dyn ReadWrite>>;
type Indigo = IndigoHidClient<Box<dyn ReadWrite>>;
fn trace(stage: &str) {
    if std::env::var_os("POKI_MCP_NATIVE_TRACE").is_some() {
        static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
        let elapsed = START.get_or_init(std::time::Instant::now).elapsed();
        eprintln!("native stage: {stage} (+{} ms)", elapsed.as_millis());
    }
}
struct Keyboard {
    hid: Touch,
    service: MainKeyboardService,
}
impl Keyboard {
    async fn connect(
        adapter: &mut idevice::tcp::handle::AdapterHandle,
        rsd: &mut RsdHandshake,
    ) -> Result<Self> {
        trace("keyboard connect");
        let mut hid = Touch::connect_rsd(adapter, rsd).await?;
        trace("keyboard create");
        let service = hid.create_main_keyboard().await?;
        trace("keyboard ready");
        tokio::time::sleep(Duration::from_millis(150)).await;
        Ok(Self { hid, service })
    }
    async fn close(&mut self) -> Result<()> {
        self.hid.remove_main_keyboard(&mut self.service).await?;
        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
enum Request {
    DisplayInfo,
    Screenshot {
        path: String,
    },
    DeviceInfo,
    ListApps,
    InstallApp {
        ipa_path: String,
        bundle_id: String,
        staging_id: String,
    },
    UninstallApp {
        bundle_id: String,
    },
    LaunchApp(apps::LaunchRequest),
    Configuration {
        method: String,
        #[serde(default)]
        value: Option<Value>,
    },
    AxTree {
        #[serde(default)]
        include_offscreen: bool,
    },
    Tap {
        x: u16,
        y: u16,
        #[serde(default = "tap_duration")]
        duration_ms: u64,
    },
    DoubleTap {
        x: u16,
        y: u16,
        #[serde(default = "tap_duration")]
        duration_ms: u64,
    },
    LongPress {
        x: u16,
        y: u16,
        #[serde(default = "long_duration")]
        duration_ms: u64,
    },
    Drag {
        x1: u16,
        y1: u16,
        x2: u16,
        y2: u16,
        #[serde(default = "drag_duration")]
        duration_ms: u64,
        #[serde(default)]
        hold_ms: u64,
        #[serde(default)]
        end_hold_ms: u64,
    },
    Pinch {
        x: u16,
        y: u16,
        start_distance: f64,
        end_distance: f64,
        #[serde(default)]
        angle: f64,
        #[serde(default = "drag_duration")]
        duration_ms: u64,
    },
    Key {
        key: String,
        #[serde(default)]
        modifiers: Vec<String>,
        #[serde(default = "one")]
        count: usize,
    },
    Button {
        button: String,
        #[serde(default = "one")]
        count: usize,
        #[serde(default = "button_interval")]
        interval_ms: u64,
    },
    TypeText {
        text: String,
        #[serde(default)]
        clear: bool,
        #[serde(default)]
        submit: bool,
        #[serde(default)]
        target: Option<pasteboard::Target>,
    },
}
fn tap_duration() -> u64 {
    50
}
fn long_duration() -> u64 {
    600
}
fn drag_duration() -> u64 {
    500
}
fn one() -> usize {
    1
}
fn button_interval() -> u64 {
    120
}

impl Request {
    fn validate(&self) -> Result<()> {
        match self {
            Self::LaunchApp(request) => request.validate()?,
            Self::InstallApp { bundle_id, .. } | Self::UninstallApp { bundle_id }
                if bundle_id.is_empty() =>
            {
                return Err("bundle_id must not be empty".into());
            }
            Self::Configuration { method, value } => match method.as_str() {
                "get_increase_contrast" | "get_larger_accessibility_sizes" if value.is_none() => {}
                "set_larger_accessibility_sizes"
                    if value.as_ref().and_then(Value::as_bool).is_some() => {}
                _ => return Err("Invalid configuration method or value".into()),
            },
            Self::Tap { duration_ms, .. }
            | Self::DoubleTap { duration_ms, .. }
            | Self::LongPress { duration_ms, .. }
            | Self::Drag { duration_ms, .. }
            | Self::Pinch { duration_ms, .. }
                if !(1..=60000).contains(duration_ms) =>
            {
                return Err("duration_ms must be 1..60000".into());
            }
            Self::Key {
                key,
                modifiers,
                count,
            } => {
                key_usage(key)?;
                for modifier in modifiers {
                    modifier_usage(modifier)?;
                }
                if !(1..=200).contains(count) {
                    return Err("count must be 1..200".into());
                }
            }
            Self::Button {
                button,
                count,
                interval_ms,
            } => {
                button_usage(button)?;
                if !(1..=200).contains(count) || *interval_ms > 10000 {
                    return Err("Invalid button count or interval_ms".into());
                }
            }
            Self::TypeText { text, .. } if text.len() > 1024 * 1024 => {
                return Err("text exceeds 1 MiB".into());
            }
            _ => {}
        }
        if let Self::Drag {
            hold_ms,
            end_hold_ms,
            ..
        } = self
        {
            if *hold_ms > 10000 || *end_hold_ms > 10000 {
                return Err("hold_ms and end_hold_ms must not exceed 10000".into());
            }
        }
        if let Self::Pinch {
            x,
            y,
            start_distance,
            end_distance,
            angle,
            ..
        } = self
        {
            pinch_contacts(*x, *y, *start_distance, *angle, true)?;
            pinch_contacts(*x, *y, *end_distance, *angle, true)?;
        }
        Ok(())
    }
}

#[tokio::main]
async fn main() {
    let result = async {
        let args: Vec<String> = std::env::args().collect();
        if args.len() == 4 && args[1] == "--remote-server" {
            return remote::serve_socket(&args[2], &args[3]).await;
        }
        if args.len() == 3 && args[1] == "--remote" {
            return remote::serve(&args[2]).await;
        }
        if args.len() != 3 || args[1] != "--udid" {
            return Err("usage: poki-native --udid ID < request.json".into());
        }
        let mut input = String::new();
        std::io::stdin()
            .take(2 * 1024 * 1024)
            .read_to_string(&mut input)?;
        let request: Request = serde_json::from_str(&input)?;
        request.validate()?;
        let limit = if matches!(request, Request::InstallApp { .. }) {
            180
        } else {
            90
        };
        tokio::time::timeout(Duration::from_secs(limit), execute(&args[2], request)).await?
    }
    .await;
    match result {
        Ok(result) => println!("{result}"),
        Err(error) => {
            eprintln!(
                "{}",
                json!({"error": error.to_string(), "screen_change_verified": false})
            );
            std::process::exit(1);
        }
    }
}

async fn connect_device(
    udid: &str,
) -> Result<(
    idevice::tcp::handle::AdapterHandle,
    RsdHandshake,
    idevice::provider::UsbmuxdProvider,
)> {
    let mut mux = UsbmuxdConnection::default().await?;
    let device = mux.get_device(udid).await?;
    let provider = device.to_provider(UsbmuxdAddr::from_env_var()?, "poki-mcp");
    let proxy = CoreDeviceProxy::connect(&provider).await?;
    let port = proxy.tunnel_info().server_rsd_port;
    let mut adapter = proxy.create_software_tunnel()?.to_async_handle();
    let rsd = RsdHandshake::new(adapter.connect(port).await?).await?;
    Ok((adapter, rsd, provider))
}

async fn execute(udid: &str, request: Request) -> Result<Value> {
    if matches!(
        request,
        Request::InstallApp { .. } | Request::UninstallApp { .. }
    ) {
        let result = async {
            let mut mux = UsbmuxdConnection::default().await?;
            let device = mux.get_device(udid).await?;
            let provider = device.to_provider(UsbmuxdAddr::from_env_var()?, "poki-mcp");
            match request {
                Request::InstallApp {
                    ipa_path,
                    bundle_id,
                    staging_id,
                } => {
                    installation::install(
                        &provider,
                        std::path::Path::new(&ipa_path),
                        &bundle_id,
                        &staging_id,
                    )
                    .await
                }
                Request::UninstallApp { bundle_id } => {
                    installation::uninstall(&provider, &bundle_id).await
                }
                _ => unreachable!(),
            }
        }
        .await;
        return Ok(result.unwrap_or_else(
            |error| json!({"isError":true,"outcome":"not_sent","error":error.to_string()}),
        ));
    }
    let connection = connect_device(udid).await;
    let (mut adapter, mut rsd, provider) = match connection {
        Ok(connection) => connection,
        Err(error) if matches!(request, Request::LaunchApp(_)) => return Ok(apps::not_sent(error)),
        Err(error) => return Err(error),
    };
    execute_connected(&mut adapter, &mut rsd, Some(&provider), request).await
}

async fn execute_connected(
    adapter: &mut idevice::tcp::handle::AdapterHandle,
    rsd: &mut RsdHandshake,
    provider: Option<&dyn idevice::provider::IdeviceProvider>,
    request: Request,
) -> Result<Value> {
    if let Request::LaunchApp(request) = request {
        return Ok(apps::launch(adapter, rsd, request)
            .await
            .unwrap_or_else(apps::not_sent));
    }
    if let Request::Screenshot { path } = request {
        let mut dvt =
            idevice::dvt::remote_server::RemoteServerClient::connect_rsd(adapter, rsd).await?;
        dvt.read_message(0).await?;
        let mut screenshot = idevice::dvt::screenshot::ScreenshotClient::new(&mut dvt).await?;
        let bytes = screenshot.take_screenshot().await?;
        use std::io::Write;
        use std::os::unix::fs::OpenOptionsExt;
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(path)?;
        file.write_all(&bytes)?;
        return Ok(json!({"saved":true}));
    }
    if matches!(request, Request::DeviceInfo) {
        let mut lockdown = idevice::lockdown::LockdownClient::connect_rsd(adapter, rsd).await?;
        let scale = lockdown
            .get_value(Some("ScreenScaleFactor"), Some("com.apple.mobile.iTunes"))
            .await?;
        let properties = ["DeviceName", "ProductType", "ProductVersion"]
            .into_iter()
            .filter_map(|key| rsd.properties.get(key).map(|value| (key, value)))
            .collect::<std::collections::HashMap<_, _>>();
        return Ok(json!({"scale":scale,"properties":properties}));
    }
    if matches!(request, Request::ListApps) {
        return apps::list(adapter, rsd, provider).await;
    }
    if let Request::Configuration { method, value } = request {
        return configuration::execute(adapter, rsd, &method, value).await;
    }
    if let Request::AxTree { include_offscreen } = request {
        return ax::read_tree(adapter, rsd, include_offscreen).await;
    }
    if matches!(request, Request::DisplayInfo) {
        let mut display =
            idevice::core_device::DisplayServiceClient::connect_rsd(adapter, rsd).await?;
        let info = display.get_media_support_info().await?;
        return Ok(serde_json::to_value(info)?);
    }
    let mut media = stream::start_screen_media_stream(adapter, rsd, 1)
        .await
        .map_err(|e| format!("Cannot establish input media session: {e}"))?;
    trace("media ready");
    tokio::time::sleep(Duration::from_millis(1000)).await;
    // The device sometimes drops a touch whose connection closes straight after its last report, with no
    // error. The touch connection therefore stays open through the wait that follows the action;
    // NATIVE.md has the counts.
    let mut touch: Option<Touch> = None;
    let mut humanize = humanize::Humanize::from_env();
    let action: Result<Value> = async {
        match request {
            Request::DisplayInfo
            | Request::DeviceInfo
            | Request::Screenshot { .. }
            | Request::ListApps
            | Request::InstallApp { .. }
            | Request::UninstallApp { .. }
            | Request::AxTree { .. }
            | Request::Configuration { .. }
            | Request::LaunchApp(_) => {
                unreachable!()
            }
            Request::Tap { x, y, duration_ms } | Request::LongPress { x, y, duration_ms } => {
                let long_press = matches!(request, Request::LongPress { .. });
                let hid = touch.insert(Touch::connect_rsd(adapter, rsd).await?);
                held_tap(hid, x, y, duration_ms, long_press, &mut humanize).await?;
            }
            Request::DoubleTap { x, y, duration_ms } => {
                let hid = touch.insert(Touch::connect_rsd(adapter, rsd).await?);
                held_tap(hid, x, y, duration_ms, false, &mut humanize).await?;
                let gap = humanize::double_tap_gap_ms(&mut humanize);
                tokio::time::sleep(Duration::from_millis(gap)).await;
                held_tap(hid, x, y, duration_ms, false, &mut humanize).await?;
            }
            Request::Drag {
                x1,
                y1,
                x2,
                y2,
                duration_ms,
                hold_ms,
                end_hold_ms,
            } => {
                let hid = touch.insert(Touch::connect_rsd(adapter, rsd).await?);
                Drag {
                    x1,
                    y1,
                    x2,
                    y2,
                    duration_ms,
                    hold_ms,
                    end_hold_ms,
                }
                .send(hid, &mut humanize)
                .await?;
            }
            Request::Pinch {
                x,
                y,
                start_distance,
                end_distance,
                angle,
                duration_ms,
            } => {
                let hid = touch.insert(Touch::connect_rsd(adapter, rsd).await?);
                Pinch {
                    x,
                    y,
                    start_distance,
                    end_distance,
                    angle,
                    duration_ms,
                }
                .send(hid, &mut humanize)
                .await?;
            }
            Request::Key {
                key,
                modifiers,
                count,
            } => {
                let mut keyboard = Keyboard::connect(adapter, rsd).await?;
                let usages = modifiers
                    .iter()
                    .map(|m| modifier_usage(m))
                    .collect::<Result<Vec<_>>>()?;
                let outcome: Result<()> = async {
                    for index in 0..count {
                        chord(&mut keyboard, key_usage(&key)?, &usages, &mut humanize, index).await?;
                    }
                    Ok(())
                }
                .await;
                let cleanup = keyboard.close().await;
                outcome?;
                cleanup?;
            }
            Request::Button {
                button,
                count,
                interval_ms,
            } => {
                let mut keyboard = Indigo::connect_rsd(adapter, rsd).await?;
                let (code, duration) = button_usage(&button)?;
                for index in 0..count {
                    if index > 0 {
                        let interval = humanize::button_interval_ms(&mut humanize, interval_ms);
                        tokio::time::sleep(Duration::from_millis(interval)).await;
                    }
                    keyboard.send_button(0x0c, code, ButtonState::Down).await?;
                    let hold = humanize::button_hold_ms(&mut humanize, duration);
                    tokio::time::sleep(Duration::from_millis(hold)).await;
                    keyboard.send_button(0x0c, code, ButtonState::Up).await?;
                }
            }
            Request::TypeText {
                text,
                clear,
                submit,
                target,
            } => {
                let mut keyboard = Keyboard::connect(adapter, rsd).await?;
                let outcome = pasteboard::paste_text(
                    adapter,
                    rsd,
                    &mut keyboard,
                    &text,
                    clear,
                    submit,
                    target.as_ref(),
                    &mut humanize,
                )
                .await;
                let cleanup = keyboard.close().await;
                let result = outcome?;
                cleanup?;
                return Ok(result);
            }
        }
        Ok(json!({"sent":true,"screen_change_verified":false}))
    }
    .await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    drop(touch);
    let stop = media.client.stop_media_stream().await;
    drop(media.audio_udp);
    drop(media.video_udp);
    let output = action?;
    stop?;
    Ok(output)
}

async fn held_tap(
    hid: &mut Touch,
    x: u16,
    y: u16,
    ms: u64,
    long_press: bool,
    h: &mut humanize::Humanize,
) -> Result<()> {
    let plan = humanize::tap_plan(h, x, y, ms, long_press);
    hid.send_touchscreen(TOUCHSCREEN_STATE_CONTACT, plan.contact.0, plan.contact.1, None)
        .await?;
    tokio::time::sleep(Duration::from_millis(plan.press_ms)).await;
    hid.send_touchscreen(TOUCHSCREEN_STATE_RELEASE, plan.release.0, plan.release.1, None)
        .await?;
    Ok(())
}

fn modifier_usage(name: &str) -> Result<u64> {
    Ok(match name.to_ascii_lowercase().as_str() {
        "command" | "cmd" | "meta" => 0xe3,
        "control" | "ctrl" => 0xe0,
        "option" | "alt" => 0xe2,
        "shift" => 0xe1,
        _ => return Err(format!("Unknown modifier: {name}").into()),
    })
}

fn key_usage(name: &str) -> Result<u64> {
    let name = name.to_ascii_lowercase();
    Ok(match name.as_str() {
        "return" | "enter" => 0x28,
        "escape" | "esc" => 0x29,
        "backspace" | "delete" => 0x2a,
        "tab" => 0x2b,
        "space" => 0x2c,
        "forward_delete" => 0x4c,
        "right" | "arrowright" => 0x4f,
        "left" | "arrowleft" => 0x50,
        "down" | "arrowdown" => 0x51,
        "up" | "arrowup" => 0x52,
        "home" => 0x4a,
        "end" => 0x4d,
        "pageup" | "page_up" => 0x4b,
        "pagedown" | "page_down" => 0x4e,
        "capslock" | "caps_lock" => 0x39,
        "0" => 0x27,
        name if name.len() == 1 && name.as_bytes()[0].is_ascii_lowercase() => {
            4 + (name.as_bytes()[0] - b'a') as u64
        }
        name if name.len() == 1 && matches!(name.as_bytes()[0], b'1'..=b'9') => {
            0x1e + (name.as_bytes()[0] - b'1') as u64
        }
        name if name.starts_with('f')
            && name[1..]
                .parse::<u64>()
                .is_ok_and(|v| (1..=12).contains(&v)) =>
        {
            0x39 + name[1..].parse::<u64>()?
        }
        name if name.starts_with("0x") => {
            let usage = u64::from_str_radix(&name[2..], 16)?;
            if usage > 0xff {
                return Err("HID keyboard usage exceeds 255".into());
            }
            usage
        }
        _ => return Err(format!("Unknown keyboard key: {name}").into()),
    })
}

fn button_usage(name: &str) -> Result<(u64, u64)> {
    Ok(match name.replace('_', "-").as_str() {
        "home" => (0x40, 80),
        "power" | "lock" | "sleep" => (0x30, 200),
        "volume-up" => (0xe9, 80),
        "volume-down" => (0xea, 80),
        "mute" => (0xe2, 80),
        "siri" => (0xcf, 1200),
        "keyboard" => (0xb8, 80),
        _ => return Err(format!("Unknown hardware button: {name}").into()),
    })
}

async fn chord(
    keyboard: &mut Keyboard,
    key: u64,
    modifiers: &[u64],
    h: &mut humanize::Humanize,
    repeat_index: usize,
) -> Result<()> {
    let pressed = modifiers
        .iter()
        .map(|&v| KeyboardUsage::new(v as u16))
        .collect::<std::result::Result<Vec<_>, _>>()?;
    let key = KeyboardUsage::new(key as u16)?;
    let lead = humanize::modifier_lead_ms(h);
    let hold = humanize::key_hold_ms(h);
    let tail = humanize::modifier_tail_ms(h);
    let gap = if repeat_index == 0 {
        humanize::key_gap_ms(h)
    } else {
        humanize::key_repeat_gap_ms(h, repeat_index)
    };
    let result: Result<()> = async {
        for usage in &pressed {
            keyboard
                .hid
                .main_keyboard_key_down(&mut keyboard.service, *usage)
                .await?;
            tokio::time::sleep(Duration::from_millis(lead)).await;
        }
        keyboard
            .hid
            .main_keyboard_key_down(&mut keyboard.service, key)
            .await?;
        tokio::time::sleep(Duration::from_millis(hold)).await;
        keyboard
            .hid
            .main_keyboard_key_up(&mut keyboard.service, key)
            .await?;
        tokio::time::sleep(Duration::from_millis(tail)).await;
        Ok(())
    }
    .await;
    let release_error = keyboard
        .hid
        .set_main_keyboard_usages(&mut keyboard.service, [])
        .await
        .err();
    result?;
    if let Some(error) = release_error {
        return Err(error.into());
    }
    tokio::time::sleep(Duration::from_millis(gap)).await;
    Ok(())
}
