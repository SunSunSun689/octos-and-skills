use serde::Deserialize;
use serde_json::json;
use std::io::Read;
use std::path::PathBuf;
use std::time::Duration;
use native_tls::TlsConnector;
use tungstenite::Message;

#[derive(Deserialize)]
struct RobotMultiControlInput {
    robots: Vec<String>,
    action: String,
    #[serde(default)]
    speed: Option<i32>,
    #[serde(default)]
    time: Option<u64>,
}

#[derive(Deserialize)]
struct RobotEntry {
    name: String,
    ip: String,
}

#[derive(Deserialize)]
struct RobotsConfig {
    robots: Vec<RobotEntry>,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let tool_name = args.get(1).map(|s| s.as_str()).unwrap_or("unknown");

    let mut buf = String::new();
    if let Err(e) = std::io::stdin().read_to_string(&mut buf) {
        fail(&format!("Failed to read stdin: {e}"));
    }

    match tool_name {
        "robot_multi_control" => handle_robot_multi_control(&buf),
        _ => fail(&format!(
            "Unknown tool '{tool_name}'. Expected: robot_multi_control"
        )),
    }
}

fn fail(msg: &str) -> ! {
    println!("{}", json!({"output": msg, "success": false}));
    std::process::exit(1);
}

fn translate_action(cn: &str) -> Result<&'static str, String> {
    match cn {
        "前进" | "up" => Ok("up"),
        "后退" | "down" => Ok("down"),
        "左转" | "left" => Ok("left"),
        "右转" | "right" => Ok("right"),
        "停止" | "stop" => Ok("stop"),
        "抓取" | "grab" => Ok("grab"),
        "释放" | "release" => Ok("release"),
        "捡球" | "捡网球" | "pickup_tennis" => Ok("pickup_tennis"),
        "停止捡球" | "stop_demo" => Ok("stop_demo"),
        _ => Err(format!(
            "Unknown action '{cn}'. Valid: 前进/后退/左转/右转/停止/抓取/释放/捡球/停止捡球"
        )),
    }
}

// Returns (steering, throttle) as signed i8
// steering: negative=left, positive=right, 0=straight
// throttle: positive=forward, negative=backward, 0=stop
fn action_to_joystick(action: &str, speed: i32) -> (i8, i8) {
    let s = speed.clamp(0, 127) as i8;
    match action {
        "up"      => (0,  s),
        "down"    => (0, -s),
        "left"    => (-s, 0),
        "right"   => ( s, 0),
        _         => (0,  0),
    }
}

// Send joystick command via WebSocket. Protocol: [0xAA, steering_i8, throttle_i8]
// Keeps connection open for `hold_ms` then sends stop, draining server heartbeats.
fn ws_drive(ip: &str, steering: i8, throttle: i8, hold_ms: u64) -> Result<(), String> {
    let tcp = std::net::TcpStream::connect(format!("{ip}:443"))
        .map_err(|e| format!("TCP connect error: {e}"))?;
    tcp.set_read_timeout(Some(Duration::from_millis(500))).ok();
    tcp.set_write_timeout(Some(Duration::from_secs(5))).ok();

    let tls = TlsConnector::builder()
        .danger_accept_invalid_certs(true)
        .build()
        .map_err(|e| format!("TLS build error: {e}"))?;
    let tls_stream = tls.connect(ip, tcp)
        .map_err(|e| format!("TLS handshake error: {e}"))?;

    let url = format!("wss://{}/ws/control", ip);
    let (mut ws, _) = tungstenite::client(url, tls_stream)
        .map_err(|e| format!("WS handshake error: {e}"))?;

    // send drive command
    let msg = vec![0xAAu8, steering as u8, throttle as u8];
    ws.send(Message::Binary(msg.into()))
        .map_err(|e| format!("WS send error: {e}"))?;

    // hold connection open, drain incoming heartbeats
    let deadline = std::time::Instant::now() + Duration::from_millis(hold_ms);
    while std::time::Instant::now() < deadline {
        match ws.read() {
            Ok(_) => {}
            Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => break,
        }
    }

    // send stop
    let stop = vec![0xAAu8, 0u8, 0u8];
    ws.send(Message::Binary(stop.into())).ok();

    // drain briefly so server sees stop before we drop
    let drain_until = std::time::Instant::now() + Duration::from_millis(200);
    while std::time::Instant::now() < drain_until {
        match ws.read() {
            Ok(_) => {}
            Err(_) => break,
        }
    }

    Ok(())
}

// Hold WebSocket connection open for `hold_ms` so demo can drive motors.
// Drains incoming messages without sending anything.
fn ws_hold(ip: &str, hold_ms: u64) -> Result<(), String> {
    let tcp = std::net::TcpStream::connect(format!("{ip}:443"))
        .map_err(|e| format!("TCP connect error: {e}"))?;
    tcp.set_read_timeout(Some(Duration::from_millis(500))).ok();
    tcp.set_write_timeout(Some(Duration::from_secs(5))).ok();

    let tls = TlsConnector::builder()
        .danger_accept_invalid_certs(true)
        .build()
        .map_err(|e| format!("TLS build error: {e}"))?;
    let tls_stream = tls.connect(ip, tcp)
        .map_err(|e| format!("TLS handshake error: {e}"))?;

    let url = format!("wss://{}/ws/control", ip);
    let (mut ws, _) = tungstenite::client(url, tls_stream)
        .map_err(|e| format!("WS handshake error: {e}"))?;

    let deadline = std::time::Instant::now() + Duration::from_millis(hold_ms);
    while std::time::Instant::now() < deadline {
        match ws.read() {
            Ok(_) => {}
            Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(_) => break,
        }
    }
    Ok(())
}

fn load_robots_config() -> RobotsConfig {
    let binary_path = std::env::current_exe().unwrap_or_else(|_| PathBuf::from("."));
    let config_path = binary_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new("."))
        .join("robots.json");

    let content = std::fs::read_to_string(&config_path).unwrap_or_else(|e| {
        fail(&format!(
            "Failed to read robots.json at {}: {}",
            config_path.display(),
            e
        ))
    });

    serde_json::from_str(&content).unwrap_or_else(|e| {
        fail(&format!("Invalid robots.json format: {e}"))
    })
}

fn handle_robot_multi_control(input_json: &str) {
    let input: RobotMultiControlInput = match serde_json::from_str(input_json) {
        Ok(v) => v,
        Err(e) => fail(&format!("Invalid input: {e}")),
    };

    let action_en = match translate_action(&input.action) {
        Ok(a) => a,
        Err(e) => fail(&e),
    };

    let config = load_robots_config();

    // resolve target list
    let targets: Vec<RobotEntry> = if input.robots.len() == 1 && input.robots[0] == "all" {
        config.robots
    } else {
        let mut found = Vec::new();
        for name in &input.robots {
            match config.robots.iter().find(|r| r.name == *name) {
                Some(_) => found.push(name.as_str()),
                None => fail(&format!("Robot '{}' not found in robots.json", name)),
            }
        }
        config
            .robots
            .into_iter()
            .filter(|r| found.contains(&r.name.as_str()))
            .collect()
    };

    if targets.is_empty() {
        fail("No robots found. Check robots.json or the robots parameter.");
    }

    // demo actions: pickup_tennis / stop_demo
    if action_en == "pickup_tennis" || action_en == "stop_demo" {
        let is_pickup = action_en == "pickup_tennis";
        let handles: Vec<_> = targets
            .into_iter()
            .map(|robot| {
                let name = robot.name;
                let ip = robot.ip.clone();
                std::thread::spawn(move || {
                    let client = reqwest::blocking::Client::builder()
                        .timeout(Duration::from_secs(10))
                        .no_proxy()
                        .danger_accept_invalid_certs(true)
                        .build()
                        .unwrap_or_default();
                    if is_pickup {
                        // stop any existing demo first
                        let _ = client.post(format!("https://{}/api/demo/stop", ip))
                            .json(&json!({})).send();
                        std::thread::sleep(Duration::from_millis(500));

                        // start tennis demo
                        let url = format!("https://{}/api/demo/init", ip);
                        match client.post(&url).json(&json!({"name": "tennis"})).send() {
                            Err(e) => return json!({"robot": name, "ip": ip, "success": false, "error": format!("HTTP error: {e}")}),
                            Ok(resp) => {
                                let text = resp.text().unwrap_or_default();
                                if text.contains("error") {
                                    return json!({"robot": name, "ip": ip, "success": false, "error": text});
                                }
                            }
                        }
                        std::thread::sleep(Duration::from_millis(300));

                        // hold WebSocket connection so demo can drive motors (80s timeout)
                        match ws_hold(&ip, 80_000) {
                            Ok(()) => json!({"robot": name, "ip": ip, "success": true, "response": "tennis demo completed"}),
                            Err(e) => json!({"robot": name, "ip": ip, "success": false, "error": e}),
                        }
                    } else {
                        let url = format!("https://{}/api/demo/stop", ip);
                        match client.post(&url).json(&json!({})).send() {
                            Ok(resp) => json!({"robot": name, "ip": ip, "success": true, "response": resp.text().unwrap_or_default().trim().to_string()}),
                            Err(e) => json!({"robot": name, "ip": ip, "success": false, "error": format!("HTTP error: {e}")}),
                        }
                    }
                })
            })
            .collect();
        return finish(handles);
    }

    let speed = input.speed.unwrap_or(80);
    let time_ms = input.time.unwrap_or(2000);
    let (steering, throttle) = action_to_joystick(action_en, speed);

    let handles: Vec<_> = targets
        .into_iter()
        .map(|robot| {
            let name = robot.name;
            let ip = robot.ip;
            std::thread::spawn(move || {
                match ws_drive(&ip, steering, throttle, time_ms) {
                    Ok(()) => json!({"robot": name, "ip": ip, "success": true, "response": "ok"}),
                    Err(e) => json!({"robot": name, "ip": ip, "success": false, "error": e}),
                }
            })
        })
        .collect();

    finish(handles);
}

fn finish(handles: Vec<std::thread::JoinHandle<serde_json::Value>>) {
    let results: Vec<serde_json::Value> = handles
        .into_iter()
        .map(|h| h.join().unwrap_or_else(|_| json!({"success": false, "error": "thread panicked"})))
        .collect();

    let any_success = results.iter().any(|r| r["success"].as_bool().unwrap_or(false));

    let summary = results
        .iter()
        .map(|r| {
            let name = r["robot"].as_str().unwrap_or("?");
            if r["success"].as_bool().unwrap_or(false) {
                format!("{}: 成功", name)
            } else {
                let err = r["error"].as_str().unwrap_or("未知错误");
                format!("{}: 失败({})", name, err)
            }
        })
        .collect::<Vec<_>>()
        .join(", ");

    println!(
        "{}",
        json!({"output": summary, "success": any_success, "results": results})
    );
}
