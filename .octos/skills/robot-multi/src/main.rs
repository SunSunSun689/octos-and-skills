use serde::Deserialize;
use serde_json::json;
use std::io::Read;
use std::path::PathBuf;
use std::time::Duration;

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
        _ => Err(format!(
            "Unknown action '{cn}'. Valid: 前进/后退/左转/右转/停止/抓取/释放"
        )),
    }
}

fn build_url(ip: &str, action_en: &str, speed: Option<i32>, time: Option<u64>) -> String {
    let mut params = vec![format!("action={}", action_en)];
    if let Some(s) = speed {
        params.push(format!("speed={}", s));
    }
    if let Some(t) = time {
        params.push(format!("time={}", t));
    }
    format!("https://{}/api/control?{}", ip, params.join("&"))
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

    // resolve target list — fail fast on unknown names
    let targets: Vec<RobotEntry> = if input.robots.len() == 1 && input.robots[0] == "all" {
        config.robots
    } else {
        let mut found = Vec::new();
        for name in &input.robots {
            match config.robots.iter().find(|r| r.name == *name) {
                Some(_) => {
                    // find returns a ref; collect names then drain config to avoid double-borrow
                    found.push(name.as_str());
                }
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

    let speed = input.speed;
    let time = input.time;

    let handles: Vec<_> = targets
        .into_iter()
        .map(|robot| {
            let url = build_url(&robot.ip, action_en, speed, time);
            let name = robot.name;
            let ip = robot.ip;
            std::thread::spawn(move || {
                let client = reqwest::blocking::Client::builder()
                    .timeout(Duration::from_secs(10))
                    .build()
                    .unwrap_or_default();
                match client.get(&url).send() {
                    Ok(resp) => match resp.text() {
                        Ok(text) => json!({
                            "robot": name,
                            "ip": ip,
                            "success": true,
                            "response": text.trim().to_string()
                        }),
                        Err(e) => json!({
                            "robot": name,
                            "ip": ip,
                            "success": false,
                            "error": format!("Failed to read response: {e}")
                        }),
                    },
                    Err(e) => json!({
                        "robot": name,
                        "ip": ip,
                        "success": false,
                        "error": format!("HTTP request failed: {e}")
                    }),
                }
            })
        })
        .collect();

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
        json!({
            "output": summary,
            "success": any_success,
            "results": results
        })
    );
}
