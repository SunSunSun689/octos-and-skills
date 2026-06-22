use serde::Deserialize;
use serde_json::json;
use std::io::Read;

#[derive(Deserialize)]
struct RobotControlInput {
    ip: String,
    action: String,
    #[serde(default)]
    speed: Option<i32>,
    #[serde(default)]
    time: Option<u64>,
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let tool_name = args.get(1).map(|s| s.as_str()).unwrap_or("unknown");

    let mut buf = String::new();
    if let Err(e) = std::io::stdin().read_to_string(&mut buf) {
        fail(&format!("Failed to read stdin: {e}"));
    }

    match tool_name {
        "robot_control" => handle_robot_control(&buf),
        _ => fail(&format!(
            "Unknown tool '{tool_name}'. Expected: robot_control"
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
            "Unknown action '{cn}'. Valid values: 前进/后退/左转/右转/停止/抓取/释放"
        )),
    }
}

fn build_url(input: &RobotControlInput, action_en: &str) -> String {
    let mut params = vec![format!("action={}", action_en)];

    if let Some(speed) = input.speed {
        params.push(format!("speed={}", speed));
    }

    if let Some(time) = input.time {
        params.push(format!("time={}", time));
    }

    let query = params.join("&");
    format!("http://{}/api/control?{}", input.ip, query)
}

fn handle_robot_control(input_json: &str) {
    let input: RobotControlInput = match serde_json::from_str(input_json) {
        Ok(v) => v,
        Err(e) => fail(&format!("Invalid input: {e}")),
    };

    let action_en = match translate_action(&input.action) {
        Ok(a) => a,
        Err(e) => fail(&e),
    };

    let url = build_url(&input, action_en);

    let resp_text = match reqwest::blocking::get(&url) {
        Ok(resp) => match resp.text() {
            Ok(text) => text,
            Err(e) => fail(&format!("Failed to read response body: {e}")),
        },
        Err(e) => fail(&format!(
            "HTTP request failed (is the car at {} reachable?): {}",
            input.ip, e
        )),
    };

    let output_msg = format!("命令已发送到小车 ({}): {}", input.ip, resp_text.trim());

    println!(
        "{}",
        json!({
            "output": output_msg,
            "success": true,
            "result": {
                "url": url,
                "response": resp_text.trim()
            }
        })
    );
}
