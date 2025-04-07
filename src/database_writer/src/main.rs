use tokio::io::{AsyncBufReadExt, BufReader};
use tokio_serial::{SerialPortBuilderExt, DataBits, FlowControl, Parity, StopBits};
use influxdb::{Client, InfluxDbWriteable, Timestamp};

#[derive(InfluxDbWriteable)]
struct MpuData {
    time: Timestamp,
    #[influxdb(tag)]
    sensor: String,
    pitch: f32,
    roll: f32,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let port_name = "COM4";
    let baud_rate = 115_200;

    let serial = tokio_serial::new(port_name, baud_rate)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .open_native_async()?;

    let reader = BufReader::new(serial);
    let mut lines = reader.lines();

    // ✅ Client InfluxDB basé sur la crate `influxdb`
    let client = Client::new("http://localhost:8086", "robot");

    println!("Lecture série async avec client InfluxDB...");

    while let Ok(Some(line)) = lines.next_line().await {
        if let Some((pitch, roll)) = parse_angles(&line) {
            let data_point = MpuData {
                time: Timestamp,
                sensor: "mpu6050".into(),
                pitch,
                roll,
            };

            match client.query(&data_point.into_query("mpu6050_data")).await {
                Ok(_) => _ ,
                Err(e) => eprintln!("InfluxDB write failed: {:?}", e),
            }
        } else {
            eprintln!("Donnée mal formée : {}", line.trim());
        }
    }

    Ok(())
}

fn parse_angles(line: &str) -> Option<(f32, f32)> {
    let parts: Vec<&str> = line.trim().split(',').collect();
    if parts.len() != 2 {
        return None;
    }

    let pitch = parts[0].parse::<f32>().ok()?;
    let roll = parts[1].parse::<f32>().ok()?;
    Some((pitch, roll))
}
