#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::{
    Async, clock::CpuClock, i2c::master::I2c, rng::Rng, time::Rate, timer::systimer::SystemTimer,
};
use esp_println as _;
use esp_radio::{
    Controller,
    wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState},
};
use getrandom::{Error, register_custom_getrandom};
use libm::round;
use static_cell::StaticCell;
use zenoh_nostd::{keyexpr::borrowed::keyexpr, protocol::core::endpoint::EndPoint};
use zenoh_nostd_embassy::PlatformEmbassy;

use crate::mpu6050::{AccelFullScaleRange, GyroFullScaleRange, MPU6050, SleepMode, TempDisable};

#[panic_handler]
fn panic(panic: &core::panic::PanicInfo) -> ! {
    defmt::error!("Panic: {}", panic);

    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

mod mpu6050;
mod twist;

const SSID: Option<&str> = option_env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASSWORD");
const CONNECT: Option<&str> = option_env!("CONNECT");
const KEYEXPR: Option<&str> = option_env!("KEYEXPR");

const SCALE_Z: Option<&str> = option_env!("SCALE_Z");
const SCALE_Y: Option<&str> = option_env!("SCALE_Y");

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let scale_z: f64 = SCALE_Z.unwrap_or("1.0").parse().unwrap_or(1.0);
    let scale_y: f64 = SCALE_Y.unwrap_or("1.0").parse().unwrap_or(1.0);

    zenoh_nostd::info!("zenoh-nostd DEMO!");

    let (stack, i2c) = init_esp32(spawner).await;
    let mut mpu = MPU6050::new(i2c);
    mpu.write_field(TempDisable::Enable).await.unwrap();
    mpu.write_field(GyroFullScaleRange::FS500).await.unwrap();
    mpu.write_field(AccelFullScaleRange::FS2).await.unwrap();
    mpu.write_field(SleepMode::WakeUp).await.unwrap();

    let mut session = zenoh_nostd::open!(
        zenoh_nostd::zconfig!(
                PlatformEmbassy: (spawner, PlatformEmbassy { stack: stack }),
                TX: 2048,
                RX: 2048,
                SUBSCRIBERS: 2
        ),
        EndPoint::try_from(CONNECT.unwrap_or("udp/192.168.21.90:7447")).unwrap()
    )
    .unwrap();

    let ke: &'static keyexpr = KEYEXPR.unwrap_or("0/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a").try_into().unwrap();
    let mut payload = [0u8; 52];

    let (offset_x, offset_y, _) = mpu.read_accel().await.unwrap();

    loop {
        defmt::debug!("Reading accel...");
        let (acc_x, acc_y, _) = mpu.read_accel().await.unwrap();
        defmt::debug!("Read accel: x={} y={}", acc_x, acc_y);

        let accz = (acc_x - offset_x) as f64 / 16384.0;
        let accy = (acc_y - offset_y) as f64 / 16384.0;

        let accz = round(accz * 100.0) / 100.0;
        let accy = round(accy * 100.0) / 100.0;

        let twist = twist::Twist {
            linear: twist::Vector3 {
                x: accy * scale_y,
                y: 0.0,
                z: 0.0,
            },
            angular: twist::Vector3 {
                x: 0.0,
                y: 0.0,
                z: accz * scale_z,
            },
        };

        defmt::info!(
            "Twist linear.x={} angular.z={}",
            accy * scale_y,
            accz * scale_z
        );

        let size = twist::serialize_twist(&twist, &mut payload);
        defmt::debug!("Putting payload of size {}", size);
        session.put(ke, &payload[..size]).await.unwrap();
        defmt::debug!("Put complete");

        embassy_time::Timer::after(embassy_time::Duration::from_hz(15)).await;
    }
}

register_custom_getrandom!(getrandom_custom);
pub fn getrandom_custom(bytes: &mut [u8]) -> Result<(), Error> {
    Rng::new().read(bytes);
    Ok(())
}

async fn init_esp32(spawner: Spawner) -> (Stack<'static>, I2c<'static, Async>) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_rtos::start(timer0.alarm0);

    defmt::info!("Embassy initialized!");

    let rng = Rng::new();

    let i2c_bus = esp_hal::i2c::master::I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_scl(peripherals.GPIO11)
    .with_sda(peripherals.GPIO10)
    .into_async();

    static RADIO_CTRL: StaticCell<Controller<'static>> = StaticCell::new();
    let radio_ctrl = esp_radio::init().expect("Failed to init radio");

    let (wifi_controller, interfaces) = esp_radio::wifi::new(
        RADIO_CTRL.init(radio_ctrl),
        peripherals.WIFI,
        Default::default(),
    )
    .expect("Failed to initialize WIFI controller");

    let wifi_interface = interfaces.sta;
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    let dhcp_config = DhcpConfig::default();
    let config = embassy_net::Config::dhcpv4(dhcp_config);

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        RESOURCES.init(StackResources::new()),
        net_seed,
    );

    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(runner)).ok();

    defmt::info!("Waiting for link to be up");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    defmt::info!("Waiting to get IP address...");
    let _ip = loop {
        if let Some(config) = stack.config_v4() {
            defmt::info!("Got IP: {}", config.address);
            break config.address;
        }
        Timer::after(Duration::from_millis(500)).await;
    };

    (stack, i2c_bus)
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    defmt::info!("start connection task");
    defmt::info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.unwrap_or("ZettaScale").into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&client_config).unwrap();
            defmt::info!("Starting wifi");
            controller.start_async().await.unwrap();
            defmt::info!("Wifi started!");
        }
        defmt::info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => defmt::info!("Wifi connected!"),
            Err(e) => {
                defmt::info!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
