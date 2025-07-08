use crate::task::web;
use crate::task::web::{web_task, WEB_TASK_POOL_SIZE};
use crate::Spi2Bus;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::{Stack, StackResources};
use embassy_net_wiznet::{chip::W5500, Device, Runner, State};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Duration;
use esp_hal::{
    gpio::{Input, InputConfig, InputPin, Level, Output, OutputConfig, OutputPin, Pull},
    rng::Rng,
    spi::{
        master::{Config as SpiConfig, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Async,
};
use esp_println::println;
use picoserve::io::Write;
use static_cell::StaticCell;

const NET_FREQ: u32 = 40_000_000;

type EthernetSPI =
    SpiDeviceWithConfig<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'static>>;

static STATE: StaticCell<State<8, 8>> = StaticCell::new();
static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
static WEB_SERVER: StaticCell<web::WebServer> = StaticCell::new();

#[embassy_executor::task]
async fn ethernet_task(
    runner: Runner<'static, W5500, EthernetSPI, Input<'static>, Output<'static>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn echo_task(stack: Stack<'static>) -> ! {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        println!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            println!("accept error: {:?}", e);
            continue;
        }
        println!("Received connection from {:?}", socket.remote_endpoint());

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    println!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    println!("{:?}", e);
                    break;
                }
            };
            println!("rxd {}", core::str::from_utf8(&buf[..n]).unwrap());

            if let Err(e) = socket.write_all(&buf[..n]).await {
                println!("write error: {:?}", e);
                break;
            }
        }
    }
}

pub struct EthernetTask {
    mac_addr: [u8; 6],
}

impl EthernetTask {
    pub fn new() -> Self {
        EthernetTask {
            mac_addr: [0x02, 0x00, 0x00, 0x00, 0x00, 0x00],
        }
    }

    pub async fn init(
        &self,
        spawner: Spawner,
        rng: &mut Rng,
        cs: impl OutputPin + 'static,
        rst: impl OutputPin + 'static,
        int: impl InputPin + 'static,
        spi_bus: &'static mut Spi2Bus,
    ) {
        let state = STATE.init(State::<8, 8>::new());

        let net_config = SpiConfig::default()
            .with_frequency(Rate::from_hz(NET_FREQ))
            .with_mode(Mode::_0);
        let spi = SpiDeviceWithConfig::new(
            spi_bus,
            Output::new(cs, Level::High, OutputConfig::default()),
            net_config,
        );

        let int = Input::new(int, InputConfig::default().with_pull(Pull::Up));
        let rst = Output::new(rst, Level::High, OutputConfig::default());
        let (device, wiznet_runner) = embassy_net_wiznet::new(self.mac_addr, state, spi, int, rst)
            .await
            .unwrap();
        spawner.spawn(ethernet_task(wiznet_runner)).unwrap();

        // Generate a random seed for the network stack
        let mut seed = [0; 8];
        rng.read(&mut seed);
        let seed = u64::from_le_bytes(seed);

        // Init network stack
        let (stack, net_runner) = embassy_net::new(
            device,
            embassy_net::Config::dhcpv4(Default::default()),
            RESOURCES.init(StackResources::new()),
            seed,
        );
        spawner.spawn(net_task(net_runner)).unwrap();

        stack.wait_config_up().await;

        println!("Waiting for DHCP...");
        let cfg = self.wait_for_config(stack).await;
        let local_addr = cfg.address.address();
        println!("IP address: {:?}", local_addr);

        let webserver = WEB_SERVER.init(web::WebServer::new());
        for id in 0..WEB_TASK_POOL_SIZE {
            spawner.must_spawn(web_task(id, stack, webserver));
        }

        spawner.must_spawn(echo_task(stack));
    }

    async fn wait_for_config(&self, stack: Stack<'static>) -> embassy_net::StaticConfigV4 {
        loop {
            if let Some(config) = stack.config_v4() {
                return config.clone();
            }
            yield_now().await;
        }
    }
}
