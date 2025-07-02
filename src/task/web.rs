use embassy_time::Duration;
use picoserve::{make_static, routing::get, AppBuilder, AppRouter};

pub const WEB_TASK_POOL_SIZE: usize = 1;

pub struct WebServer<'a> {
    app: &'a AppRouter<AppProps>,
    config: &'a picoserve::Config<Duration>,
}

impl<'a> WebServer<'a> {
    pub fn new() -> Self {
        let app = make_static!(AppRouter<AppProps>, AppProps.build_app());

        let config = make_static!(
            picoserve::Config::<Duration>,
            picoserve::Config::new(picoserve::Timeouts {
                start_read_request: Some(Duration::from_secs(5)),
                persistent_start_read_request: Some(Duration::from_secs(1)),
                read_request: Some(Duration::from_secs(1)),
                write: Some(Duration::from_secs(1)),
            })
            .keep_connection_alive()
        );

        Self { app, config }
    }
}

struct AppProps;

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        picoserve::Router::new().route("/", get(|| async move { "Hello World" }))
    }
}

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
pub async fn web_task(
    id: usize,
    stack: embassy_net::Stack<'static>,
    webserver: &'static WebServer<'static>,
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::listen_and_serve(
        id,
        webserver.app,
        webserver.config,
        stack,
        port,
        &mut tcp_rx_buffer,
        &mut tcp_tx_buffer,
        &mut http_buffer,
    )
    .await
}
