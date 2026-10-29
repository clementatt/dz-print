// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

//! 蓝牙 (BLE) 后端
//!
//! DP305P 的蓝牙其实是"串口透传": 板上是一颗品栋 PDBT33F BLE 模块,
//! 主机把命令流原样写给模块, 模块再灌给打印机串口。所以**命令体与 USB 完全一致**,
//! 唯一的区别是没有 `0x1e` 包头 (那个字节在 USB 上其实是 HID Report ID)。
//!
//! 实测出来的 GATT 布局 (详见 `protocol.md` 的「蓝牙 (BLE)」一节):
//!
//! ```text
//! svc 0000ff00-… : ff02 write            ← 主机→打印机 (命令流, 无包头)
//!                  ff01 notify           ← 打印机→主机 (一条响应一个通知)
//!                  ff03 notify           ← 事件 / 流控信用 (收到写入后回 01 01)
//! ```
//!
//! 流控很重要: 每写一次 `ff02`, 模块会在 `ff03` 上回一个 `01 01`。
//! 如果一口气把几十 KB 灌进去而不等信用, 模块会丢弃数据, 表现为
//! **打印到一半就停**。这里用一个信号量把在飞写入限制在 `WINDOW` 个。
//!
//! 另: BLE 链路实测有效带宽只有 ~4.9 KB/s (瓶颈在模块→打印机那段串口),
//! 打印机会因为缺数据而走纸变慢、画质下降。追求画质时用最慢速度 + 更高浓度。
//!
//! 实测机型: DeTong DP305P (BLE 模块 PingDong PDBT33F, 固件 5.20)。

use std::{collections::VecDeque, sync::Arc, time::Duration};

use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Manager, Peripheral};
use futures::StreamExt;
use thiserror::Error;
use tokio::sync::{broadcast, mpsc, oneshot, Mutex, Semaphore};
use tracing::{debug, info, warn};
use uuid::Uuid;

use super::{Command, CommandResponse};

/// 厂商私有服务 / 特征值
// UUID 按 4-2-2-2-6 分组最易读, 所以这里不采用 clippy 建议的等宽分组
#[allow(clippy::unusual_byte_groupings)]
const SVC_VENDOR: Uuid = Uuid::from_u128(0x0000_ff00_0000_1000_8000_00805f9b34fb);
#[allow(clippy::unusual_byte_groupings)]
const CH_WRITE: Uuid = Uuid::from_u128(0x0000_ff02_0000_1000_8000_00805f9b34fb);
#[allow(clippy::unusual_byte_groupings)]
const CH_REPLY: Uuid = Uuid::from_u128(0x0000_ff01_0000_1000_8000_00805f9b34fb);
#[allow(clippy::unusual_byte_groupings)]
const CH_EVENT: Uuid = Uuid::from_u128(0x0000_ff03_0000_1000_8000_00805f9b34fb);

/// 在飞写入窗口 (实测 8 最好; 调到 32 反而更慢)
const WINDOW: usize = 8;
/// 单次写入上限, 留出 ATT 头部余量 (实测 MTU 515)
const CHUNK: usize = 512;
/// 连接后等模块开始工作的余量
const CONNECT_SETTLE: Duration = Duration::from_millis(200);

#[derive(Error, Debug)]
pub enum BleError {
    #[error("btleplug error: `{0}`")]
    Btle(#[from] btleplug::Error),
    #[error("no bluetooth adapter available")]
    NoAdapter,
    #[error("no matching BLE printer found (scan `--ble-name`)")]
    NotFound,
    #[error("printer `{0}` is missing the vendor service ff00")]
    NoVendorService(String),
    #[error("printer is missing required characteristic `{0}`")]
    MissingCharacteristic(&'static str),
}

/// BLE 后端句柄, 与 `USBBackend` 暴露同一套 `push` 接口
pub struct BleBackend {
    cmd_tx: mpsc::Sender<Command>,
    close_chan: broadcast::Sender<()>,
}

impl BleBackend {
    /// 扫描并连接打印机。
    ///
    /// BLE 广播名与 USB 设备名同构: **`型号-序列号`**, 例如 `DP305P-Y60870279`,
    /// 其中 `Y60870279` 是**出厂序列号, 每台设备都不同**。所以:
    /// - 默认 (`name_hint = None`) 按**型号前缀** `DP` / `DTP` 或 `detong` 识别;
    /// - 传 `--ble-name DP305P` 这种**型号**片段最稳, 不要传完整名字
    ///   (换一台机器序列号就变了)。
    /// - macOS 上 BLE 地址是随机 UUID, 也不能当标识用。
    pub async fn new(name_hint: Option<String>) -> Result<Self, BleError> {
        let (peripheral, device_name) = scan(name_hint.as_deref()).await?;
        info!("BLE: found `{device_name}` ({})", peripheral.address());

        peripheral.connect().await?;
        info!("BLE: connected");
        tokio::time::sleep(CONNECT_SETTLE).await;
        peripheral.discover_services().await?;

        let chars: Vec<Characteristic> = peripheral.characteristics().into_iter().collect();
        if !chars.iter().any(|c| c.service_uuid == SVC_VENDOR) {
            return Err(BleError::NoVendorService(device_name));
        }
        let pick = |uuid: Uuid, what: &'static str| -> Result<Characteristic, BleError> {
            chars
                .iter()
                .find(|c| c.uuid == uuid)
                .cloned()
                .ok_or(BleError::MissingCharacteristic(what))
        };
        let ch_write = pick(CH_WRITE, "ff02 (write)")?;
        let ch_reply = pick(CH_REPLY, "ff01 (notify)")?;
        let ch_event = pick(CH_EVENT, "ff03 (notify)")?;
        debug!(
            "BLE: ff02 props={:?}",
            CharPropFlags::from_bits_truncate(ch_write.properties.bits())
        );

        // 订阅响应与事件
        peripheral.subscribe(&ch_reply).await?;
        peripheral.subscribe(&ch_event).await?;
        let notifications = peripheral.notifications().await?;

        let (cmd_tx, cmd_rx) = mpsc::channel::<Command>(64);
        let (close_chan, _) = broadcast::channel(1);
        tokio::spawn(run(
            peripheral,
            ch_write,
            ch_reply.uuid,
            ch_event.uuid,
            notifications,
            cmd_rx,
            close_chan.subscribe(),
        ));
        Ok(Self { cmd_tx, close_chan })
    }

    /// 推一条命令进发送队列 (与 `USBBackend::push` 语义一致)
    pub async fn push(
        &self,
        cmd: Command,
    ) -> Result<(), tokio::sync::mpsc::error::SendError<Command>> {
        self.cmd_tx.send(cmd).await
    }
}

impl Drop for BleBackend {
    fn drop(&mut self) {
        self.close_chan.send(()).ok();
    }
}

/// 扫描并选出打印机
async fn scan(name_hint: Option<&str>) -> Result<(Peripheral, String), BleError> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters.into_iter().next().ok_or(BleError::NoAdapter)?;
    adapter.start_scan(ScanFilter::default()).await?;

    // 最多扫 10 秒
    for _ in 0..40 {
        for peripheral in adapter.peripherals().await? {
            let props = match peripheral.properties().await? {
                Some(p) => p,
                None => continue,
            };
            let name = props.local_name.unwrap_or_default();
            if name.is_empty() {
                continue;
            }
            let hit = match name_hint {
                Some(hint) => name.contains(hint),
                None => {
                    let low = name.to_lowercase();
                    low.starts_with("dp") || low.contains("detong") || low.contains("dtprint")
                }
            };
            if hit {
                adapter.stop_scan().await.ok();
                return Ok((peripheral, name));
            }
        }
        tokio::time::sleep(Duration::from_millis(250)).await;
    }
    adapter.stop_scan().await.ok();
    Err(BleError::NotFound)
}

type PendingReply = oneshot::Sender<CommandResponse>;

/// 主循环: 一个任务管通知 (响应 + 信用), 一个任务管写入
#[allow(clippy::too_many_arguments)]
async fn run(
    peripheral: Peripheral,
    ch_write: Characteristic,
    reply_uuid: Uuid,
    event_uuid: Uuid,
    notifications: impl futures::Stream<Item = btleplug::api::ValueNotification> + Unpin + Send + 'static,
    mut cmd_rx: mpsc::Receiver<Command>,
    mut close_sig: broadcast::Receiver<()>,
) {
    let pending: Arc<Mutex<VecDeque<PendingReply>>> = Arc::new(Mutex::new(VecDeque::new()));
    let credits = Arc::new(Semaphore::new(WINDOW));
    let (write_tx, mut write_rx) = mpsc::channel::<Vec<u8>>(64);

    // ---- 通知任务: 信用 + 响应 ----
    let pending_rx = pending.clone();
    let credits_rx = credits.clone();
    let notif_task = tokio::spawn(async move {
        let mut notifications = notifications;
        let mut acc: Vec<u8> = Vec::new();
        while let Some(n) = notifications.next().await {
            if n.uuid == event_uuid {
                // `01 01` = 一次写入的流控信用
                if n.value.len() == 2 && n.value[0] == 0x01 && n.value[1] == 0x01 {
                    credits_rx.add_permits(1);
                }
                continue;
            }
            if n.uuid != reply_uuid {
                continue;
            }
            acc.extend_from_slice(&n.value);
            // 一条通知里可能有多条命令, 也可能一条命令跨多个通知
            while let Some((cmd, len)) = super::command::Command::parse_device_command(&acc) {
                acc.drain(..len);
                let mut queue = pending_rx.lock().await;
                match queue.pop_front() {
                    Some(tx) => {
                        tx.send(cmd).ok();
                    }
                    None => debug!("BLE: 收到无人等待的响应, 丢弃"),
                }
            }
        }
        debug!("BLE: notification stream closed");
    });

    // ---- 写入任务: 受窗口限制地写 ff02 ----
    let peripheral_w = peripheral.clone();
    let credits_w = credits.clone();
    let writer = tokio::spawn(async move {
        while let Some(mut bytes) = write_rx.recv().await {
            for chunk in bytes.chunks_mut(CHUNK) {
                // 等一个信用 (窗口满时会阻塞; 信用由通知任务发放)
                let permit = match tokio::time::timeout(
                    Duration::from_secs(5),
                    credits_w.clone().acquire_owned(),
                )
                .await
                {
                    Ok(Ok(p)) => Some(p),
                    Ok(Err(_)) => return,
                    Err(_) => {
                        warn!("BLE: 流控信用超时, 仍然继续写入");
                        None
                    }
                };
                let piece: Vec<u8> = chunk.to_vec();
                if let Err(e) = peripheral_w
                    .write(&ch_write, &piece, WriteType::WithoutResponse)
                    .await
                {
                    warn!("BLE write failed: {e}");
                    break;
                }
                drop(permit);
            }
        }
    });

    // ---- 命令分发 ----
    loop {
        tokio::select! {
            _ = close_sig.recv() => break,
            cmd = cmd_rx.recv() => {
                match cmd {
                    Some(Command::WithResponse(payload, sender)) => {
                        let (tx, rx) = oneshot::channel();
                        sender.send(Some(rx)).ok();
                        pending.lock().await.push_back(tx);
                        if write_tx.send(payload).await.is_err() { break; }
                    }
                    Some(Command::WithoutResponse(payload, sender)) => {
                        let ok = write_tx.send(payload).await.is_ok();
                        sender.send(ok).ok();
                    }
                    Some(Command::Reset(sender)) => {
                        // BLE 上没有"复位设备"的等价物, 直接当成功
                        sender.send(true).ok();
                    }
                    None => break,
                }
            }
        }
    }

    notif_task.abort();
    writer.abort();
    peripheral.disconnect().await.ok();
    debug!("BLE: backend stopped");
}

/// 列出扫描到的 BLE 设备名 (排查用)
pub async fn list_ble_names() -> Result<Vec<String>, BleError> {
    let manager = Manager::new().await?;
    let adapters = manager.adapters().await?;
    let adapter = adapters.into_iter().next().ok_or(BleError::NoAdapter)?;
    adapter.start_scan(ScanFilter::default()).await?;
    tokio::time::sleep(Duration::from_secs(5)).await;
    let mut out = Vec::new();
    for p in adapter.peripherals().await? {
        if let Some(props) = p.properties().await?
            && let Some(name) = props.local_name
        {
            out.push(name);
        }
    }
    adapter.stop_scan().await.ok();
    out.sort();
    out.dedup();
    Ok(out)
}
