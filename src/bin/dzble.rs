// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

//! 通过蓝牙 (BLE) 打印一张图片
//!
//! ```bash
//! # 76mm x 130mm 标签, 300dpi (DP305P)
//! cargo run --bin dzble -- --width-mm 76 --height-mm 130 label.png
//!
//! # 指定设备 (广播名是 `型号-序列号`, 不同机器序列号不同, 传型号片段即可)
//! cargo run --bin dzble -- --name DP305P label.png
//! ```
//!
//! 和 `dzprint` (USB) 的区别只有传输层: BLE 走串口透传, 命令流**没有 `0x1e` 包头**,
//! 其余命令与 USB 完全一致。
//!
//! 画质提示: BLE 链路有效带宽约 4.9 KB/s, 打印机会因缺数据而走纸变慢、加热不连续。
//! 想要接近 USB 的效果, 用**最慢速度 + 更高浓度** (例如 `--speed 0 --darkness 11`)。

use std::time::Duration;

use clap::Parser;
use dz_print::{
    backend::{self, ble},
    command::{self, HostCommand},
    image_proc::{
        Bitmap, DitherMode,
        cmd_parser::{BitmapParser, PrintCommand},
    },
};

/// 内容在打印头宽度内的水平摆放
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Align {
    /// 靠打印头左侧
    Left,
    /// 在打印头宽度内居中
    Center,
}

#[derive(Parser, Debug)]
#[command(about = "Print an image on a DeTong label printer over BLE")]
struct Args {
    /// Image file to print (PNG / JPEG / ...)
    image: String,

    /// BLE device-name substring (default: auto-detect `DP*` / `detong`)
    #[arg(long, value_name = "NAME")]
    name: Option<String>,

    /// Content width in millimetres
    #[arg(long, value_name = "MM", default_value_t = 76.0)]
    width_mm: f64,

    /// Content height in millimetres
    #[arg(long, value_name = "MM", default_value_t = 130.0)]
    height_mm: f64,

    /// Vertical DPI used to turn millimetres into dot rows
    #[arg(long, value_name = "DPI", default_value_t = 300)]
    dpi: u32,

    /// Print darkness 0..=14 (device value; default: leave the printer's setting alone)
    #[arg(long, value_name = "N")]
    darkness: Option<u8>,

    /// Print speed 0..=4, 0 = slowest (device value)
    #[arg(long, value_name = "N")]
    speed: Option<u8>,

    /// Use Floyd-Steinberg error diffusion instead of a hard threshold
    #[arg(long)]
    floyd: bool,

    /// Horizontal placement inside the print head width
    #[arg(long, value_enum, default_value_t = Align::Left)]
    align: Align,

    /// Print head width in dots (DP305P = 960)
    #[arg(long, value_name = "DOTS", default_value_t = 960)]
    head_dots: u32,

    /// Explicit left offset in dots; overrides --align (snapped to a multiple of 8)
    #[arg(long, value_name = "DOTS")]
    offset_dots: Option<u32>,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();
    let args = Args::parse();

    let backend = ble::BleBackend::new(args.name.clone()).await?;

    // 纸型换成不干胶 (间隙纸); 没有这个, 0x0c 定位不到标签边界
    send(&backend, command::Command::new_host(HostCommand::GetSetPrintPaperType).package(vec![0x02], false)).await;
    if let Some(d) = args.darkness {
        send(&backend, command::Command::new_host(HostCommand::GetSetPrintDarkness).package(vec![d], false)).await;
    }
    if let Some(s) = args.speed {
        send(&backend, command::Command::new_host(HostCommand::GetSetPrintSpeed).package(vec![s], false)).await;
    }
    // >0x80 的命令需要先激活
    let (cmd, chan) = backend::Command::with_response(
        command::Command::new_host(HostCommand::EnableHighCommand).package(vec![0x7f], false),
    );
    backend.push(cmd).await.ok();
    if let Ok(Some(chan)) = chan.await {
        chan.await.ok();
    }

    let dots_w = (args.width_mm * args.dpi as f64 / 25.4).round() as u32;
    let max_h = (args.height_mm * args.dpi as f64 / 25.4).round() as u32;
    let luma = image::ImageReader::open(&args.image)?.decode()?.into_luma8();
    let mode = if args.floyd {
        DitherMode::FloydSteinberg
    } else {
        DitherMode::Threshold
    };
    let mut bitmap = Bitmap::from_gray_image(&luma, mode);

    // 水平摆放: `0x1f 0x2b` 的前导空白以字节为单位, 偏移要对齐到 8 的倍数。
    // 实测 DP305P: 76mm 纸**居中**于 81.28mm (960 点) 打印头, 纸左边缘 ~dot 31,
    // 所以居中时偏移取 32 点 (2.71mm); 不补会整体偏左、左侧边框被切掉一点。
    let raw_offset = match (args.offset_dots, args.align) {
        (Some(offset), _) => offset,
        (None, Align::Left) => 0,
        (None, Align::Center) => args.head_dots.saturating_sub(bitmap.width()) / 2,
    };
    let offset = ((raw_offset + 4) / 8) * 8;
    if offset > 0 && bitmap.width() + offset <= args.head_dots {
        println!(
            "horizontal offset {} dots ({:.2}mm)",
            offset,
            offset as f64 * 25.4 / args.dpi as f64
        );
        bitmap = bitmap.pad_left(offset);
    } else if offset > 0 {
        eprintln!("offset {offset} dots would exceed the {}-dot head, ignoring", args.head_dots);
    }
    println!(
        "image {}x{} -> target width {} dots, max {} rows @ {}dpi",
        bitmap.width(),
        bitmap.height(),
        dots_w,
        max_h,
        args.dpi
    );

    let (cmd, _) = backend::Command::without_response(
        PrintCommand::ResetPrinter
            .parse()
            .unwrap()
            .into_iter()
            .flatten()
            .collect::<Vec<_>>(),
    );
    backend.push(cmd).await.ok();

    for c in BitmapParser::new(bitmap, 0) {
        if let Some(commands) = c.parse() {
            for bytes in commands {
                let (cmd, _) = backend::Command::without_response(bytes);
                backend.push(cmd).await.ok();
            }
        }
    }

    let (cmd, _) = backend::Command::without_response(
        PrintCommand::NextPaper
            .parse()
            .unwrap()
            .into_iter()
            .flatten()
            .collect::<Vec<_>>(),
    );
    backend.push(cmd).await.ok();

    tokio::time::sleep(Duration::from_secs(10)).await;
    Ok(())
}

/// 发一条不需要响应的命令, 并等它真的写出去
async fn send(backend: &ble::BleBackend, payload: Vec<u8>) {
    let (cmd, chan) = backend::Command::without_response(payload);
    if backend.push(cmd).await.is_ok() {
        chan.await.ok();
    }
}
