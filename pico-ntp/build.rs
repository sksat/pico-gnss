//! memory.x をリンカ検索パス (OUT_DIR) に置く。
//!
//! cortex-m-rt の link.x が `INCLUDE memory.x` するため、リンカが memory.x を見つけられる
//! 必要がある。CWD 依存 (workspace ではビルド CWD が member 外になり得る) を避けるため、
//! OUT_DIR にコピーして `-L` で渡す (cortex-m-quickstart 標準パターン)。
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    // 測定用に送信先ポートを変えたとき、焼き直しても古いバイナリのままにならないように。
    println!("cargo:rerun-if-env-changed=NTP_DST_PORT");
}
