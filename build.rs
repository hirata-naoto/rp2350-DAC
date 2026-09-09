//! RP2350 ファームウェアのリンク設定を Cargo へ渡すホスト側ビルドスクリプト。
//!
//! リポジトリの memory.x をコンパイル時に取り込み、Cargo が指定する OUT_DIR へ
//! コピーしてリンカの検索パスに追加する。memory.x の変更時には再実行する。
//! バイナリのリンクには --nmagic、Cortex-M 用 link.x、ログ用 defmt.x を指定し、
//! メモリ配置と起動用セクション、defmt メタデータをリンク処理に反映する。
//! ファームウェア上では実行されず、環境変数の欠落やファイル入出力の失敗は
//! panic としてビルドを中断する。

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

// OUT_DIR にリンカスクリプトを配置し、検索パス・再実行条件・リンク引数を標準出力へ通知する。
// Cargo による起動を前提とし、OUT_DIR の取得、ファイル作成、書き込みの失敗時は panic する。
fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
