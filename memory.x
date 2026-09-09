/*
 * RP2350 ファームウェアのメモリ領域と起動メタデータを定義するリンカスクリプト。
 * build.rs が OUT_DIR にコピーし、Cortex-M の link.x と組み合わせて使用する。
 * FLASH は 0x10000000 から 2 MiB、メインRAM は 0x20000000 から 512 KiB、
 * SRAM8/9 はそれぞれ独立した 4 KiB 領域として宣言する。
 * 起動ブロックをベクタテーブル直後、バイナリ情報を .text の後、
 * 終端ブロックを .uninit の後に挿入し、各ブロックを 4 バイト境界へ揃える。
 * KEEP は未参照セクションの削除を防ぎ、起動に必要な情報を保持する。
 * 開始・終端アドレスと相対距離を公開し、起動ブロックを避けてコード開始位置を指定する。
 */

MEMORY {
    /* 実行コード、定数、起動情報を配置するフラッシュ領域。 */
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    /* 通常のデータ、スタックなどに用いるメインSRAM 領域。 */
    RAM : ORIGIN = 0x20000000, LENGTH = 512K
    /* 個別配置用の SRAM バンク 8。本スクリプトでは専用セクションを割り当てない。 */
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K
    /* 個別配置用の SRAM バンク 9。本スクリプトでは専用セクションを割り当てない。 */
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K
}

SECTIONS {
    .start_block : ALIGN(4)
    {
        /* 起動ブロックの先頭アドレス。終端ブロックとの相対距離の基準となる。 */
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

/* link.x が使用するコード開始アドレスを起動ブロックの直後に置く。 */
_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    .bi_entries : ALIGN(4)
    {
        /* バイナリ情報エントリ列の開始アドレス。 */
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        /* 4 バイト境界へ丸めたエントリ列の終端アドレス（範囲に含まない）。 */
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    .end_block : ALIGN(4)
    {
        /* 終端ブロックの先頭アドレス。開始ブロックへの逆方向参照にも用いる。 */
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH
} INSERT AFTER .uninit;

/* 他で定義されていない場合に提供する、開始から終端までのバイト差分。 */
PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
/* 他で定義されていない場合に提供する、終端から開始までの逆向きバイト差分。 */
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
