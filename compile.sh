#!/bin/sh
cargo build --release
cp ./target/x86_64-unknown-uefi/release/crossboot.efi ./esp/efi/boot/BOOTX64.efi
if [[ $1 == "run" ]]; then
	./run.sh	
fi
