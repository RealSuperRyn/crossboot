#!/bin/sh
qemu-system-x86_64 -d int,cpu_reset -nographic -enable-kvm     -drive if=pflash,format=raw,readonly=on,file=OVMF_CODE.fd     -drive if=pflash,format=raw,readonly=on,file=OVMF_VARS.fd     -drive format=raw,file=fat:rw:esp
