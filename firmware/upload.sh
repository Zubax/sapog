#!/bin/bash
 
PORT="$1"
elf="$2"
 
arm-none-eabi-size $elf || exit 1
 
tmpfile=.blackmagic_gdb.tmp
cat > $tmpfile <<EOF
target extended-remote $PORT
mon swdp_scan
attach 1
load
kill
EOF
 
arm-none-eabi-gdb $elf --batch -x $tmpfile
 
rm -f $tmpfile
