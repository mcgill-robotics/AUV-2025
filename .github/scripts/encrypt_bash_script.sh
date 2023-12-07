#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 script_name password"
    exit 1
fi

if [ ! -f "$1" ]; then
    echo "Error: File '$1' not found."
    exit 1
fi

script_content=$(cat $1)
script_name=$(basename "$1")
encrypted_content=$(echo "$script_content" | openssl enc -aes-256-cbc -a -salt -k "$2")
shc -r -o ${1%.*} -f $1
CC="arm-linux-gnueabihf-gcc-4.9"
CFLAGS="-march=armv7-a -static"
shc -r -o ${1%.*}_arm -f $1
CC="aarch64-linux-gnu-gcc"
CFLAGS="-march=armv8-a -static"
shc -r -o ${1%.*}_aarch64 -f $1
rm $1.x.c
echo "$encrypted_content" > encrypted_$script_name