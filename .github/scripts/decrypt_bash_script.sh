#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 script_name password"
    exit 1
fi

if [ ! -f "$1" ]; then
    echo "Error: File '$1' not found."
    exit 1
fi

encrypted_content=$(cat $1)
script_name=$(basename "$1")
decrypted_content=$(echo "$encrypted_content" | openssl enc -d -aes-256-cbc -a -salt -k "$2")
echo "$decrypted_content" > decrypted_$script_name