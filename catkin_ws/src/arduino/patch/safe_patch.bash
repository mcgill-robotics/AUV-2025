#!/bin/bash

# Applies patch if patch has not already been applied
# ------------------------
# Arguments
# $1 - path to original file
# $2 - path to diff file

original=$1
patchFile=$2

echo "patching file $original with $patchFile";

patch -Rsf --dry-run $original $patchFile;
patched=$?

# if reverse patch succeeds, that means the file was previously patched
if [ $patched -ne 0 ]; then 
    echo "patching file...";
    patch --verbose $original $patchFile;
else
    echo "file is already patched.";
fi