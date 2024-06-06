#!/usr/bin/env bash
# use like sudo ./install_package_recusrive.sh <package_name>
# Run the command and capture the output
install_package() {
    echo "Installing $1"
    output=$(sudo apt-get install -y --allow-downgrades $1 2>&1)

    # Check if there are any unmet dependencies
    if [[ $output == *"The following packages have unmet dependencies"* ]]; then
        # Extract package names
        # echo "$output"
        
        packages=$(echo "$output" | grep ": Depends:" | awk '{print $4 "=" substr($6, 1, length($6)-1)}')
        # Loop through each package and echo its name
        for package in $packages; do
            # echo "$package"
            install_package $package 
        done
        
        packages="$(echo "$output" | grep "  Depends:" | awk '{print $2 "=" substr($4, 1, length($6)-1)}')"
        # Loop through each package and echo its name
        for package in $packages; do
            # echo "$package"
            install_package $package 
        done

        install_package $1
    else
        echo "Installed $1"
    fi
}

install_package $1