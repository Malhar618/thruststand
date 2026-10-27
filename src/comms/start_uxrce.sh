#!/bin/bash

# sudo password for the Odroid user. Set it in your shell first
# (export SUDO_PASS=...) or the script asks for it. Never commit it.
if [ -z "$SUDO_PASS" ]; then
    read -r -s -p "sudo password: " SUDO_PASS
    echo
fi
password="$SUDO_PASS"

# Define the command
command="sudo MicroXRCEAgent serial --dev /dev/ttyPixhawk -b 921600"

# Execute the command with sudo, providing the password 
echo $password | sudo -S $command
