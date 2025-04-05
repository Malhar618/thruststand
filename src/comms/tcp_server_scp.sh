#!/bin/bash
local_ip=$(hostname -I)
local_dir=$(pwd)
scp odroid@192.168.12.1:L2norm.json "$local_ip:$local_dir"

