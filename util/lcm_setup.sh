#!/usr/bin/env bash

set -euo pipefail

sudo apt install -y liblcm-dev libboost-all-dev
pip install lcm || true

# set IP of ethernet to 192.168.123.100/24
# try to ping 192.168.123.10 and check whether it is reachable
sudo ip route add 239.255.76.67/32 dev lo || true
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=0

cd $(dirname "$0")/../unitree_legged_sdk
mkdir build || true
cd build
cmake ..
make clean
make

./lcm_server_high &
