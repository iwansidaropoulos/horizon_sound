#!/bin/bash
apt-get update
mkdir tools

# Install gcc_arm_none_eabi_10_2020_q4_major
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
tar -xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2 -C tools
rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2

# Install nRF command line tools for mergehex
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/Desktop-software/nRF-command-line-tools/sw/Versions-10-x-x/10-13-0/nRF-Command-Line-Tools_10_13_0_Linux64.zip
unzip nRF-Command-Line-Tools_10_13_0_Linux64.zip -d tools
rm nRF-Command-Line-Tools_10_13_0_Linux64.zip
cd tools/nRF-Command-Line-Tools_10_13_0_Linux64
tar -xf nRF-Command-Line-Tools_10_13_0_Linux-amd64.tar.gz
rm nRF-Command-Line-Tools_10_13_0_Linux-amd64.tar.gz
tar -xf nRF-Command-Line-Tools_10_13_0.tar
rm nRF-Command-Line-Tools_10_13_0.tar

# Install nrfutil
pip3 install nrfutil
