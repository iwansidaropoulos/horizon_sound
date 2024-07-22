#!/bin/bash

# Update PATH environment variable to include necessary tools
export PATH="/[path to]/CLS-Argos-Linkit-CORE-3.4.3/tools/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
export GNU_INSTALL_ROOT="/[path to]/CLS-Argos-Linkit-CORE-3.4.3/tools/gcc-arm-none-eabi-10-2020-q4-major/bin/"
export PATH="/[path to]/CLS-Argos-Linkit-CORE-3.4.3/tools/nRF-Command-Line-Tools_10_13_0_Linux64/mergehex:$PATH"

# Clean the build directory for HORIZON project
rm -r ports/nrf52840/build/HORIZON/*
mkdir ports/nrf52840/build
mkdir ports/nrf52840/build/HORIZON
cd ports/nrf52840/build/HORIZON

# Build
cmake -DCMAKE_TOOLCHAIN_FILE=../../toolchain_arm_gcc_nrf52.cmake -DDEBUG_LEVEL=4 -DBOARD=LINKIT -DCMAKE_BUILD_TYPE=Release -DMODEL=CORE ../..
make -j 4
nrfutil settings generate --family NRF52840 --application LinkIt_CORE_board.hex --application-version 0 --bootloader-version 1 --bl-settings-version 2 --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --sd-boot-validation VALIDATE_ECDSA_P256_SHA256 --softdevice ../../drivers/nRF5_SDK_17.0.2/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --key-file ../../nrfutil_pkg_key.pem settings.hex
mergehex -m ../../bootloader/gentracker_secure_bootloader/gentracker_v1.0/armgcc/_build/cls_bootloader_v1_linkit_merged.hex LinkIt_CORE_board.hex -o m1.hex
mergehex -m m1.hex settings.hex -o LinkIt_CORE_board_merged.hex
rm -f m1.hex settings.hex
