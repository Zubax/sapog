//
// Created by ramy on 23.02.21.
//

#ifndef SAPOG_APP_DESC_HPP
#define SAPOG_APP_DESC_HPP


namespace uavcan_node {
/**
 * This is the Brickproof Bootloader's app descriptor.
 * Details: https://github.com/PX4/Firmware/tree/nuttx_next/src/drivers/bootloaders/src/uavcan
 */
    static const volatile struct __attribute__((packed)) {
        std::uint8_t signature[8] = {'A', 'P', 'D', 'e', 's', 'c', '0', '0'};
        std::uint64_t image_crc = 0;
        std::uint32_t image_size = 0;
        std::uint32_t vcs_commit = GIT_HASH;
        std::uint8_t major_version = FW_VERSION_MAJOR;
        std::uint8_t minor_version = FW_VERSION_MINOR;
        std::uint8_t reserved[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    } _app_descriptor __attribute__((section(".app_descriptor")));
}


#endif //SAPOG_APP_DESC_HPP
