#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "hardware/spi.h"
#include "hardware/uart.h"

// Include board-specific configuration
// Both Pico W and Pico 2W use same pinout
#if defined(RASPBERRYPI_PICO2_W) || defined(PICO_RP2350)
#include "pico2_w.h"
#else
#include "raspberrypi_pico_w_config.h"
#endif

#endif  // CONFIGURATION_H_