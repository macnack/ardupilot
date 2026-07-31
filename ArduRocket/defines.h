#pragma once

// Logging bitmask - which log streams are enabled
#define MASK_LOG_ATTITUDE       (1<<0)
#define MASK_LOG_GPS            (1<<1)
#define MASK_LOG_PM             (1<<2)
#define MASK_LOG_CTUN           (1<<3)
#define MASK_LOG_NTUN           (1<<4)
#define MASK_LOG_IMU            (1<<5)
#define MASK_LOG_CURRENT        (1<<6)
#define MASK_LOG_RCIN           (1<<7)
#define MASK_LOG_RCOUT          (1<<8)
#define MASK_LOG_ANY            0xFFFF
