#include "ArduRocket.h"

#if HAL_LOGGING_ENABLED

/*
  Vehicle log structures.

  ArduRocket adds no static message types of its own: RKT and RKTC are
  emitted through the dynamic AP_Logger::WriteStreaming() API with inline
  format strings (see ModeFlight::update()), which is self-describing and
  needs no entry here.

  LOG_COMMON_STRUCTURES is still mandatory. AP_Logger::validate_structures()
  walks every message type the common libraries register and panics
  ("PANIC: No structure for msg_type=NN") if the vehicle supplies none.
 */
const struct LogStructure ArduRocket::log_structure[] = {
    LOG_COMMON_STRUCTURES,
};

uint8_t ArduRocket::get_num_log_structures() const
{
    return ARRAY_SIZE(log_structure);
}

#endif  // HAL_LOGGING_ENABLED
