/**
 * @file    globals.cpp
 * @brief   Runtime global variable definitions.
 *
 * @details Defines the storage for module-level globals declared in
 *          globals.hpp.  These are placed in the BSS section and
 *          initialised to zero.
 *
 * @ingroup runtime
 */

#include "runtime/globals.hpp"

namespace drone::runtime {

/** @brief  Main loop counter (one increment per ~1 ms iteration). */
volatile std::uint32_t counter = 0;

} // namespace drone::runtime
