/**
 * @file    globals.hpp
 * @brief   Runtime global variables — shared across all modules.
 *
 * @details Declares module-level globals such as the main loop counter.
 *          These are placed in a dedicated namespace to avoid collisions
 *          and are typically incremented once per main loop iteration.
 *
 *          Usage:
 *          @code
 *          ++drone::runtime::counter;
 *          @endcode
 *
 * @ingroup runtime
 */

#pragma once

#include <cstdint>

namespace drone::runtime {

/**
 * @defgroup runtime Runtime Globals
 * @brief    Global state accessible from any module.
 * @{
 */

/** @brief  Monotonic loop counter, incremented each iteration (~1 kHz). */
extern volatile std::uint32_t counter;

/** @} */

} // namespace drone::runtime
