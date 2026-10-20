/* Copyright 2024 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "tensorflow/lite/micro/micro_time.h"

// Set in micro/tools/make/targets/cortex_m_generic_makefile.inc.
// Needed for the DWT and PMU counters.
#ifdef CMSIS_DEVICE_ARM_CORTEX_M_XX_HEADER_FILE
#include CMSIS_DEVICE_ARM_CORTEX_M_XX_HEADER_FILE
#endif

namespace tflite {

#if defined(PROJECT_GENERATION)

// Stub functions for the project_generation target since these will be replaced
// by the target-specific implementation in the overall infrastructure that the
// TFLM project generation will be a part of.
uint32_t ticks_per_second() { return 1000; }
uint32_t GetCurrentTimeTicks() { return 0; }

#else

uint32_t ticks_per_second() { return 1000; }

extern "C" uint32_t rt_tick_get(void);
uint32_t GetCurrentTimeTicks() {
    return 0;
}

#endif  // defined(PROJECT_GENERATION)

}  // namespace tflite
