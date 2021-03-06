/* LibreSolar charge controller firmware
 * Copyright (c) 2016-2019 Martin Jäger (www.libre.solar)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PCB_H
#define PCB_H

/** @file
 *
 * @brief
 * Includes PCB hardware specific settings like pin names depending on the compiler flags set in platformio.ini.
 */

// specific board settings
///////////////////////////////////////////////////////////////////////////////

#if defined(MPPT_2420_LC_0V10)
    #include "pcbs/mppt_2420_lc_0v10.h"
#elif defined(MPPT_1210_HUS_0V2)
    #include "pcbs/mppt_1210_hus_0v2.h"
#elif defined(MPPT_1210_HUS_0V4)
    #include "pcbs/mppt_1210_hus_0v4.h"
#elif defined(MPPT_1210_HUS_0V7)
    #include "pcbs/mppt_1210_hus_0v7.h"
#elif defined(PWM_2420_LUS_0V2) || defined(PWM_2420_LUS_0V3)
    #include "pcbs/pwm_2420_lus.h"
#elif defined(UNIT_TEST)
    #include "pcbs/pcb_stub.h"
#else
    #error "PCB has to be specified!"
#endif

// generic default settings (if not defined different in board config)
///////////////////////////////////////////////////////////////////////////////

/** Main control function frequency (Hz)
 *
 * Frequencies higher than 10 Hz caused issues with MPPT control during lab test with
 * PV simulator. Might be different with actual solar panel.
 */
#ifndef CONTROL_FREQUENCY
#define CONTROL_FREQUENCY 10
#endif

/** Maximum Tj of MOSFETs (°C)
 *
 * This value is used for model-based control of overcurrent protection. It represents
 * the steady-state junction temperature for max. continuous current at ambient
 * temperature of 25°C.
 */
#ifndef MOSFET_MAX_JUNCTION_TEMP
#define MOSFET_MAX_JUNCTION_TEMP    120
#endif

/** Internal reference temperature at full load (°C)
 *
 * This value is used for model-based control of overcurrent protection. It represents
 * the steady-state internal temperature for max. continuous current at ambient
 * temperature of 25°C. Value here is conservative and can be overwritten in PCB configuration.
 */
#ifndef INTERNAL_MAX_REFERENCE_TEMP
#define INTERNAL_MAX_REFERENCE_TEMP 50
#endif

/** Thermal time constant junction to ambient (s)
 *
 * This value is used for model-based control of overcurrent protection. It does not reflect
 * the much lower MOSFET-internal time constant junction to case, but includes thermal inertia
 * of the board.
 *
 * Around 5s seems to be a good conservative estimation for 5x6 type SMD MOSFETs
 */
#ifndef MOSFET_THERMAL_TIME_CONSTANT
#define MOSFET_THERMAL_TIME_CONSTANT  5
#endif

#endif /* PCB_H */
