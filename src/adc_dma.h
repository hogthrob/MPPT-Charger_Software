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

#ifndef ADC_DMA_H
#define ADC_DMA_H

/** @file
 *
 * @brief Reads ADC via DMA and stores data into necessary structs
 */

#include "dcdc.h"
#include "load.h"
#include "bat_charger.h"

#define ADC_FILTER_CONST 5          // filter multiplier = 1/(2^ADC_FILTER_CONST)

/** Struct to definie upper and lower limit alerts for any ADC channel
 */
template<typename CompareOp> struct AdcAlert {
    void (*callback)() = NULL;      ///< Function to be called when limits are exceeded
    uint16_t limit = 0;             ///< ADC reading for lower limit
    int debounce_ms = 0;            ///< Milliseconds delay for triggering alert

    /**
     * Check if the given value is outside of the limit and call respective callback
     * after checking ("debouncing") that this is not just a glitch in the input data
     * 
     * @param value value to be checked
     */

    void limit_check(uint16_t value) {
        debounce_ms++;
        CompareOp op;

        if (callback != NULL &&
            op(value,limit))
        {
            if (debounce_ms > 1) {
                // create function pointer and call function
                callback();
            }
        }
        else if (debounce_ms > 0) {
            // reset debounce ms counter only if already close to triggering to allow setting negative
            // values to specify a one-time inhibit delay
            debounce_ms = 0;
        }
    };
};

/** Sets offset to actual measured value, i.e. sets zero current point.
 *
 * All input/output switches and consumers should be switched off before calling this function
 */
void calibrate_current_sensors();

/** Updates structures with data read from ADC
 */
void update_measurements();

/** Initializes registers and starts ADC timer
 */
void adc_timer_start(int freq_Hz);

/** Sets necessary ADC registers
 */
void adc_setup(void);

/** Sets necessary DMA registers
 */
void dma_setup(void);

/** Read, filter and check raw ADC readings stored by DMA controller
 */
void adc_update_value(unsigned int pos);

/** Set lv side (battery) voltage limits where an alert should be triggered
 *
 * @param upper Upper voltage limit
 * @param lower Lower voltage limit
 */
void adc_set_lv_alerts(float upper, float lower);

/** Add an inhibit delay to the alerts to disable it temporarily
 *
 * @param adc_pos The position of the ADC measurement channel
 * @param timeout_ms Timeout in milliseconds
 */
void adc_upper_alert_inhibit(int adc_pos, int timeout_ms);

#endif /* ADC_DMA */
