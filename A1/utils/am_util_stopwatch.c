//*****************************************************************************
//
//! @file am_util_stopwatch.c
//!
//! @brief Provides functionality to measure elapsed time.
//!
//! Functions for measuring elapsed time. These can be useful for providing
//! 'ticks' where needed.
//!
//! @note These functions require a RTC to function properly. Therefore, if any
//! RTC configuring takes place after calling am_util_stopwatch_start() the
//! resulting elapsed time will be incorrect unless you first call
//! am_util_stopwatch_restart()
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2016, Ambiq Micro
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 1.1.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "hal/am_hal_rtc.h"
#include "am_util_stopwatch.h"

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
volatile uint32_t g_ui64ElapsedTime = 0;      // Total elapsed time in ms.
volatile uint32_t g_ui64PausedTime = 0;       // Total paused time in ms.
volatile bool g_bStarted = false;             // Stopwatch started state.
volatile bool g_bPaused = false;              // Stopwatch paused state.
am_hal_rtc_time_t g_sStartTime;               // Start time to determine elapsed time.
am_hal_rtc_time_t g_sPauseTime;               // Pause time to determine elapsed time.

//*****************************************************************************
//
// Format time based on resolution.
//
//*****************************************************************************
static uint32_t
time_format(uint64_t ui64TimeMS, uint32_t ui32Resolution)
{
    switch (ui32Resolution)
    {
        case AM_UTIL_STOPWATCH_MS:
            return ui64TimeMS;
        case AM_UTIL_STOPWATCH_SEC:
            return ui64TimeMS / 1000;
        case AM_UTIL_STOPWATCH_MIN:
            return ui64TimeMS / 60000;
        case AM_UTIL_STOPWATCH_HOUR:
            return ui64TimeMS / 3600000;
        case AM_UTIL_STOPWATCH_DAY:
            return ui64TimeMS / 86400000;
        case AM_UTIL_STOPWATCH_MONTH:
             return ui64TimeMS / (uint64_t) 2629740000;
        case AM_UTIL_STOPWATCH_YEAR:
             return ui64TimeMS / (uint64_t) 31556900000;
        default:
            return ui64TimeMS;
    }
}

//*****************************************************************************
//
// Return the absolute time in milliseconds from a RTC structure.
//
//*****************************************************************************
static uint64_t
elapsed_time_ms(am_hal_rtc_time_t *psStartTime, am_hal_rtc_time_t *psStopTime)
{
    int64_t i64DeltaYear = 0;
    int64_t i64DelataMonth = 0;
    int64_t i64DeltaDay = 0;
    int64_t i64DelatHour = 0;
    int64_t i64DeltaMinute = 0;
    int64_t i64DeltaSecond = 0;
    int64_t i64DeltaHundredths = 0;
    uint64_t ui64DeltaTotal = 0;

    i64DeltaYear = (psStopTime->ui32Year - psStartTime->ui32Year) * (uint64_t) 31556900000;
    i64DelataMonth = (psStopTime->ui32Month - psStartTime->ui32Month) * (uint64_t) 2629740000;
    i64DeltaDay = (psStopTime->ui32DayOfMonth - psStartTime->ui32DayOfMonth) * 86400000;
    i64DelatHour = (psStopTime->ui32Hour - psStartTime->ui32Hour) * 3600000;
    i64DeltaMinute = (psStopTime->ui32Minute - psStartTime->ui32Minute) * 60000;
    i64DeltaSecond = (psStopTime->ui32Second - psStartTime->ui32Second) * 1000;
    i64DeltaHundredths = (psStopTime->ui32Hundredths - psStartTime->ui32Hundredths) * 10;

    ui64DeltaTotal = (i64DeltaYear + i64DelataMonth + i64DeltaDay + i64DelatHour +
                      i64DeltaMinute + i64DeltaSecond + i64DeltaHundredths);

    return ui64DeltaTotal;
}

//*****************************************************************************
//
//! @brief Start the stopwatch.
//!
//! This function records the current time from the RTC and sets the start time.
//!
//! @return None.
//
//*****************************************************************************
void
am_util_stopwatch_start(void)
{
    am_hal_rtc_time_t rtc_time;

    //
    // If the start time is clear, read the RTC time to get a reference starting
    // time.
    if (g_bPaused == false && g_bStarted == false)
    {
        //
        // Clear the timer which gets the current time as well.
        //
        am_util_stopwatch_clear();
    }

    //
    // We were paused.
    // Now we need to figure out how long we were paused for.
    //
    else if (g_bPaused == true && g_bStarted == true)
    {
        //
        // Get the RTC time.
        //
        while(am_hal_rtc_time_get(&rtc_time));

        //
        // Add the time we spent paused to the time we already spent paused.
        //
        g_ui64PausedTime += elapsed_time_ms(&g_sPauseTime, &rtc_time);
    }

    //
    // Set started to true.
    //
    g_bStarted = true;

    //
    // Set paused to false.
    //
    g_bPaused = false;
}

//*****************************************************************************
//
//! @brief Stop the stopwatch.
//!
//! This function stops the stop watch and anytime am_util_stopwatch_elapsed_get()
//! is called it will return the same elapsed time until am_util_stopwatch_start()
//! is called again.
//!
//! @return None.
//
//*****************************************************************************
void
am_util_stopwatch_stop(void)
{
    //
    // Save the current time so we know how long we've been paused for.
    //
    while(am_hal_rtc_time_get(&g_sPauseTime));

    //
    // Set the state to paused.
    //
    g_bPaused = true;
}

//*****************************************************************************
//
//! @brief Clears the stopwatch.
//!
//! This function clears the start time on the stop watch. If the stop watch is
//! running, it will continue to count the elapsed time from the new start time.
//!
//! @return None.
//
//*****************************************************************************
void
am_util_stopwatch_clear(void)
{
    //
    // Read the RTC and save in g_ui64StartTime.
    //
    while(am_hal_rtc_time_get(&g_sStartTime));

    //
    // Reset the paused time.
    //
    g_ui64PausedTime = 0;

    //
    // Reset the elapsed time.
    //
    g_ui64ElapsedTime = 0;
}

//*****************************************************************************
//
//! @brief Restart the stopwatch.
//!
//! This function restarts the stopwatch.
//!
//! If the stopwatch was previously stopped this is functionally equivalent
//! calling am_util_stopwatch_clear() followed by am_util_stopwatch_start().
//!
//! If the stopwatch was previously started this is functionally equivalent to
//! am_util_stopwatch_clear().
//!
//! @return None.
//
//*****************************************************************************
void
am_util_stopwatch_restart(void)
{
    //
    // Clear the stopwatch.
    //
    am_util_stopwatch_clear();

    //
    // Start the stopwatch.
    //
    am_util_stopwatch_start();
}

//*****************************************************************************
//
//! @brief Get the elapsed time from the stopwatch.
//!
//! @param ui32Resolution - the desired resolution to return the elapsed time in.
//!
//! This function returns the elapsed time in the desired resolution as requested
//! from ui32Resolution.
//!
//! Valid values for ui32Resolution:
//!     AM_UTIL_STOPWATCH_MS
//!     AM_UTIL_STOPWATCH_SEC
//!     AM_UTIL_STOPWATCH_MIN
//!     AM_UTIL_STOPWATCH_HOUR
//!     AM_UTIL_STOPWATCH_DAY
//!     AM_UTIL_STOPWATCH_MONTH
//!     AM_UTIL_STOPWATCH_YEAR
//!
//! @return Elapsed Time in ui32Resolution.
//
//*****************************************************************************
uint64_t
am_util_stopwatch_elapsed_get(uint32_t ui32Resolution)
{
    am_hal_rtc_time_t rtc_time;

    //
    // Stop watch is not paused and is running.
    // Figure out elapsed time.
    //
    if (g_bPaused == false && g_bStarted == true)
    {
        //
        // Get the RTC time.
        //
        while(am_hal_rtc_time_get(&rtc_time));

        g_ui64ElapsedTime = elapsed_time_ms(&g_sStartTime, &rtc_time) -
                                g_ui64PausedTime;
    }

    //
    // Return the elapsed time.
    //
    return time_format(g_ui64ElapsedTime, ui32Resolution);
}
