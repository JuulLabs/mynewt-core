/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stdbool.h>
#include <stdint.h>
#include "syscfg/syscfg.h"
#include "mcu/mcu.h"
#include "mcu/da1469x_clock.h"
#include "mcu/da1469x_lpclk.h"
#include "hal/hal_system.h"
#include "hal/hal_timer.h"
#include "os/os_cputime.h"
#include "da1469x_priv.h"

bool g_mcu_lpclk_available;

static da1469x_lpclk_cb *g_da1469x_lpclk_cmac_cb;

#if MYNEWT_VAL(MCU_LPCLK_SOURCE)
static void
da1469x_lpclk_settle_tmr_cb(void *arg)
{
#if MYNEWT_VAL_CHOICE(MCU_LPCLK_SOURCE, XTAL32K)
    da1469x_clock_lp_xtal32k_switch();
#endif
#if MYNEWT_VAL_CHOICE(MCU_LPCLK_SOURCE, RCX)
    da1469x_clock_lp_rcx_switch();
#endif
    g_mcu_lpclk_available = true;

    if (g_da1469x_lpclk_cmac_cb) {
        g_da1469x_lpclk_cmac_cb();
    }
}
#endif

void
da1469x_lpclk_register_cmac_cb(da1469x_lpclk_cb *cb)
{
    g_da1469x_lpclk_cmac_cb = cb;

    if (g_mcu_lpclk_available) {
        cb();
    }
}

void
da1469x_lpclk_init(void)
{
#if MYNEWT_VAL_CHOICE(MCU_LPCLK_SOURCE, XTAL32K)
    static struct hal_timer lpclk_settle_tmr;
    da1469x_clock_lp_xtal32k_enable();
    os_cputime_timer_init(&lpclk_settle_tmr, da1469x_lpclk_settle_tmr_cb, NULL);
    os_cputime_timer_relative(&lpclk_settle_tmr,
                              MYNEWT_VAL(MCU_CLOCK_XTAL32K_SETTLE_TIME_MS) * 1000);
#endif
#if MYNEWT_VAL_CHOICE(MCU_LPCLK_SOURCE, RCX)
    static struct hal_timer lpclk_settle_tmr;
    da1469x_clock_lp_rcx_enable();
    os_cputime_timer_init(&lpclk_settle_tmr, da1469x_lpclk_settle_tmr_cb, NULL);
    os_cputime_timer_relative(&lpclk_settle_tmr,
                              MYNEWT_VAL(MCU_CLOCK_RCX_SETTLE_TIME_MS) * 1000);
#endif
}
