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

#include <stdlib.h>

#include "syscfg/syscfg.h"
#include "os/os.h"
#include "node/utilities.h"
#include "node/lora_priv.h"

int32_t
randr(int32_t min, int32_t max)
{
    return rand() % (max - min + 1) + min;
}

uint32_t
TimerGetElapsedTime(uint32_t savedTime)
{
    return hal_timer_read(MYNEWT_VAL(LORA_MAC_TIMER_NUM)) - savedTime;
}

uint32_t
TimerGetFutureTime(uint32_t eventInFuture)
{
    return hal_timer_read(MYNEWT_VAL(LORA_MAC_TIMER_NUM)) + eventInFuture;
}
