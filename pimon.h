/******************************************************************************\
 * Native ESXi on Arm driver for hardware monitoring on the Raspberry Pi.
 * Copyright (c) 2020 Tom Hebel
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
\******************************************************************************/

#ifndef PIMON_H
#define PIMON_H

#define VMKAPI_ONLY
#include "vmkapi.h"

#define PIMON_DEBUG

#define PIMON_DRIVER_NAME "thpimon"

// Probably overkill
#define PIMON_HEAP_INITIAL_SIZE (1024 * 1024)
#define PIMON_HEAP_MAX_SIZE (2 * 1024 * 1024)

#define PIMON_INT_MAX ((vmk_uint32)~0)

#define PIMON_LOG_ERR       1
#define PIMON_LOG_WARN      2
#define PIMON_LOG_NOTE      3
#define PIMON_LOG_INIT      4
#define PIMON_LOG_DISC      5
#define PIMON_LOG_INFO      6
#define PIMON_LOG_FUNC      7
#define PIMON_LOG_TRACEIO   8

#endif /* PIMON_H */