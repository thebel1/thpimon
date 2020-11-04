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

/*
 * pimon_charDev.h --
 */

#ifndef PIMON_CHARDEV_H
#define PIMON_CHARDEV_H

/***********************************************************************/

#define VMKAPI_ONLY
#include "vmkapi.h"

#include "pimon.h"

/***********************************************************************/

#define PIMON_CHARDEV_BUFFER_SIZE       4096 /* Probably overkill */
#define PIMON_CHARDEV_POLL_TIMEOUT_US   1000000

/***********************************************************************/

/*
 * Callbacks for gluing the char dev driver to the THX driver
 */
typedef VMK_ReturnStatus (*pimon_CharDevOpenCB_t)(vmk_CharDevFdAttr *attr);
typedef VMK_ReturnStatus (*pimon_CharDevCloseCB_t)(vmk_CharDevFdAttr *attr);

typedef VMK_ReturnStatus (*pimon_CharDevIoctlCB_t)(unsigned int cmd,
                                                 void *ioctlData,
                                                 vmk_ByteCount dataLen);
typedef VMK_ReturnStatus (*pimon_CharDevReadCB_t)(char *buffer,
                                                vmk_ByteCount nbytes,
                                                vmk_loff_t *ppos,
                                                vmk_ByteCountSigned *nread);
typedef VMK_ReturnStatus (*pimon_CharDevWriteCB_t)(char *buffer,
                                                 vmk_ByteCount nbytes,
                                                 vmk_loff_t *ppos,
                                                 vmk_ByteCountSigned *nread);
typedef struct pimon_CharDevCallbacks_t {
   pimon_CharDevOpenCB_t open;
   pimon_CharDevCloseCB_t close;
   pimon_CharDevIoctlCB_t ioctl;
   pimon_CharDevReadCB_t read;
   pimon_CharDevWriteCB_t write;
} pimon_CharDevCallbacks_t;

typedef struct pimon_CharDevPriv_t {
   /* The size of the data being passed via ioctl */
   vmk_ByteCount ioctlDataLen;
} pimon_CharDevPriv_t;

typedef struct pimon_CharDev_t {
   vmk_Device vmkDevice;
   vmk_CharDevRegData regData;
   pimon_CharDevCallbacks_t devCBs;
} pimon_CharDev_t;

typedef struct pimon_CharFileData_t {
   vmk_Lock lock;
   char *data;
   vmk_Bool timerPending;
   vmk_Bool deathPending;
   vmk_Timer timer;
   vmk_int32 timeoutUS;
   vmk_uint32 pollMask;
} pimon_CharFileData_t;

typedef struct pimon_CharDevProps_t {
   vmk_Driver driverHandle;
   vmk_BusType logicalBusType;
   vmk_Device parentDevice;
   pimon_CharDev_t *charDev;
   pimon_CharDevPriv_t *privData;
   vmk_uint32 logicalPort;
   pimon_CharDevCallbacks_t *callbacks;
} pimon_CharDevProps_t;

/***********************************************************************/

VMK_ReturnStatus pimon_charDevInit(vmk_ModuleID moduleID,
                                 vmk_HeapID heapID,
                                 vmk_LogComponent logger);

VMK_ReturnStatus pimon_charDevRegister(pimon_CharDevProps_t *props);

VMK_ReturnStatus pimon_charVmkDevRemove(vmk_Device logicalDev);

VMK_ReturnStatus pimon_charDevAssoc(vmk_AddrCookie charDevPriv,
                                  vmk_CharDevHandle charDevHandle);

VMK_ReturnStatus pimon_charDevDisassoc(vmk_AddrCookie charDevPriv);

VMK_ReturnStatus pimon_charDevOpen(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus pimon_charDevClose(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus pimon_charDevIoctl(vmk_CharDevFdAttr *attr,
                                  vmk_uint32 cmd,
                                  vmk_uintptr_t userData,
                                  vmk_IoctlCallerSize callerSize,
                                  vmk_int32 *result);

VMK_ReturnStatus pimon_charDevRead(vmk_CharDevFdAttr *attr,
                                 char *buffer,
                                 vmk_ByteCount nbytes,
                                 vmk_loff_t *ppos,
                                 vmk_ByteCountSigned *nread);

VMK_ReturnStatus pimon_charDevWrite(vmk_CharDevFdAttr *attr,
                                  char *buffer,
                                  vmk_ByteCount nbytes,
                                  vmk_loff_t *ppos,
                                  vmk_ByteCountSigned *nwritten);

VMK_ReturnStatus pimon_charDevIO(vmk_CharDevFdAttr *attr,
                               char *buffer,
                               vmk_ByteCount nbytes,
                               vmk_loff_t *ppos,
                               vmk_ByteCountSigned *ndone,
                               vmk_Bool isWrite);

VMK_ReturnStatus pimon_charDevPoll(vmk_CharDevFdAttr *attr,
                                 vmk_PollContext pollCtx,
                                 vmk_uint32 *pollMask);

void pimon_charDevTimerCB(vmk_TimerCookie data);

void pimon_charDevFileDestroy(pimon_CharFileData_t *fileData);

/***********************************************************************/

#endif /* PIMON_CHARDEV_H */