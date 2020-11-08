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
#include "rpiq_drv.h"

/***********************************************************************/

#define PIMON_CHARDEV_IOCTL_BUFFER_LEN  (~(vmk_uint32)0)
#define PIMON_CHARDEV_POLL_TIMEOUT_US   1000000

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