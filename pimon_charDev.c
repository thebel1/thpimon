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
 * pimon_charDev.c --
 * 
 *    Generic character device driver implementation.
 */

#include "pimon_charDev.h"

/***********************************************************************/

static vmk_ModuleID pimon_moduleID;
static vmk_HeapID pimon_heapID;
static vmk_LogComponent pimon_logger;

static vmk_CharDevOps pimon_charDevFileOps = {
   .open = pimon_charDevOpen,
   .close = pimon_charDevClose,
   .ioctl = pimon_charDevIoctl,
   .poll = pimon_charDevPoll,
   .read = pimon_charDevRead,
   .write = pimon_charDevWrite,
};

static vmk_CharDevRegOps pimon_CharDevOps = {
   .associate = pimon_charDevAssoc,
   .disassociate = pimon_charDevDisassoc,
   .fileOps = &pimon_charDevFileOps,
};

/*
 * Dev ops for the new vmk_Device associated with the char dev
 */
static vmk_DeviceOps pimon_CharVmkDevOps = {
   .removeDevice = pimon_charVmkDevRemove,
};

/*
 * Call backs that glue the char dev to the GPIO driver
 */
static pimon_CharDevCallbacks_t *pimon_CharDevCBs;

/***********************************************************************/

static vmk_TimerQueue pimon_charDevTimerQueue;

/*
 ***********************************************************************
 * pimon_charDevInit --
 * 
 *    Initialize the character device driver. Onyl called once, as it sets
 *    a number of global variables.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevInit(vmk_ModuleID moduleID,
                  vmk_HeapID heapID,
                  vmk_LogComponent logger)
{
   VMK_ReturnStatus status = VMK_OK;

   pimon_moduleID = moduleID;
   pimon_heapID = heapID;
   pimon_logger = logger;

   return status;
}

/*
 ***********************************************************************
 * pimon_charDevRegister --
 * 
 *    Register a character device with the vmkernel chardev layer.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevRegister(pimon_CharDevProps_t *props)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_Device newDevice;
   vmk_DeviceID devID;
   vmk_AddrCookie registeringDriverData;
   vmk_AddrCookie registrationData;
   vmk_DeviceProps deviceProps;
   pimon_CharDev_t *charDev;
   vmk_AddrCookie charDevPriv;

   devID.busType = props->logicalBusType;
   status = vmk_LogicalCreateBusAddress(props->driverHandle,
                                        props->parentDevice,
                                        props->logicalPort,
                                        &devID.busAddress,
                                        &devID.busAddressLen);
   if (status != VMK_OK) {
      goto logical_bus_failed;
   }

   charDev = props->charDev;

   /*
    * As per vmkapi_char.h it can only be graphics or a test device
    */
   devID.busIdentifier = VMK_CHARDEV_IDENTIFIER_GRAPHICS;
   devID.busIdentifierLen = vmk_Strnlen(devID.busIdentifier, VMK_MISC_NAME_MAX);

   charDevPriv.ptr = props->privData;

   /*
    * Set up char dev registration
    */

   charDev->regData.moduleID = pimon_moduleID;
   charDev->regData.deviceOps = &pimon_CharDevOps;
   charDev->regData.devicePrivate = charDevPriv;

   registrationData.ptr = &charDev->regData;
   registeringDriverData.ptr = props->charDev;

   deviceProps.registeringDriver = props->driverHandle;
   deviceProps.deviceID = &devID;
   deviceProps.deviceOps = &pimon_CharVmkDevOps;
   deviceProps.registeringDriverData = registeringDriverData;
   deviceProps.registrationData = registrationData;

   status = vmk_DeviceRegister(&deviceProps, props->parentDevice, &newDevice);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "failed to register device: %s",
                  vmk_StatusToString(status));
      goto register_device_failed;
   }

   status = vmk_LogicalFreeBusAddress(props->driverHandle,
                                      devID.busAddress);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "failed to free logical bus: %s",
                  vmk_StatusToString(status));
      goto free_bus_failed;
   }

   /*
    * Set CBs
    */
   pimon_CharDevCBs = props->callbacks;

   return VMK_OK;

free_bus_failed:
   vmk_LogicalFreeBusAddress(props->driverHandle,
                             devID.busAddress);
register_device_failed:
logical_bus_failed:
   return status;
}

/*
 ***********************************************************************
 * pimon_charVmkDevRemove --
 * 
 *    Remove character device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charVmkDevRemove(vmk_Device logicalDev)
{
   VMK_ReturnStatus status = VMK_OK;

   status = vmk_DeviceUnregister(logicalDev);
   if (status != VMK_OK)
   {
      vmk_Warning(pimon_logger,
                  "failed to unregister device: %s",
                  vmk_StatusToString(status));
   }

   return status;
}

/*
 ***********************************************************************
 * pimon_charDevAssoc --
 * 
 *    Associate a char device with a device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevAssoc(vmk_AddrCookie charDevPriv,
                   vmk_CharDevHandle charDevHandle)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_Name charDevAlias;

   status = vmk_CharDeviceGetAlias(charDevHandle, &charDevAlias);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "failed to obtain logical device alias: %s",
                  vmk_StatusToString(status));
      goto get_alias_failed;
   }

#ifdef PIMON_DEBUG
   {
      vmk_Log(pimon_logger,
              "obtained logical device alias %s",
              charDevAlias.string);
   }
#endif /* PIMON_DEBUG */

   return VMK_OK;

get_alias_failed:
   return status;
}

/*
 ***********************************************************************
 * pimon_charDevDisassoc --
 * 
 *    Disassociate a char device with a device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevDisassoc(vmk_AddrCookie charDevPriv)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * pimon_charDevOpen --
 * 
 *    Opens a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevOpen(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;
   pimon_CharFileData_t *fileData;
   vmk_SpinlockCreateProps lockProps;

   /*
    * Init private file data
    */

   fileData = vmk_HeapAlloc(pimon_heapID, sizeof(*fileData));
   if (fileData == NULL) {
      status = VMK_NO_MEMORY;
      vmk_Warning(pimon_logger,
                  "failed to create file private data: %s",
                  vmk_StatusToString(status));
      goto file_priv_alloc_failed;
   }
   vmk_Memset(fileData, 0, sizeof(*fileData));

   /*
    * Init lock
    */

   lockProps.moduleID = pimon_moduleID;
   lockProps.heapID = pimon_heapID;
   status = vmk_NameInitialize(&lockProps.name, PIMON_DRIVER_NAME);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "failed to init lock name: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   lockProps.type = VMK_SPINLOCK;
   lockProps.domain = VMK_LOCKDOMAIN_INVALID;
   lockProps.rank = VMK_SPINLOCK_UNRANKED;
   status = vmk_SpinlockCreate(&lockProps, &fileData->lock);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "failed to create spinlock: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   /*
    * We don't use this buffer, so set it to NULL
    */
   fileData->data = NULL;

   /*
    * Prep file for I/O
    */
   fileData->timerPending = VMK_FALSE;
   fileData->deathPending = VMK_FALSE;
   fileData->pollMask = VMKAPI_POLL_WRITE;
   fileData->timeoutUS = PIMON_CHARDEV_POLL_TIMEOUT_US;
   attr->clientInstanceData.ptr = fileData;

#ifdef PIMON_DEBUG
   {
      vmk_Log(pimon_logger,
              "opened file; priv %p lock %p",
              fileData,
              fileData->lock);
   }
#endif /* PIMON_DEBUG */

   /* Call CB */
   status = pimon_CharDevCBs->open(attr);

   return status;

lock_init_failed:
   vmk_HeapFree(pimon_heapID, fileData);

file_priv_alloc_failed:
   return status;
}

/*
 ***********************************************************************
 * pimon_charDevClose --
 * 
 *    Closes a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevClose(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;
   pimon_CharFileData_t *fileData = attr->clientInstanceData.ptr;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_logger, "file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   fileData->deathPending = VMK_TRUE;

   if (fileData->timerPending == VMK_FALSE) {
      vmk_SpinlockUnlock(fileData->lock);
      pimon_charDevFileDestroy(fileData);
   }

   /* Call CB */
   status = pimon_CharDevCBs->close(attr);

file_data_null:
   return status;
}

/*
 ***********************************************************************
 * pimon_charDevIoctl --
 * 
 *    GPIO chardev-specific I/O ops. Used for programming reads to input
 *    pins.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevIoctl(vmk_CharDevFdAttr *attr,
                   unsigned int cmd,
                   vmk_uintptr_t userData,
                   vmk_IoctlCallerSize callerSize,
                   vmk_int32 *result)
{
   VMK_ReturnStatus status = VMK_OK;
   void *ioctlData;
   vmk_ByteCount ioctlDataLen;
   pimon_CharFileData_t *fileData = attr->clientInstanceData.ptr;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_logger, "file data null");
      goto file_data_null;
   }

   /*
    * Allocate ioctl data
    */
   ioctlDataLen = ((pimon_CharDevPriv_t *)attr->clientDeviceData.ptr)->ioctlDataLen;
   ioctlData = vmk_HeapAlloc(pimon_heapID, ioctlDataLen);
   if (ioctlData == NULL) {
      status = VMK_NO_MEMORY;
      vmk_Warning(pimon_logger,
                  "failed to allocate memory for ioctl data: %s",
                  vmk_StatusToString(status));
      goto ioctl_alloc_failed;
   }

#ifdef PIMON_DEBUG
   {
      vmk_Log(pimon_logger,
              "copying ioctl data from %p (UW) to %p (vmk)",
              (vmk_VA)userData,
              (vmk_VA)ioctlData);
   }
#endif /* PIMON_DEBUG */

   /*
    * Copy ioctl data from UW
    */
   status = vmk_CopyFromUser((vmk_VA)ioctlData,
                             (vmk_VA)userData,
                             ioctlDataLen);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "unable to copy ioctl data from UW ptr %p: %s",
                  userData,
                  vmk_StatusToString(status));
      goto ioctl_uw2vmk_failed;
   }

#ifdef PIMON_DEBUG
   {
      vmk_Log(pimon_logger,
              "executing ioctl cmd %d with data %p",
              cmd,
              userData);
   }
#endif

   /*
    * Call CB
    */
   vmk_SpinlockLock(fileData->lock);
   status = pimon_CharDevCBs->ioctl(cmd, ioctlData, ioctlDataLen);
   vmk_SpinlockUnlock(fileData->lock);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "ioctl cmd %d with data %p failed: %s",
                  cmd,
                  userData,
                  vmk_StatusToString(status));
      goto ioctl_cmd_failed;
   }

   /*
    * Copy iotl data back to UW
    */
   status = vmk_CopyToUser((vmk_VA)userData,
                           (vmk_VA)ioctlData,
                           ioctlDataLen);
   if (status != VMK_OK) {
      vmk_Warning(pimon_logger,
                  "unable to copy ioctl data back to UW: %s",
                  vmk_StatusToString(status));
      goto ioctl_vmk2uw_failed;
   }

   return VMK_OK;

ioctl_vmk2uw_failed:
ioctl_cmd_failed:
ioctl_uw2vmk_failed:
ioctl_alloc_failed:
file_data_null:
   return status;
}

/*
 ***********************************************************************
 * pimon_charDevRead --
 * 
 *    Reads from a file. Not supported for now, as we only allow writing
 *    commands of sorts to the chardev file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevRead(vmk_CharDevFdAttr *attr,
                  char *buffer,
                  vmk_ByteCount nbytes,
                  vmk_loff_t *ppos,
                  vmk_ByteCountSigned *nread)
{
   return pimon_charDevIO(attr, buffer, nbytes, ppos, nread, VMK_FALSE);
}

/*
 ***********************************************************************
 * pimon_charDevWrite --
 * 
 *    Writes to a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevWrite(vmk_CharDevFdAttr *attr,
                   char *buffer,
                   vmk_ByteCount nbytes,
                   vmk_loff_t *ppos,
                   vmk_ByteCountSigned *nwritten)
{
   return pimon_charDevIO(attr, buffer, nbytes, ppos, nwritten, VMK_TRUE);
}

/*
 ***********************************************************************
 * pimon_charDevIO --
 * 
 *    Read/write to file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevIO(vmk_CharDevFdAttr *attr,
                char *buffer,
                vmk_ByteCount nbytes,
                vmk_loff_t *ppos,
                vmk_ByteCountSigned *ndone,
                vmk_Bool isWrite)
{
   return VMK_NOT_SUPPORTED;
}

/*
 ***********************************************************************
 * pimon_charDevPoll --
 * 
 *    Polls a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_charDevPoll(vmk_CharDevFdAttr *attr,
                  vmk_PollContext pollCtx,
                  vmk_uint32 *pollMask)
{
   VMK_ReturnStatus status = VMK_OK;
   pimon_CharFileData_t *fileData = attr->clientInstanceData.ptr;
   vmk_TimerCookie tmrCookie;

   if (fileData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_logger, "file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   if (fileData->pollMask == VMK_FALSE
       && fileData->timerPending == VMK_FALSE
       && fileData->deathPending == VMK_FALSE) {
          tmrCookie.ptr = fileData;
          status = vmk_TimerSchedule(pimon_charDevTimerQueue,
                                     pimon_charDevTimerCB,
                                     tmrCookie,
                                     PIMON_CHARDEV_POLL_TIMEOUT_US,
                                     VMK_TIMER_DEFAULT_TOLERANCE,
                                     VMK_TIMER_ATTR_NONE,
                                     VMK_LOCKDOMAIN_INVALID,
                                     VMK_SPINLOCK_UNRANKED,
                                     &fileData->timer);
      if (status == VMK_OK) {
         fileData->timerPending = VMK_TRUE;
      }
      else {
         vmk_Warning(pimon_logger,
                     "failed to create poll timer: %s",
                     vmk_StatusToString(status));
         goto create_poll_timer_failed;
      }
   }

   vmk_CharDevSetPollContext(pollCtx, (void *)fileData);
   *pollMask = fileData->pollMask;

   vmk_SpinlockUnlock(fileData->lock);

   return VMK_OK;

create_poll_timer_failed:
   vmk_SpinlockUnlock(fileData->lock);

file_data_null:
   return status;
}

/*
 ***********************************************************************
 * pimon_charDevTimerCB --
 * 
 *    Callback for char device poll timer.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
void
pimon_charDevTimerCB(vmk_TimerCookie data)
{
   pimon_CharFileData_t *fileData = (pimon_CharFileData_t *)data.ptr;

   if (fileData == NULL) {
      vmk_Warning(pimon_logger, "file data null");
      goto file_data_null;
   }

   vmk_SpinlockLock(fileData->lock);

   fileData->timerPending = VMK_FALSE;
   fileData->pollMask = VMKAPI_POLL_WRITE;

   if (fileData->deathPending == VMK_FALSE) {
      /* Wake up pollers in UW */
      vmk_CharDevWakePollers(fileData);
   }
   else {
      vmk_SpinlockUnlock(fileData->lock);
      pimon_charDevFileDestroy(fileData);
      goto death_pending;
   }

   vmk_SpinlockUnlock(fileData->lock);

death_pending:
file_data_null:
   return;
}

/*
 ***********************************************************************
 * pimon_charDevFileDestroy --
 * 
 *    Destroy a file.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
void
pimon_charDevFileDestroy(pimon_CharFileData_t *fileData)
{
#ifdef PIMON_DEBUG
   {
      vmk_Log(pimon_logger,
              "destroying file %p",
              fileData);
   }
#endif /* PIMON_DEBUG */

   vmk_SpinlockDestroy(fileData->lock);
   vmk_HeapFree(pimon_heapID, fileData);
}