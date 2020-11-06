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

// TODO:
// - Implement a fail-safe for when the requested DMA heap is above 1GB, and
//   program the DMA controller to move the DMA aperture upwards to match the
//   heap start MA.

/*
 * pimon_os.c --
 * 
 *    Module initialization and other OS stuff.
 */

#include "pimon.h"
#include "pimon_types.h"
#include "rpiq_drv.h"
#include "pimon_charDev.h"

/***********************************************************************/

static pimon_Driver_t pimon_Driver;
static rpiq_Device_t rpiq_Device;

/* For RPIQ char dev */
static pimon_CharDev_t rpiq_CharDev;
static vmk_BusType rpiq_logicalBusType;

VMK_ReturnStatus pimon_attachDevice(vmk_Device device);
VMK_ReturnStatus pimon_scanDevice(vmk_Device device);
VMK_ReturnStatus pimon_detachDevice(vmk_Device device);
VMK_ReturnStatus pimon_quiesceDevice(vmk_Device device);
VMK_ReturnStatus pimon_startDevice(vmk_Device device);
void pimon_forgetDevice(vmk_Device device);

static vmk_DriverOps pimon_DriverOps = {
   .attachDevice = pimon_attachDevice,
   .scanDevice = pimon_scanDevice,
   .detachDevice = pimon_detachDevice,
   .quiesceDevice = pimon_quiesceDevice,
   .startDevice = pimon_startDevice,
   .forgetDevice = pimon_forgetDevice,
};

/*
 * Callbacks gluing the chardev driver to the RPIQ driver.
 */
static pimon_CharDevCallbacks_t rpiq_CharDevCBs = {
   .open = rpiq_mmioOpenCB,
   .close = rpiq_mmioCloseCB,
   .ioctl = rpiq_mmioIoctlCB,
   .read = rpiq_mmioReadCB,
   .write = rpiq_mmioWriteCB,
};

/*
 ***********************************************************************
 * init_module --
 * 
 *    Entry point for vmkernel module.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
int
init_module(void)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_HeapID heapID;
   vmk_HeapCreateProps heapProps;
   vmk_DriverProps driverProps;
   vmk_LogProperties logProps;
   vmk_Name charDevBusName;

   status = vmk_NameInitialize(&pimon_Driver.driverName, PIMON_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);

   pimon_Driver.moduleID = vmk_ModuleCurrentID;
   rpiq_Device.initialized = VMK_FALSE;

   /*
    * Create module heap
    */

   heapProps.type = VMK_HEAP_TYPE_SIMPLE;
   status = vmk_NameInitialize(&heapProps.name, PIMON_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);
   heapProps.module = pimon_Driver.moduleID;
   heapProps.initial = PIMON_HEAP_INITIAL_SIZE;
   heapProps.creationTimeoutMS = VMK_TIMEOUT_NONBLOCKING;
   heapProps.max = PIMON_HEAP_MAX_SIZE;

   status = vmk_HeapCreate(&heapProps, &heapID);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to create heap: %s",
                         PIMON_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto heap_create_failed;
   }

   vmk_ModuleSetHeapID(pimon_Driver.moduleID, heapID);
   VMK_ASSERT(vmk_ModuleGetHeapID(pimon_Driver.moduleID) == heapID);
   pimon_Driver.heapID = heapID;

   /*
    * Set up logger
    */

   status = vmk_NameInitialize(&logProps.name, PIMON_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);
   logProps.module = pimon_Driver.moduleID;
   logProps.heap = pimon_Driver.heapID;
   logProps.defaultLevel = 0;
   logProps.throttle = NULL;

   status = vmk_LogRegister(&logProps, &pimon_Driver.logger);
   if (status != VMK_OK) {
      vmk_WarningMessage("%s: %s: failed to register logger: %s",
                         PIMON_DRIVER_NAME,
                         __FUNCTION__,
                         vmk_StatusToString(status));
      goto logger_reg_failed;
   }

   /*
    * Init logical bus for char dev
    */

   status = vmk_NameInitialize(&charDevBusName, VMK_LOGICAL_BUS_NAME);
   status = vmk_BusTypeFind(&charDevBusName, &rpiq_logicalBusType);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "failed to find logical bus: %s",
                  vmk_StatusToString(status));
      goto chardev_bus_failed;
   }

   /*
    * Register driver
    */

   vmk_Memset(&driverProps, 0, sizeof(driverProps));
   status = vmk_NameInitialize(&driverProps.name, PIMON_DRIVER_NAME);
   VMK_ASSERT(status == VMK_OK);
   driverProps.moduleID = pimon_Driver.moduleID;
   driverProps.ops = &pimon_DriverOps;

   status = vmk_DriverRegister(&driverProps,
                               &(pimon_Driver.driverHandle));
   if (status == VMK_OK) {
      vmk_Log(pimon_Driver.logger,
              "driver registration successful",
              PIMON_DRIVER_NAME,
              __FUNCTION__);
   }
   else {
      vmk_Warning(pimon_Driver.logger,
                  "driver registration failed: %s",
                  vmk_StatusToString(status));
      goto driver_register_failed;
   }

   /*
    * Init RPIQ driver & char dev driver
    */
   rpiq_drvInit(&pimon_Driver, &rpiq_Device);
   pimon_charDevInit(pimon_Driver.moduleID,
                     pimon_Driver.heapID,
                     pimon_Driver.logger);

   return VMK_OK;

driver_register_failed:
chardev_bus_failed:
   vmk_LogUnregister(pimon_Driver.logger);

logger_reg_failed:
   vmk_HeapDestroy(pimon_Driver.heapID);

heap_create_failed:
   return 0;
}

/*
 ***********************************************************************
 * cleanup_module --
 * 
 *    Cleanup code for when module is unloaded.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
void
cleanup_module(void)
{
   vmk_HeapDestroy(rpiq_Device.dmaHeapID);
   vmk_ACPIUnmapIOResource(pimon_Driver.moduleID, rpiq_Device.acpiDevice, 0);
   vmk_LogUnregister(pimon_Driver.logger);
   vmk_BusTypeRelease(rpiq_logicalBusType);
   vmk_HeapDestroy(pimon_Driver.heapID);
   vmk_DriverUnregister(pimon_Driver.driverHandle);
}

/*
 ***********************************************************************
 * pimon_attachDevice --
 * 
 *    Driver callback that attaches RPIQ logical device to the driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_attachDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_ACPIDevice acpiDev;
   vmk_ACPIInfo acpiInfo;
   vmk_MappedResourceAddress mappedAddr;
   vmk_HeapAllocationDescriptor heapAllocDesc;
   vmk_ByteCount dmaHeapSize;
   vmk_HeapCreateProps heapProps;
   vmk_HeapID dmaHeapID;
   vmk_SpinlockCreateProps mboxLockProps;
   vmk_Lock mboxLock;
   rpiq_Device_t *adapter = &rpiq_Device;

   /*
    * Since there is only one RPIQ device on the system board, we're making
    * sure we're not initializing more.
    */
   if (adapter->initialized != VMK_FALSE) {
      vmk_Warning(pimon_Driver.logger,
                  "RPIQ device %p already initialized as %p",
                  device,
                  adapter->vmkDevice);
      status = VMK_EXISTS;
      goto device_already_exists;
   }

   /*
    * Get vmk ACPI dev
    */
   status = vmk_DeviceGetRegistrationData(device, (vmk_AddrCookie *)&acpiDev);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "failed to get ACPI dev for vmk dev %p: %s",
                  device,
                  vmk_StatusToString(status));
      goto get_acpi_dev_failed;
   }

   /*
    * Find out if this is an ACPI dev
    */
   status = vmk_ACPIQueryInfo(acpiDev, &acpiInfo);
   if (status != VMK_OK) {
      goto not_an_acpi_dev;
   }

   vmk_Log(pimon_Driver.logger,
           "retrieved ACPI dev %p for vmk dev %p",
           acpiDev,
           device);

   /*
    * Map io resources
    */

   status = vmk_ACPIMapIOResource(pimon_Driver.moduleID,
                                  acpiDev,
                                  0,
                                  &mappedAddr);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "unable to acquire io resources"
                  " for device %p: %s",
                  device,
                  vmk_StatusToString(status));
      goto iores_map_failed;
   }
   else if (mappedAddr.type != VMK_IORESOURCE_MEM) {
      vmk_Warning(pimon_Driver.logger,
                  "mapped io space %p of type %d, expecting %d",
                  mappedAddr,
                  mappedAddr.type,
                  VMK_IORESOURCE_MEM);
      goto iores_map_failed;
   }

   vmk_Log(pimon_Driver.logger,
           "mapped io space at %p of length %d for device %p",
           (void*)mappedAddr.address.vaddr,
           mappedAddr.len,
           device);

   /*
    * Create DMA heap for device
    */

   heapProps.type = VMK_HEAP_TYPE_CUSTOM;
   heapProps.module = pimon_Driver.moduleID;
   heapProps.creationTimeoutMS = VMK_TIMEOUT_NONBLOCKING;
   heapProps.typeSpecific.custom.physContiguity = VMK_MEM_PHYS_CONTIGUOUS;
   
   /*
    * The DMA addresses accessible by the DMA engine are below 1GB by default,
    * so the below may not be sufficient.
    */
   // TODO: Make more robust by programming DMA controller to move aperture.
   heapProps.typeSpecific.custom.physRange = VMK_PHYS_ADDR_BELOW_2GB;

   status = vmk_NameInitialize(&heapProps.name, RPIQ_DMA_HEAP_NAME);
   VMK_ASSERT(status == VMK_OK);

   heapAllocDesc.size = sizeof(rpiq_MboxBuffer_t);
   heapAllocDesc.alignment = RPIQ_DMA_MBOX_ALIGNMENT;
   heapAllocDesc.count = RPIQ_DMA_MBOX_OBJ_MAX;

   status = vmk_HeapDetermineMaxSize(&heapAllocDesc, 1, &dmaHeapSize);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "failed to determine DMA max heap size: %s",
                  vmk_StatusToString(status));
      goto dma_heap_size_failed;
   }

   heapProps.initial = dmaHeapSize;
   heapProps.max = dmaHeapSize;

   status = vmk_HeapCreate(&heapProps, &dmaHeapID);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "failed to create DMA heap: %s",
                  vmk_StatusToString(status));
      goto dma_heap_create_failed;
   }

   /*
    * Create Mbox lock
    */

   // TODO: consider making this an MCS lock since the latency is quite high.
   mboxLockProps.moduleID = pimon_Driver.moduleID;
   mboxLockProps.heapID = pimon_Driver.heapID;
   status = vmk_NameInitialize(&mboxLockProps.name, PIMON_DRIVER_NAME);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "failed to init lock name: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   mboxLockProps.type = VMK_SPINLOCK;
   mboxLockProps.domain = VMK_LOCKDOMAIN_INVALID;
   mboxLockProps.rank = VMK_SPINLOCK_UNRANKED;
   status = vmk_SpinlockCreate(&mboxLockProps, &mboxLock);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "failed to create spinlock: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   /*
    * Init adapter
    */
   
   vmk_Memset(adapter, 0, sizeof(*adapter));
   adapter->vmkDevice = device;
   adapter->acpiDevice = acpiDev;
   adapter->acpiInfo = acpiInfo;
   vmk_AtomicWrite64(&adapter->refCount, 0);

   vmk_Memcpy(&adapter->mmioMappedAddr,
              &mappedAddr,
              sizeof(adapter->mmioMappedAddr));
   adapter->mmioBase = (void *)mappedAddr.address.vaddr;
   adapter->mmioLen = mappedAddr.len;
   adapter->dmaHeapID = dmaHeapID;
   vmk_Memcpy(&adapter->mboxLock,
              mboxLock,
              sizeof(adapter->mboxLock));
   adapter->initialized = VMK_TRUE;

   /*
    * Attach device
    */
   status = vmk_DeviceSetAttachedDriverData(adapter->vmkDevice, adapter);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver.logger,
                  "failed to attach device %p: %s",
                  adapter->vmkDevice,
                  vmk_StatusToString(status));
      goto device_attach_failed;
   }

#ifdef PIMON_DEBUG
   {
      vmk_Log(pimon_Driver.logger,
              "DMA heap %p",
              adapter->dmaHeapID);
   }
#endif /* PIMON_DEBUG */

   return VMK_OK;

device_attach_failed:
lock_init_failed:
   vmk_HeapDestroy(rpiq_Device.dmaHeapID);

dma_heap_create_failed:
dma_heap_size_failed:
   vmk_ACPIUnmapIOResource(pimon_Driver.moduleID, acpiDev, 0);

iores_map_failed:
not_an_acpi_dev:
get_acpi_dev_failed:
device_already_exists:
   vmk_Warning(pimon_Driver.logger,
               "no device attached",
               PIMON_DRIVER_NAME,
               __FUNCTION__);
   return status;
}

/*
 ***********************************************************************
 * pimon_scanDevice --
 * 
 *    Register char dev.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_scanDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;
   pimon_CharDevPriv_t *privData;
   pimon_CharDevProps_t charDevProps;

   /* We only allow data to be transmitted in a particular size */
   privData = vmk_HeapAlloc(pimon_Driver.heapID, sizeof(*privData));
   if (privData == NULL) {
      status = VMK_NO_MEMORY;
      vmk_Warning(pimon_Driver.logger, "unable to allocate char dev priv data");
      goto priv_alloc_failed;
   }
   privData->ioctlDataLen = sizeof(rpiq_MboxBuffer_t);

   charDevProps.driverHandle = pimon_Driver.driverHandle;
   charDevProps.logicalBusType = rpiq_logicalBusType;
   charDevProps.parentDevice = device;
   charDevProps.charDev = &rpiq_CharDev;
   charDevProps.logicalPort = 0;
   charDevProps.privData = privData;
   charDevProps.callbacks = &rpiq_CharDevCBs;
   
   status = pimon_charDevRegister(&charDevProps);

priv_alloc_failed:
   return status;
}

/*
 ***********************************************************************
 * pimon_detachDevice --
 * 
 *    Detaches the device from the driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_detachDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * pimon_quiesceDevice --
 * 
 *    Do nothing.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_quiesceDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * pimon_startDevice --
 *
 *    Start the GPIO device.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
pimon_startDevice(vmk_Device device)
{
   VMK_ReturnStatus status = VMK_OK;

   return status;
}

/*
 ***********************************************************************
 * pimon_forgetDevice --
 * 
 *    Do nothing.
 * 
 * Results:
 *    None.
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
void
pimon_forgetDevice(vmk_Device device)
{
   return;
}