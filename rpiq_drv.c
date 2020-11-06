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
// - Figure out if it's possible to support arbitrary length mbox buffers.
// - Currently, the mbox buffer is copied four times: (1) from UW to kernel
//   space, (2) from driver heap to DMA heap, (3) from DMA heap to driver heap,
//   and (4) from kernel space to UW. Perhaps this could be simplified.

/*
 * rpiq_drv.c --
 * 
 *    Implementation of RPIQ hardware interface.
 */

#include "rpiq_drv.h"

/***********************************************************************/

static pimon_Driver_t *pimon_Driver;
static rpiq_Device_t *rpiq_Device;
static rpiq_MboxDMABuffer_t rpiq_MboxDMABuffer;

/*
 ***********************************************************************
 * rpiq_drvInit --
 * 
 *    Initialize the RPIQ driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_drvInit(pimon_Driver_t *driver,
             rpiq_Device_t *adapter)
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_MA dmaBufPtrMA;
   vmk_SpinlockCreateProps lockProps;

   pimon_Driver = driver;
   rpiq_Device = adapter;

   /*
    * Allocate mbox DMA buffer. We allocate once and re-use it as a work-around,
    * since repeated allocations will eventually allocate beyond 1GB. This in
    * turn causes the VC to be unable to access it.
    * 
    * TODO: Program the DMA controller & move the aperture to match the
    * allocation.
    */
   rpiq_MboxDMABuffer.ptr = vmk_HeapAlign(rpiq_Device->dmaHeapID,
                                          sizeof(*rpiq_MboxDMABuffer.ptr),
                                          RPIQ_DMA_MBOX_ALIGNMENT);

   /* Verify that mbox buffer is below 2GB */
   vmk_VA2MA((vmk_VA)rpiq_MboxDMABuffer.ptr, 0, &dmaBufPtrMA);
   if (dmaBufPtrMA > RPIQ_DMA_MAX_ADDR) {
      status = VMK_NO_MEMORY;
      vmk_Warning(pimon_Driver->logger,
                  "unable to allocate below 2GB");
      goto dma_addr_invalid;
   }

   /*
    * Init lock
    */

   lockProps.moduleID = pimon_Driver->moduleID;
   lockProps.heapID = pimon_Driver->heapID;
   status = vmk_NameInitialize(&lockProps.name, RPIQ_DMA_LOCK_NAME);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver->logger,
                  "failed to init DMA buffer lock name: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

   lockProps.type = VMK_SPINLOCK;
   lockProps.domain = VMK_LOCKDOMAIN_INVALID;
   lockProps.rank = VMK_SPINLOCK_UNRANKED;
   status = vmk_SpinlockCreate(&lockProps, &rpiq_MboxDMABuffer.lock);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver->logger,
                  "failed to create DMA buffer spinlock: %s",
                  vmk_StatusToString(status));
      goto lock_init_failed;
   }

lock_init_failed:
dma_addr_invalid:
   vmk_HeapFree(rpiq_Device->dmaHeapID, rpiq_MboxDMABuffer.ptr);
   return status;
}

/*
 ***********************************************************************
 * rpiq_drvCleanUp --
 * 
 *    Clean up the RPIQ driver.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None
 ***********************************************************************
 */
void
rpiq_drvCleanUp()
{
   vmk_SpinlockDestroy(rpiq_MboxDMABuffer.lock);
   vmk_HeapFree(rpiq_Device->dmaHeapID, rpiq_MboxDMABuffer.ptr);
}

/*
 ***********************************************************************
 * rpiq_mboxDrain --
 * 
 *    Drain the VideoCore mailbox.
 * 
 * Results:
 *    VMK_OK      on success
 *    VMK_TIMEOUT otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mboxDrain()
{
   int retries = 0;
   vmk_uint32 mboxStatus, mboxVal;

   do {
      vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                               RPIQ_MBOX_STATUS,
                               &mboxStatus);
      if (mboxStatus == RPIQ_MBOX_EMPTY) {
         return VMK_OK;
      }
      RPIQ_DMA_MEM_BARRIER();
      vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                               RPIQ_MBOX_READ,
                               &mboxVal);
   } while (retries++ < RPIQ_MBOX_MAX_RETRIES);

   return VMK_TIMEOUT;
}

/*
 ***********************************************************************
 * rpiq_mboxStatusCleared --
 * 
 *    Spin until VC mailbox status flag is clear.
 * 
 * Results:
 *    VMK_OK      on success
 *    VMK_TIMEOUT otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mboxStatusCleared(vmk_uint32 status)
{
   int retries = 0;
   vmk_uint32 mboxStatus;

   do {
      vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                               RPIQ_MBOX_STATUS,
                               &mboxStatus);
      if ((mboxStatus & status) == 0) {
         return VMK_OK;
      }
      RPIQ_DMA_MEM_BARRIER();
   } while (retries++ < RPIQ_MBOX_MAX_RETRIES);

   return VMK_TIMEOUT;
}

/*
 ***********************************************************************
 * rpiq_mboxReadChannel --
 * 
 *    Read from a VC mbox channel.
 * 
 * Results:
 *    VMK_OK      on success
 *    VMK_TIMEOUT otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mboxReadChannel(vmk_uint8 channel,
                     vmk_uint32 *mboxOut)
{
   int retries = 0;
   vmk_uint32 mboxVal;

   do {
      vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                               RPIQ_MBOX_READ,
                               &mboxVal);
      if ((mboxVal & RPIQ_MBOX_CHAN_MASK) == channel) {
         *mboxOut = mboxVal;
         return VMK_OK;
      }
      RPIQ_DMA_MEM_BARRIER();
   } while (retries++ < RPIQ_MBOX_MAX_RETRIES);

   return VMK_TIMEOUT;
}

/*
 ***********************************************************************
 * rpiq_mboxSend --
 * 
 *    Send a request to a VideoCore mailbox and receive its response.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mboxSend(rpiq_MboxChannel_t channel,   // IN
              rpiq_MboxBuffer_t *buffer)    // IN/OUT
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_uint32 mboxStatus = RPIQ_MBOX_FULL;
   rpiq_MboxDMABuffer_t *dmaBuf = &rpiq_MboxDMABuffer;
   rpiq_MboxBuffer_t *dmaBufPtr = dmaBuf->ptr;
   vmk_MA dmaBufPtrMA;
   vmk_uint32 mboxIn;
   vmk_uint32 mboxOut;
   vmk_uint32 mboxFullRetries = 0;
   vmk_uint32 mboxReadRetries = 0;

   /*
    *-------------------------------------------------------------------
    * MBOX WRITE
    *-------------------------------------------------------------------
    */

   /*
    * Drain mbox
    */
   if (rpiq_mboxDrain() != VMK_OK) {
      status = VMK_TIMEOUT;
      vmk_Warning(pimon_Driver->logger, "failed to drain mailbox");
      goto mbox_drain_timeout;
   }

   /* Lock the DMA buffer */
   vmk_SpinlockLock(dmaBuf->lock);

   /*
    * Wait for mbox to become empty
    */
   if (rpiq_mboxStatusCleared(RPIQ_MBOX_FULL) != VMK_OK) {
      status = VMK_TIMEOUT;
      vmk_Warning(pimon_Driver->logger,
                  "timeout waiting for mbox to become empty");
      goto mbox_empty_timeout;
   }

   /*
    * Copy buffer to location accessible by VC DMA
    */
   vmk_Memcpy(dmaBufPtr, buffer, sizeof(*dmaBufPtr));

   /*
    * Prepare mbox input data
    */
   vmk_VA2MA((vmk_VA)dmaBufPtr, 0, &dmaBufPtrMA);
   mboxIn = ((vmk_uint32)dmaBufPtrMA
             | RPIQ_DMA_COHERENT_ADDR
             | (vmk_uint32)channel);

#ifdef RPIQ_DEBUG
   {
      vmk_Log(pimon_Driver->logger,
              "writing mboxOut %p to DMA buffer %p (MA %p) on channel %d",
              mboxIn,
              dmaBufPtr,
              dmaBufPtrMA,
              channel);
   }
#endif /* RPIQ_DEBUG */

   RPIQ_DMA_MEM_BARRIER();

   status = vmk_MappedResourceWrite32(&rpiq_Device->mmioMappedAddr,
                                      RPIQ_MBOX_WRITE,
                                      mboxIn);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver->logger,
                  "failed to write to RPIQ mailbox: %s",
                  vmk_StatusToString(status));
      goto mbox_write_failed;
   }

   RPIQ_DMA_MEM_BARRIER();

   /*
    * Invalidate, but do not evict dirty cache lines.
    */
   RPIQ_DMA_CLEAN_DCACHE((void *)dmaBufPtr);

   /*
    *-------------------------------------------------------------------
    * MBOX READ
    *-------------------------------------------------------------------
    */

   /*
    * Wait for mbox to fill up
    */
   mboxFullRetries = 0;
   do {
      status = vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                                       RPIQ_MBOX_STATUS,
                                       &mboxStatus);
      if (status != VMK_OK) {
         vmk_Warning(pimon_Driver->logger,
                     "failed to read RPIQ mailbox status: %s",
                     vmk_StatusToString(status));
         goto mbox_read_status_failed;
      }

      ++mboxFullRetries;
      if (mboxFullRetries >= RPIQ_MBOX_MAX_RETRIES) {
         status = VMK_BUSY;
         vmk_Warning(pimon_Driver->logger,
                     "RPIQ mailbox not full after %d retries",
                     mboxFullRetries);
         //goto mbox_full_attempts;
         break;
      }
   } while (mboxStatus & RPIQ_MBOX_EMPTY);

#ifdef RPIQ_DEBUG
      {
         vmk_Log(pimon_Driver->logger,
                 "RPIQ mailbox full after %d retries, attempting read",
                 mboxFullRetries);
      }
#endif /* RPIQ_DEBUG */

   /*
    * Perform read
    */
   RPIQ_DMA_MEM_BARRIER();
   do {
      status = vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                                        RPIQ_MBOX_READ,
                                        &mboxOut);
      RPIQ_DMA_MEM_BARRIER();
      if (status != VMK_OK) {
         vmk_Warning(pimon_Driver->logger,
                     "failed to read from RPIQ mailbox: %s",
                     vmk_StatusToString(status));
         goto mbox_read_failed;
      }

      ++mboxReadRetries;
      if (mboxReadRetries >= RPIQ_MBOX_MAX_RETRIES) {
         status = VMK_BUSY;
         vmk_Warning(pimon_Driver->logger,
                     "RPIQ mailbox read failed after %d retries",
                     mboxReadRetries);
         goto mbox_read_attempts;
      }
   } while ((mboxOut & RPIQ_MBOX_CHAN_MASK) != channel);
   RPIQ_DMA_MEM_BARRIER();

#ifdef RPIQ_DEBUG
   {
      vmk_Log(pimon_Driver->logger,
              "Read value %p from RPIQ mailbox",
              mboxOut);
   }
#endif /* RPIQ_DBUG */

   if (mboxOut != mboxIn) {
      status = VMK_FAILURE;
      vmk_Warning(pimon_Driver->logger,
                  "mismatch in mbox input %p and output %p",
                  mboxIn,
                  mboxOut);
      goto mbox_response_mismatch;
   }

   /*
    * Flush cache and copy data back to in/out buffer
    */
   RPIQ_DMA_FLUSH_DCACHE((void *)dmaBufPtr);
   if (dmaBufPtr->header.requestResponse == RPIQ_PROCESS_REQ) {
      status = VMK_FAILURE;
      vmk_Warning(pimon_Driver->logger,
                  "no response data received");
      goto mbox_no_response;
   }
   vmk_Memcpy(buffer, dmaBufPtr, sizeof(*buffer));

   /* Unlock the DMA buffer */
   vmk_SpinlockUnlock(dmaBuf->lock);

   return VMK_OK;

mbox_no_response:
mbox_response_mismatch:
mbox_read_attempts:
mbox_read_failed:
mbox_read_status_failed:
mbox_write_failed:
mbox_empty_timeout:
mbox_drain_timeout:
   vmk_SpinlockUnlock(dmaBuf->lock);
   return status;
}

/*
 ***********************************************************************
 * rpiq_mmioOpenCB --
 * 
 *    Callback used by char dev driver when the /dev/vmgfx32 file is
 *    opened.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mmioOpenCB(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;

   vmk_Log(pimon_Driver->logger, "Opening file.");

   return status;
}

/*
 ***********************************************************************
 * rpiq_mmioCloseCB --
 * 
 *    Callback used by char dev driver when the /dev/vmgfx32 file is
 *    closed.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mmioCloseCB(vmk_CharDevFdAttr *attr)
{
   VMK_ReturnStatus status = VMK_OK;

   vmk_Log(pimon_Driver->logger, "Closing file.");

   return status;
}

/*
 ***********************************************************************
 * rpiq_mmioIoctlCB --
 * 
 *    Callback used by char dev driver for I/O control. The RPIQ char dev
 *    file only supports access via ioctl, since it allows for sending
 *    and receiving data structures easily.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mmioIoctlCB(unsigned int channel,       // IN
                 void *data,                 // IN/OUT
                 vmk_ByteCount ioctlDataLen) // IN
{
   VMK_ReturnStatus status = VMK_OK;
   rpiq_MboxBuffer_t *buffer;

   // TODO: add support for arbitrary length mailbox data, to allow for things
   // like requesting console data.

   /*
    * Sanity checks
    */

   if (data == NULL || ioctlDataLen != sizeof(*buffer)) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_Driver->logger, "invalid ioctl data");
      goto invalid_ioctl_data;
   }

   if (channel < RPIQ_CHAN_MBOX_MIN
       || channel > RPIQ_CHAN_MBOX_MAX) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_Driver->logger, "ioctl command %d invalid", channel);
      goto invalid_ioct_channel;
   }

   // TODO: Should we check the tag validity too?
   buffer = (rpiq_MboxBuffer_t *)data;
   if (buffer->endTag != 0
       || buffer->padding[0] != 0
       || buffer->padding[1] != 0
       || buffer->header.bufLen != sizeof(*buffer)) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_Driver->logger, "invalid mailbox buffer");

      goto invalid_mbox_buffer;
   }

#ifdef RPIQ_DEBUG
   {
      vmk_Log(pimon_Driver->logger,
               "bufLen=%d rqRp=%d tag=0x%x rpLen=%d rqLen=%d endTag=%d"
               " padding[0]=%d padding[1]=%d",
               buffer->header.bufLen,
               buffer->header.requestResponse,
               buffer->header.tag,
               buffer->header.responseLen,
               buffer->header.requestLen,
               buffer->endTag,
               buffer->padding[0],
               buffer->padding[1]);
   }
#endif /* RPIQ_DEBUG */

   /*
    * Mbox I/O
    */

   status = rpiq_mboxSend(channel, buffer);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver->logger,
                  "unable to send to mailbox: %s",
                  vmk_StatusToString(status));
      goto mbox_send_failed;
   }

   return VMK_OK;

mbox_send_failed:
invalid_mbox_buffer:
invalid_ioct_channel:
invalid_ioctl_data:
   return status;
}

/*
 ***********************************************************************
 * rpiq_mmioReadCB --
 * 
 *    Callback used by char dev driver when the file is read. Currently
 *    not supported, as all I/O is done via ioctl.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mmioReadCB(char *buffer,
                vmk_ByteCount nbytes,
                vmk_loff_t *ppos,
                vmk_ByteCountSigned *nread)
{
   return VMK_NOT_SUPPORTED;
}

/*
 ***********************************************************************
 * rpiq_mmioWrite --
 * 
 *    Callback used by char dev driver when the file is written to.
 *    Currently not supported, as all I/O is done via ioctl.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_mmioWriteCB(char *buffer,
                 vmk_ByteCount nbytes,
                 vmk_loff_t *ppos,
                 vmk_ByteCountSigned *nwritten)
{
   return VMK_NOT_SUPPORTED;
}