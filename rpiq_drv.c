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
// - Implement a lock to prevent concurrent access to mbox memory.
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

   pimon_Driver = driver;
   rpiq_Device = adapter;

   return status;
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
   rpiq_MboxBuffer_t *dmaBuf;
   vmk_MA dmaBufMA;
   vmk_uint32 mboxIn;
   vmk_uint32 mboxOut;
   vmk_uint32 mboxFullRetries = 0;
   vmk_uint32 mboxReadRetries = 0;

   // TODO: implement locking
   
   /*
    *-------------------------------------------------------------------
    * MBOX WRITE
    *-------------------------------------------------------------------
    */

   /*
    * Spin until mbox is empty
    */
   do {
      status = vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                                        RPIQ_MBOX_STATUS,
                                        &mboxStatus);
      if (status != VMK_OK) {
         vmk_Warning(pimon_Driver->logger,
                     "failed to read RPIQ mailbox status: %s",
                     vmk_StatusToString(status));
         goto mbox_write_status_failed;
      }
   } while (mboxStatus & RPIQ_MBOX_FULL);

#ifdef RPIQ_DEBUG
   {
      vmk_Log(pimon_Driver->logger,
              "allocating from heap %p, size %d, alignment %d",
              rpiq_Device->dmaHeapID,
              sizeof(*dmaBuf),
              RPIQ_DMA_MBOX_ALIGNMENT);
   }
#endif /* RPIQ_DEBUG */

   /*
    * Copy buffer to location accessible by VC DMA
    */
   dmaBuf = vmk_HeapAlign(rpiq_Device->dmaHeapID,
                          sizeof(*dmaBuf),
                          RPIQ_DMA_MBOX_ALIGNMENT);
   vmk_Memcpy(dmaBuf, buffer, sizeof(*dmaBuf));

   /*
    * Prepare mbox input data
    */
   vmk_VA2MA((vmk_VA)dmaBuf, 0, &dmaBufMA);
   mboxIn = ((vmk_uint32)dmaBufMA | (vmk_uint32)channel);

#ifdef RPIQ_DEBUG
   {
      vmk_Log(pimon_Driver->logger,
              "writing mboxOut %p to DMA buffer %p (MA %p) on channel %d",
              mboxIn,
              dmaBuf,
              dmaBufMA,
              channel);
   }
#endif /* RPIQ_DEBUG */

   /*
    * Barrier required before first write to a peripheral as per BCM2711 data-
    * sheet section 1.3.
    */
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

   /*
    * Invalidate, but do not evict dirty cache lines.
    */
   RPIQ_DMA_FLUSH_DCACHE((void *)dmaBuf);

   /*
    *-------------------------------------------------------------------
    * MBOX READ
    *-------------------------------------------------------------------
    */

   /*
    * As a precaution, we place a memory barrier before the read.
    */
   RPIQ_DMA_MEM_BARRIER();

   do {
   
      /*
       * Wait for mbox to fill up
       */
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
         if (mboxFullRetries >= RPIQ_MBOX_FULL_ATTEMPTS) {
            status = VMK_BUSY;
            vmk_Warning(pimon_Driver->logger,
                        "RPIQ mailbox not full after %d retries",
                        mboxFullRetries);
            goto mbox_full_attempts;
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
      if (mboxReadRetries >= RPIQ_MBOX_READ_ATTEMPTS) {
         status = VMK_BUSY;
         vmk_Warning(pimon_Driver->logger,
                     "RPIQ mailbox read failed after %d retries",
                     mboxReadRetries);
         goto mbox_read_attempts;
      }
   } while ((mboxOut & RPIQ_MBOX_CHAN_MASK) != channel);

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
   // TODO: Figure out why the same data comes out as went in.
   RPIQ_DMA_FLUSH_DCACHE((void *)dmaBuf);
   vmk_Memcpy(buffer, dmaBuf, sizeof(*buffer));

   vmk_HeapFree(rpiq_Device->dmaHeapID, dmaBuf);

   return VMK_OK;

mbox_response_mismatch:
mbox_read_attempts:
mbox_read_failed:
mbox_full_attempts:
mbox_read_status_failed:
mbox_write_failed:
   vmk_HeapFree(rpiq_Device->dmaHeapID, dmaBuf);

mbox_write_status_failed:
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