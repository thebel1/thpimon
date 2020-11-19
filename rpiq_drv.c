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
// - Figure out how to program the RPi's DMA controller to add robustness to the
//   DMA logic.
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
                                          RPIQ_MBOX_BUFFER_SIZE,
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
      RPIQ_MEM_BARRIER();
      vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                               RPIQ_MBOX_READ,
                               &mboxVal);
   } while (++retries < RPIQ_MBOX_MAX_RETRIES);

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
      RPIQ_MEM_BARRIER();
   } while (++retries < RPIQ_MBOX_MAX_RETRIES);

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
rpiq_mboxSend(rpiq_MboxChannel_t channel, // IN
              rpiq_MboxBuffer_t *buffer)  // IN/OUT)
{
   VMK_ReturnStatus status = VMK_OK;
   rpiq_MboxDMABuffer_t *dmaBuf = &rpiq_MboxDMABuffer;
   rpiq_MboxBuffer_t *dmaBufPtr = dmaBuf->ptr;
   vmk_MA dmaBufPtrMA;
   vmk_uint32 mboxIn;
   vmk_uint32 mboxOut;
   vmk_uint32 mboxReadRetries = 0;

   // <DEBUG>
   asm volatile (".word 0" ::: "memory");
   // </DEBUG>


   if (buffer->header.bufLen > RPIQ_MBOX_BUFFER_SIZE) {
      status = VMK_NOT_SUPPORTED;
      vmk_Warning(pimon_Driver->logger,
                  "cannot allocate mbox buffer of size %d",
                  buffer->header.bufLen);
      goto buf_too_large;
   }

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
    *-------------------------------------------------------------------
    * MBOX WRITE
    *-------------------------------------------------------------------
    */

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
   vmk_Memcpy(dmaBufPtr, buffer, buffer->header.bufLen);

   /*
    * Prepare mbox input data
    */
   vmk_VA2MA((vmk_VA)dmaBufPtr, 0, &dmaBufPtrMA);
   mboxIn = ((vmk_uint32)dmaBufPtrMA
             | RPIQ_DMA_COHERENT_ADDR
             | (vmk_uint32)channel);

   RPIQ_MEM_BARRIER();

   vmk_MappedResourceWrite32(&rpiq_Device->mmioMappedAddr,
                             RPIQ_MBOX_WRITE,
                             mboxIn);

   RPIQ_MEM_BARRIER();

   /*
    * This needs to be here.
    */
   RPIQ_INVAL_DCACHE((void *)dmaBufPtr);

   /*
    *-------------------------------------------------------------------
    * MBOX READ
    *-------------------------------------------------------------------
    */

   /*
    * Wait for mbox to fill up
    */
   if (rpiq_mboxStatusCleared(RPIQ_MBOX_EMPTY) != VMK_OK) {
      status = VMK_TIMEOUT;
      vmk_Warning(pimon_Driver->logger,
                  "timeout waiting for mbox to become full");
      goto mbox_full_timeout;
   }

   RPIQ_MEM_BARRIER();

   /*
    * Perform read
    */
   do {
      vmk_MappedResourceRead32(&rpiq_Device->mmioMappedAddr,
                               RPIQ_MBOX_READ,
                               &mboxOut);
      
      RPIQ_MEM_BARRIER();

      ++mboxReadRetries;
      if (mboxReadRetries >= RPIQ_MBOX_MAX_RETRIES) {
         status = VMK_TIMEOUT;
         vmk_Warning(pimon_Driver->logger,
                     "RPIQ mailbox read failed after %d retries",
                     mboxReadRetries);
         goto mbox_read_attempts;
      }
   } while ((mboxOut & RPIQ_MBOX_CHAN_MASK) != channel);

   RPIQ_MEM_BARRIER();

   /*
    * Something went wrong, in & out buffers don't match
    */
   if (mboxOut != mboxIn) {
      status = VMK_FAILURE;
      vmk_Warning(pimon_Driver->logger,
                  "mismatch in mbox input %p and output %p",
                  mboxIn,
                  mboxOut);
      goto mbox_response_mismatch;
   }

   /*
    * Invalidate cache and copy data back to in/out buffer
    */

   RPIQ_INVAL_DCACHE((void *)dmaBufPtr);
   vmk_Memcpy(buffer, dmaBufPtr, buffer->header.bufLen);
   if (buffer->header.requestResponse != RPIQ_MBOX_SUCCESS) {
      status = VMK_FAILURE;
      vmk_Warning(pimon_Driver->logger,
                  "transaction error: 0x%x",
                  buffer->header.requestResponse);
   }

   /* Unlock the DMA buffer */
   vmk_SpinlockUnlock(dmaBuf->lock);

   return VMK_OK;

mbox_response_mismatch:
mbox_read_attempts:
mbox_full_timeout:
mbox_empty_timeout:
   vmk_SpinlockUnlock(dmaBuf->lock);

mbox_drain_timeout:
buf_too_large:
   return status;
}

/*
 ***********************************************************************
 * rpiq_fbufAlloc --
 * 
 *    Allocate the framebuffer and copy it.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_fbufAlloc(rpiq_FrameBuffer_t *fbuf) // OUT
{
   VMK_ReturnStatus status = VMK_OK;
   vmk_MapRequest mapReq;
   vmk_uint32 buffer[9];
   vmk_uint32 width, height, pitch, bpp;
   vmk_MA fbufMA;
   vmk_uint32 fbufLen;
   vmk_MPN firstMPN;
   vmk_MpnRange mpnRange;
   vmk_VA fbufVA;

   // TODO: try to get the mbox commands into one transation.

   /*
    * Get dimensions
    */

   buffer[0] = 4 * 8;
   buffer[1] = RPIQ_PROCESS_REQ;
   buffer[2] = RPIQ_MBOX_TAG_GET_FB_PHYS;
   buffer[3] = 2 * sizeof(vmk_uint32);
   buffer[4] = 0;
   buffer[5] = 0;
   buffer[6] = 0;
   buffer[7] = 0;
   buffer[8] = 0;

   status = rpiq_mboxSend(RPIQ_CHAN_MBOX_PROP_ARM2VC,
                          (rpiq_MboxBuffer_t *)&buffer);

   width = buffer[5];
   height = buffer[6];

   /*
    * Get pitch
    */

   buffer[0] = 4 * 8;
   buffer[1] = RPIQ_PROCESS_REQ;
   buffer[2] = RPIQ_MBOX_TAG_GET_PITCH;
   buffer[3] = 1 * sizeof(vmk_uint32);
   buffer[4] = 0;
   buffer[5] = 0;
   buffer[6] = 0;
   buffer[7] = 0;
   buffer[8] = 0;

   status |= rpiq_mboxSend(RPIQ_CHAN_MBOX_PROP_ARM2VC,
                           (rpiq_MboxBuffer_t *)&buffer);

   pitch = buffer[5];

   /*
    * Set bits per pixel
    */

   buffer[0] = 4 * 8;
   buffer[1] = RPIQ_PROCESS_REQ;
   buffer[2] = RPIQ_MBOX_TAG_GET_FB_DEPTH;
   buffer[3] = 1 * sizeof(vmk_uint32);
   buffer[4] = 0;
   buffer[5] = 0;
   buffer[6] = 0;
   buffer[7] = 0;
   buffer[8] = 0;

   status |= rpiq_mboxSend(RPIQ_CHAN_MBOX_PROP_ARM2VC,
                           (rpiq_MboxBuffer_t *)&buffer);

   bpp = buffer[5];

   /*
    * Get frame buffer
    */

   buffer[0] = 4 * 8;
   buffer[1] = RPIQ_PROCESS_REQ;
   buffer[2] = RPIQ_MBOX_TAG_ALLOC_FB;
   buffer[3] = 2 * sizeof(vmk_uint32);
   buffer[4] = 0;
   buffer[5] = 0;
   buffer[6] = 0;
   buffer[7] = 0;
   buffer[8] = 0;

   status |= rpiq_mboxSend(RPIQ_CHAN_MBOX_PROP_ARM2VC,
                           (rpiq_MboxBuffer_t *)&buffer);

   fbufMA = buffer[5] ^ RPIQ_DMA_COHERENT_ADDR;
   fbufLen = buffer[6];

   if (status != VMK_OK) {
      status = VMK_FAILURE;
      vmk_Warning(pimon_Driver->logger, "VC mailbox transaction(s) failed");
      goto mbox_txn_failed;
   }

   /*
    * Map the returned frame buffer
    */

   firstMPN = vmk_MA2MPN(fbufMA);

   vmk_Memset(&mpnRange, 0, sizeof(mpnRange));
   mpnRange.startMPN = firstMPN;
   mpnRange.numPages = VMK_UTIL_ROUNDUP(fbufLen, VMK_PAGE_SIZE) / VMK_PAGE_SIZE;

   RPIQ_DEBUG_LOG(pimon_Driver->logger,
                  "mapping MA %p: firstMPN %p, %d MPNs",
                  fbufMA, firstMPN, mpnRange.numPages);

   vmk_Memset(&mapReq, 0, sizeof(mapReq));
   mapReq.mapType = VMK_MAPTYPE_DEFAULT;
   mapReq.mapAttrs = VMK_MAPATTRS_READONLY;
   mapReq.numElements = 1;
   mapReq.mpnRanges = &mpnRange;
   mapReq.reservation = NULL;

   status = vmk_Map(pimon_Driver->moduleID, &mapReq, &fbufVA);
   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver->logger,
                  "failed to map frame buffer: %s",
                  vmk_StatusToString(status));
      goto fbuf_map_failed;
   }

   /*
    * Copy the frame buffer
    */

   fbuf->width = width;
   fbuf->height = height;
   fbuf->pitch = pitch;
   fbuf->bpp = bpp;
   fbuf->rawLen = fbufLen;

   RPIQ_DEBUG_LOG(pimon_Driver->logger,
                  "allocating frame buffer of len %d",
                  fbuf->rawLen);
   
   fbuf->raw = vmk_HeapAlloc(pimon_Driver->heapID,
                             fbuf->rawLen);
   if (fbuf->raw == NULL) {
      status = VMK_NO_MEMORY;
      vmk_Warning(pimon_Driver->logger,
                  "unable to allocate frame buffer: %s",
                  vmk_StatusToString(status));
      goto fbuf_alloc_failed;
   }

   RPIQ_DEBUG_LOG(pimon_Driver->logger,
                  "copying frame buffer from %p (len %p) to range %p - %p",
                  fbufVA, fbuf->rawLen, fbuf->raw,
                  (char *)fbuf->raw + fbuf->rawLen);

   vmk_Memcpy((void *)fbuf->raw,
               (void *)fbufVA,
               fbuf->rawLen);

   /*
    * Unmap the frame buffer
    */
   vmk_Unmap(fbufVA);

   return VMK_OK;

fbuf_alloc_failed:
fbuf_map_failed:
mbox_txn_failed:
   return status;
}

/*
 ***********************************************************************
 * rpiq_fbufFree --
 * 
 *    Free a copied framebuffer.
 * 
 * Results:
 *    None.
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
void
rpiq_fbufFree(rpiq_FrameBuffer_t *fbuf)
{
   vmk_HeapFree(pimon_Driver->heapID, fbuf->raw);
}

/*
 ***********************************************************************
 * rpiq_fbufToBitmap --
 * 
 *    Convert a raw frame buffer into a bitmap.
 * 
 * Results:
 *    VMK_OK   on success, error code otherwise
 * 
 * Side Effects:
 *    None.
 ***********************************************************************
 */
VMK_ReturnStatus
rpiq_fbufToBitmap(rpiq_FrameBuffer_t *fbuf,     // IN
                  rpiq_Bitmap_t *bitmap,        // IN/OUT
                  vmk_ByteCount *bitmapLenOut)  // OUT
{
   VMK_ReturnStatus status = VMK_OK;
   rpiq_BitmapHeader_t *bitmapHeader;
   rpiq_FrameBufferPixel_t *fbufBase, *fbufPixel;
   vmk_ByteCount paddingLen, bitmapLen;
   char *image;
   int row, col;

   if (fbuf->raw == NULL
       || fbuf->rawLen == 0
       || fbuf->width == 0
       || fbuf->height == 0) {
      status = VMK_FAILURE;
      vmk_Warning(pimon_Driver->logger, "invalid frame buffer");
      goto fbuf_invalid;
   }

   /*
    * Determine bitmap len
    */
   paddingLen = fbuf->width & 3;
   bitmapLen = (((fbuf->width * 3) + paddingLen) * fbuf->height)
               + sizeof(*bitmapHeader);

   /*
    * Ensure enough space was allocated for the bitmap
    */
   if (bitmap->bitmapLen >= bitmapLen) {
      bitmap->bitmapLen = bitmapLen;
   }
   else {
      status = VMK_NO_MEMORY;
      vmk_Warning(pimon_Driver->logger,
                  "bitmap buffer overflow; data size %d, alloc size %d",
                  bitmapLen, bitmap->bitmapLen);
      goto bitmap_buf_overflow;
   }

   RPIQ_DEBUG_LOG(pimon_Driver->logger,
                  "bitmap buffer len %d; frame buffer len %d",
                  bitmapLen,
                  fbuf->rawLen);

   /*
    * Set bitmap header
    */

   bitmapHeader                  = (rpiq_BitmapHeader_t *)&bitmap->bitmap;
   bitmapHeader->charB           = 'B';
   bitmapHeader->charM           = 'M';
   bitmapHeader->len             = bitmapLen;
   bitmapHeader->reserved[0]     = 0;
   bitmapHeader->reserved[1]     = 0;
   bitmapHeader->imageOffset     = sizeof(*bitmapHeader);
   bitmapHeader->headerLen       = sizeof(*bitmapHeader)
                                   - PIMON_OFFSET_OF(typeof(*bitmapHeader),
                                                     headerLen);
   bitmapHeader->width           = fbuf->width;
   bitmapHeader->height          = fbuf->height;
   bitmapHeader->planes          = 1;
   bitmapHeader->bitsPerPixel    = 24;
   bitmapHeader->compressionType = 0;
   bitmapHeader->imageLen        = bitmapLen - sizeof(*bitmapHeader);
   bitmapHeader->xPixelsPerMeter = 0;
   bitmapHeader->yPixelsPerMeter = 0;
   bitmapHeader->numColors       = 0;
   bitmapHeader->importantColors = 0;

   /*
    * Perform conversion to bitmap
    */

   image = (char *)bitmapHeader + sizeof(*bitmapHeader);
   fbufBase = (rpiq_FrameBufferPixel_t *)fbuf->raw;
   for (row = 0; row < fbuf->height; ++row) {
      fbufPixel = &fbufBase[(fbuf->height - row - 1) * fbuf->width];

      RPIQ_DEBUG_LOG(pimon_Driver->logger,
                     "row %d fbufPixel %p fbufBase %p max %p",
                     row, fbufPixel, fbufBase,
                     (char *)fbufBase + (fbuf->height - 1) * fbuf->width);

      for (col = 0; col < fbuf->width; ++col) {
         *image++ = fbufPixel->blue;
         *image++ = fbufPixel->green;
         *image++ = fbufPixel->red;
         ++fbufPixel;
      }

      image += paddingLen;
   }

   /*
    * Return bitmap length
    */
   if (bitmapLenOut != NULL) {
      *bitmapLenOut = bitmapLen;
   }

bitmap_buf_overflow:
fbuf_invalid:
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
rpiq_mmioIoctlCB(unsigned int cmd,                 // IN
                 pimon_IoctlHeader_t *ioctlData)   // IN/OUT
{
   VMK_ReturnStatus status = VMK_OK;
   unsigned int channel;
   rpiq_ScreenshotIoctlData_t *screenshotIoctlData = (rpiq_ScreenshotIoctlData_t *)ioctlData;
   rpiq_FrameBuffer_t fbuf;
   vmk_ByteCount bitmapLen;

   /*
    * Sanity checks
    */

   if (ioctlData == NULL) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_Driver->logger, "invalid ioctl data");
      goto invalid_ioctl_data;
   }

   if (cmd < RPIQ_IOCTL_CMD_MIN
       || cmd > RPIQ_IOCTL_CMD_MAX) {
      status = VMK_BAD_PARAM;
      vmk_Warning(pimon_Driver->logger, "ioctl command %d invalid", cmd);
      goto invalid_ioct_channel;
   }

   /*
    * Execute ioctl command
    */
   
   channel = cmd & RPIQ_MBOX_CHAN_MASK;
   switch (cmd) {

      /* Print screen */
      case RPIQ_CMD_PRINT_SCRN:
         rpiq_fbufAlloc(&fbuf);
         status = rpiq_fbufToBitmap(&fbuf,
                                    &screenshotIoctlData->bitmap,
                                    &bitmapLen);
         screenshotIoctlData->bitmap.bitmapLen = bitmapLen;
         rpiq_fbufFree(&fbuf);
         break;
      
      /* Generic mbox transaction */
      default:
         status = rpiq_mboxSend(channel, (rpiq_MboxBuffer_t *)ioctlData);
   }

   if (status != VMK_OK) {
      vmk_Warning(pimon_Driver->logger,
                  "unable to execute ioctl command %d: %s",
                  cmd,
                  vmk_StatusToString(status));
      goto mbox_send_failed;
   }

   return VMK_OK;

mbox_send_failed:
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
 *    VMK_NOT_SUPPORTED
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
 *    VMK_NOT_SUPPORTED
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
