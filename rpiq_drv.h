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
 * rpiq_drv.h --
 *
 *    Definition for RPIQ device layer interface.
 */

#ifndef RPIQ_DRV_H
#define RPIQ_DRV_H

#include "pimon.h"
#include "pimon_types.h"

#define RPIQ_DEBUG

/***********************************************************************/

#define RPIQ_DMA_HEAP_NAME "rpiqDmaHeap"

#define RPIQ_DMA_COHERENT_ADDR      0xC0000000
#define RPIQ_DMA_MAX_ADDR           ((vmk_VA)1<<32)
#define RPIQ_DMA_MBOX_ALIGNMENT     16

/* Maximum mbox objects to allocate on DMA heap */
#define RPIQ_DMA_MBOX_OBJ_MAX       256

#define RPIQ_MBOX_CHAN_MASK         15
#define RPIQ_MBOX_MAX_RETRIES       0x1000

/***********************************************************************/

/*
 * Offsets from RPIQ MMIO base
 */
#define RPIQ_MBOX_READ              0x0
#define RPIQ_MBOX_STATUS            0x18
#define RPIQ_MBOX_WRITE             0x20

#define RPIQ_PROCESS_REQ            0x0
#define RPIQ_MBOX_FULL              (1 << 31)
#define RPIQ_MBOX_EMPTY             (1 << 30)
#define RPIQ_MBOX_SUCCESS           0x80000000

/*
 * Mbox tags
 */

/* Firmware */
#define RPIQ_MBOX_TAG_FWREV          0x00000001

/* Hardware */
#define RPIQ_MBOX_TAG_BOARDMODEL     0x00010001
#define RPIQ_MBOX_TAG_BOARDREV       0x00010002
#define RPIQ_MBOX_TAG_BOARDMAC       0x00010003
#define RPIQ_MBOX_TAG_BOARDSERIAL    0x00010004
#define RPIQ_MBOX_TAG_ARMMEM         0x00010005
#define RPIQ_MBOX_TAG_VCMEM          0x00010006
#define RPIQ_MBOX_TAG_CLKS           0x00010007

/* Config */
#define RPIQ_MBOX_TAG_CMDLINE        0x00050001

/* Shared resources */
#define RPIQ_MBOX_TAG_DMACHAN        0x00060001

/* Power */
#define RPIQ_MBOX_TAG_GETPWR         0x00020001
#define RPIQ_MBOX_TAG_TIMING         0x00020002
#define RPIQ_MBOX_TAG_SETPWR         0x00028001

/* Clocks */
#define RPIQ_MBOX_TAG_GETCLK_STATE   0x00030001
#define RPIQ_MBOX_TAG_SETCLK_STATE   0x00038001
#define RPIQ_MBOX_TAG_GETCLK_RATE    0x00030002
#define RPIQ_MBOX_TAG_SETCLK_RATE    0x00038002
#define RPIQ_MBOX_TAG_GETCLK_MAX     0x00030004
#define RPIQ_MBOX_TAG_GETCLK_MIN     0x00038007
#define RPIQ_MBOX_TAG_GETCLK_TURBO   0x00030009
#define RPIQ_MBOX_TAG_SETCLK_TURBO   0x00038009

// TODO: the resto fot he mbox tags...

#define RPIQ_MBOX_TAG_GET_TEMP       0x00030006

#define RPIQ_INVALID_RESPONSE (~((vmk_uint32)0))

/***********************************************************************/

#define RPIQ_DMA_LOCK_NAME "dmaBufLock"

#define RPIQ_DMA_MEM_BARRIER() asm volatile ("dsb sy" ::: "memory")

/*
 * Should be sufficient as per p. 33 in:
 * https://web.wpi.edu/Pubs/ETD/Available/etd-012017-170924/unrestricted/green_archive.pdf
 */
#define RPIQ_DMA_FLUSH_DCACHE(_va)                                             \
   asm volatile ("dc ivac, %0" :: "r" (_va))

#define RPIQ_DMA_CLEAN_DCACHE(_va)                                             \
   asm volatile ("dc civac, %0" :: "r" (_va))

/***********************************************************************/

typedef struct rpiq_Device_t {
   /* Object */
   vmk_Bool initialized;
   vmk_atomic64 refCount;
   /* Device */
   vmk_Device vmkDevice;
   vmk_ACPIDevice acpiDevice;
   vmk_ACPIInfo acpiInfo;
   /* MMIO */
   vmk_MappedResourceAddress mmioMappedAddr;
   char *mmioBase;
   vmk_ByteCount mmioLen;
   /* DMA */
   vmk_HeapID dmaHeapID;
} rpiq_Device_t;

/*
 * Data structures for passing data between UW and RPIQ interface.
 */

typedef struct rpiq_MboxHeader_t {
   vmk_uint32 bufLen;
   vmk_uint32 requestResponse;
   vmk_uint32 tag;
   vmk_uint32 responseLen;
   vmk_uint32 requestLen;
} rpiq_MboxHeader_t;

typedef struct rpiq_MboxBuffer_t {
   rpiq_MboxHeader_t header;
   /* Everything below be set to zero! */
   vmk_uint32 endTag;
   vmk_uint32 padding[2];
} rpiq_MboxBuffer_t, rpiq_IoctlData_t;

/*
 * RPIQ mailbox channels which selected using the ioctl cmd.
 */
typedef enum rpiq_MboxChannel_t {
   /* Update the min value when adding elements */
   RPIQ_CHAN_MBOX_MIN            = 8,
   RPIQ_CHAN_MBOX_PROP_ARM2VC    = 8,
   RPIQ_CHAN_MBOX_MAX            = RPIQ_CHAN_MBOX_PROP_ARM2VC,
} rpiq_MboxChannel_t, rpiq_IoctlCommand_t;

typedef struct rpiq_MboxDMABuffer_t {
   rpiq_MboxBuffer_t *ptr;
   vmk_Lock lock;
} rpiq_MboxDMABuffer_t;

/***********************************************************************/

VMK_ReturnStatus rpiq_drvInit(pimon_Driver_t *driver,
                              rpiq_Device_t *adapter);

void rpiq_drvCleanUp();

VMK_ReturnStatus rpiq_mboxRead(rpiq_MboxChannel_t channel,
                               vmk_uint32 *response);

VMK_ReturnStatus rpiq_mboxWrite(rpiq_MboxChannel_t channel,
                                rpiq_MboxBuffer_t *buffer);

VMK_ReturnStatus rpiq_mmioOpenCB(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus rpiq_mmioCloseCB(vmk_CharDevFdAttr *attr);

VMK_ReturnStatus rpiq_mmioIoctlCB(unsigned int cmd,
                                  void *data,
                                  vmk_ByteCount ioctlDataLen);

VMK_ReturnStatus rpiq_mmioReadCB(char *buffer,
                                 vmk_ByteCount nbytes,
                                 vmk_loff_t *ppos,
                                 vmk_ByteCountSigned *nread);

VMK_ReturnStatus rpiq_mmioWriteCB(char *buffer,
                                  vmk_ByteCount nbytes,
                                  vmk_loff_t *ppos,
                                  vmk_ByteCountSigned *nwritten);

/***********************************************************************/

#endif /* RPIQ_DRV_H */