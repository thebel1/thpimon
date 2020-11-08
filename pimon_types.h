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
 * pimon_types.h --
 *
 *    Pimon module/driver types.
 */

#ifndef PIMON_TYPES_H
#define PIMON_TYPES_H

/***********************************************************************/

typedef struct pimon_Driver_t {
   vmk_Name driverName;
   vmk_ModuleID moduleID;
   vmk_HeapID heapID;
   vmk_Driver driverHandle;
   vmk_IOResource resHandle;
   vmk_LogComponent logger;
} pimon_Driver_t;

/***********************************************************************/

typedef struct pimon_IoctlHeader_t {
   vmk_uint32 bufLen;
} pimon_IoctlHeader_t;

/*
 * Callbacks for gluing the char dev driver to the RPIQ driver
 */
typedef VMK_ReturnStatus (*pimon_CharDevOpenCB_t)(vmk_CharDevFdAttr *attr);
typedef VMK_ReturnStatus (*pimon_CharDevCloseCB_t)(vmk_CharDevFdAttr *attr);

typedef VMK_ReturnStatus (*pimon_CharDevIoctlCB_t)(unsigned int cmd,
                                                   pimon_IoctlHeader_t *buffer);
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
   vmk_Device logicalDev;
   pimon_CharDevCallbacks_t *callbacks;
   vmk_ByteCount ioctlDataLen;
} pimon_CharDevPriv_t;

typedef struct pimon_CharDev_t {
   vmk_Device vmkDevice;
   vmk_CharDevRegData regData;
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
} pimon_CharDevProps_t;

/***********************************************************************/

/*
 * The RPIQ adapter
 */
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
 * RPIQ mailbox commands. If <16, then it's the VC channel number, if >=16, it's
 * a custom command.
 */
typedef enum rpiq_MboxChannel_t {
   /* Update the min and max values when adding elements */
   RPIQ_IOCTL_CMD_MIN            = 8,
   RPIQ_CHAN_MBOX_PROP_ARM2VC    = 8,
   RPIQ_CMD_PRINT_SCRN           = 24,
   RPIQ_IOCTL_CMD_MAX            = RPIQ_CMD_PRINT_SCRN,
} rpiq_MboxChannel_t, rpiq_IoctlCommand_t;

#pragma pack(push, 1)

/*
 * VideoCore mailbox types
 */

typedef struct rpiq_MboxHeader_t {
   vmk_uint32 bufLen;
   vmk_uint32 requestResponse;
} rpiq_MboxHeader_t;

typedef struct rpiq_MboxBuffer_t {
   rpiq_MboxHeader_t header;
   vmk_uint32 tag;
   vmk_uint32 tagLen;
   vmk_uint32 response[2];
   vmk_uint32 endTag;
   vmk_uint32 padding[2];
} rpiq_MboxBuffer_t, rpiq_IoctlData_t;

typedef struct rpiq_MboxDMABuffer_t {
   rpiq_MboxBuffer_t *ptr;
   vmk_Lock lock;
} rpiq_MboxDMABuffer_t;

/*
 * Bitmap types
 */

typedef struct rpiq_BitmapHeader_t {
   char        charB;
   char        charM;
   vmk_uint32  len;
   vmk_uint16  reserved[2];
   vmk_uint32  imageOffset;
   vmk_uint32  headerLen;
   vmk_uint32  width;
   vmk_uint32  height;
   vmk_uint16  planes;
   vmk_uint16  bitsPerPixel;
   vmk_uint32  compressionType;
   vmk_uint32  imageLen;
   vmk_uint32  xPixelsPerMeter;
   vmk_uint32  yPixelsPerMeter;
   vmk_uint32  numColors;
   vmk_uint32  importantColors;
} rpiq_BitmapHeader_t;

typedef struct rpiq_Bitmap_t {
   vmk_ByteCount bitmapLen;
   char bitmap[1];
} rpiq_Bitmap_t;

/*
 * Frame buffer types
 */

typedef struct rpiq_ScreenshotIoctlData_t {
   pimon_IoctlHeader_t  header;
   rpiq_Bitmap_t        bitmap;
} rpiq_ScreenshotIoctlData_t;

typedef struct rpiq_FrameBuffer_t {
   vmk_uint32     width;
   vmk_uint32     height;
   vmk_uint32     depth;
   char           *raw;
   vmk_ByteCount  rawLen;
} rpiq_FrameBuffer_t;

typedef struct rpiq_FbufMboxBuffer_t {
   rpiq_MboxHeader_t header;
   vmk_uint32 physSizeTag;
   vmk_uint32 physSizeTagSize;
   vmk_uint32 physSizeTagValueSize;
   vmk_uint32 physSizeWidth;
   vmk_uint32 physSizeHeight;
   vmk_uint32 virtSizeTag;
   vmk_uint32 virtSizeTagSize;
   vmk_uint32 virtSizeTagvalueSize;
   vmk_uint32 virtSizeWidth;
   vmk_uint32 virtSizeHeight;
   vmk_uint32 depthTag;
   vmk_uint32 depthTagSize;
   vmk_uint32 depthTagValueSize;
   vmk_uint32 depth;
   vmk_uint32 allocFbufTag;
   vmk_uint32 allocFbufTagSize;
   vmk_uint32 allocFbufTagValueSize;
   vmk_uint32 allocFbufAlignBase;
   vmk_uint32 allocFbufBase;
   vmk_uint32 allocFbufSize;
   vmk_uint32 pitchTag;
   vmk_uint32 pitchTagSize;
   vmk_uint32 pitchTagValueSize;
   vmk_uint32 endTag;
} rpiq_FbufMboxBuffer_t;

typedef struct rpiq_FrameBufferPixel_t {
   vmk_uint8   blue;
   vmk_uint8   green;
   vmk_uint8   red;
   vmk_uint8   reserved;
} rpiq_FrameBufferPixel_t;

#pragma pack(pop)

/***********************************************************************/

#endif /* PIMON_TYPES_H */