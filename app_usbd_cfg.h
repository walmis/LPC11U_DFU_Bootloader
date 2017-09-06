/*----------------------------------------------------------------------------
 * U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbcfg.h
 * Purpose: USB Custom Configuration
 * Version: V1.20
 *----------------------------------------------------------------------------
 * This software is supplied "AS IS" without any warranties, express,
 * implied or statutory, including but not limited to the implied
 * warranties of fitness for purpose, satisfactory quality and
 * noninfringement. Keil extends you a royalty-free right to reproduce
 * and distribute executable files created using this software for use
 * on NXP ARM microcontroller devices only. Nothing else gives
 * you the right to use this software.
 *
 * Copyright (c) 2005-2009 Keil Software.
 * Adaption to LPCxxxx, Copyright (c) 2009 NXP.
 *----------------------------------------------------------------------------*/

#ifndef __USBCFG_H__
#define __USBCFG_H__

#define USE_MSC_DESC    0
#define USE_HID_DESC    0
#define USE_DFU_DESC    1
#define USE_COMP_DESC   0

#define MSC_EP_IN   0x81
#define MSC_EP_OUT  0x01
#define HID_EP_IN   0x82
#define HID_EP_OUT  0x02

#define USB_MAX_IF_NUM  8
#define USB_MAX_EP_NUM  5
#define USB_MAX_PACKET0 64
/* Max In/Out Packet Size */
#define USB_FS_MAX_BULK_PACKET  64
/* Full speed device only */
#define USB_HS_MAX_BULK_PACKET  USB_FS_MAX_BULK_PACKET
/* IP3511 is full speed only */
#define FULL_SPEED_ONLY

#define HID_RPRT_DESC_SZ  0x21

#define USB_DFU_XFER_SIZE   64

#define IDVENDOR 0xffff
#define IDPRODUCT 0x12c9

/* DFU boot definitions */
#define DFU_DEST_BASE         0x1000
#define DFU_MAX_IMAGE_LEN     (28 * 1024)
#define DFU_MAX_BLOCKS        (DFU_MAX_IMAGE_LEN/USB_DFU_XFER_SIZE)

#define FLASH_BUF_SIZE 512

#endif  /* __USBCFG_H__ */
