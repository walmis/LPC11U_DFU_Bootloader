/*----------------------------------------------------------------------------
 *      Name:    MEMORY.C
 *      Purpose: USB Mass Storage Demo
 *      Version: V1.10
 *----------------------------------------------------------------------------
 *      This file is part of the uVision/ARM development tools.
 *      This software may only be used under the terms of a valid, current,
 *      end user licence from KEIL for a compatible version of KEIL software
 *      development tools. Nothing else gives you the right to use it.
 *
 *      Copyright (c) 2005-2007 Keil Software.
 *---------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include "mw_usbd_rom_api.h"
#include "app_usbd_cfg.h"
#include "sbl_iap.h"
#include <LPC11Uxx.h>

extern USBD_API_T* pUsbApi;

extern USBD_HANDLE_T hUsb;




