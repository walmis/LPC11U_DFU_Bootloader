/***********************************************************************
 * $Id:: main.c 211 2012-01-17 21:34:04Z usb06052                      $
 *
 * Project: USB_MSC_HID_DFU
 *
 * Description:
 *     A USB_MSC_HID_DFU example.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/
#include <string.h>
#include "LPC11Uxx.h"
#include "mw_usbd_rom_api.h"
#include <sbl_config.h>
#include <power_api.h>
#include <uart.h>
#include <sbl_iap.h>
#include <stdlib.h>
#include <stdint.h>

extern ErrorCode_t usb_dfu_init(USBD_HANDLE_T hUsb,
		USB_INTERFACE_DESCRIPTOR* pIntfDesc, uint32_t* mem_base,
		uint32_t* mem_size);

//extern ErrorCode_t USB_Configure_Event (USBD_HANDLE_T hUsb);
extern uint8_t USB_DeviceDescriptor[];
extern uint8_t USB_StringDescriptor[];
extern uint8_t USB_FsConfigDescriptor[];

void reset_system();
void process_dfu();
void delay(uint32_t t);

char* ltoa( long value, char *string, int radix )
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v;
  int sign;
  char *sp;

  if ( string == NULL )
  {
    return 0 ;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0 ;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
  {
    v = -value;
  }
  else
  {
    v = (unsigned long)value;
  }

  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}

void putHex(int i) {
	char str[32];
	ltoa(i, str, 16);
	UARTSend(str, strlen(str));
}
void putHexByte(char i) {
	char str[32];
	ltoa(i, str, 16);
	if(str[1] == '\0') {
		str[2] = '\0';
		str[1] = str[0];
		str[0] = '0';
	}
	UARTSend(str, strlen(str));
}
void putDec(int i) {
	char str[32];
	ltoa(i, str, 10);
	UARTSend(str, strlen(str));
}

void print(char* str) {
	UARTSend(str, strlen(str));
}

void printbuf(char* buf, int len) {
	int i = 0;
	while(len--) {
		putHexByte(*buf++); print(" ");
		i++;
		if(i >= 16) { print("\n"); i = 0; }
	}
	print("\n");
}

typedef struct {
    unsigned int word0; //Word 0 of 128-bit signature (bits 31 to 0).
    unsigned int word1; //Word 1 of 128-bit signature (bits 63 to 32).
    unsigned int word2; //Word 2 of 128-bit signature (bits 95 to 64).
    unsigned int word3; //Word 3 of 128-bit signature (bits 127 to 96).
} FLASH_SIG_Type;

void softSig(unsigned int startAddr, unsigned int length, FLASH_SIG_Type *pSig)
{
    FLASH_SIG_Type flashWord;
    FLASH_SIG_Type refSignature = {0, 0, 0, 0};
    FLASH_SIG_Type nextSign;
    uint32_t* PageAddr = (uint32_t*)(startAddr);

    if(length > DFU_MAX_IMAGE_LEN) {
    	pSig->word0 = 0;
    	pSig->word1 = 0;
    	pSig->word2 = 0;
    	pSig->word3 = 0;
    	return;
    }

   for (unsigned int i = 0; i < (length >> 4); i++) {
        flashWord.word0 = *PageAddr;
        PageAddr++;
        flashWord.word1 = *PageAddr;
        PageAddr++;
        flashWord.word2 = *PageAddr;
        PageAddr++;
        flashWord.word3 = *PageAddr;
        PageAddr++;

        //Update 128 bit signature
        nextSign.word0 = flashWord.word0 ^ refSignature.word0 >> 1 ^ refSignature.word1 << 31;
        nextSign.word1 = flashWord.word1 ^ refSignature.word1 >> 1 ^ refSignature.word2 << 31;
        nextSign.word2 = flashWord.word2 ^ refSignature.word2 >> 1 ^ refSignature.word3 << 31;
        nextSign.word3 = flashWord.word3 ^ refSignature.word3 >> 1 ^
                         (refSignature.word0 & 1 << 29) << 2 ^
                         (refSignature.word0 & 1 << 27) << 4 ^
                         (refSignature.word0 & 1 << 2) << 29 ^
                         (refSignature.word0 & 1 << 0) << 31;

        //Point to the calculated value
        refSignature.word0 = nextSign.word0;
        refSignature.word1 = nextSign.word1;
        refSignature.word2 = nextSign.word2;
        refSignature.word3 = nextSign.word3;
    }

    //Copy the reference signature to the result pointer
    pSig->word0 = refSignature.word0;
    pSig->word1 = refSignature.word1;
    pSig->word2 = refSignature.word2;
    pSig->word3 = refSignature.word3;

}

int check_signature() {
	FLASH_SIG_Type sig;
	int len  = *((uint32_t*)(0x10D0));
	uint32_t* signature = ((uint32_t*)(0x10C0));
	softSig(0x10E0, len, &sig);

//	putDec(len); print("\n");
//	print("Signature ");
//	putHex(sig.word0); print(" ");
//	putHex(sig.word1); print(" ");
//	putHex(sig.word2); print(" ");
//	putHex(sig.word3); print("\n");

	int res = sig.word0 == signature[0] && sig.word1 == signature[1] && sig.word2 == signature[2] && sig.word3 == signature[3];
	//if(!res) print("Verification failed\n");
	return res;
}

ErrorCode_t USB_Configure_Event(USBD_HANDLE_T hUsb) {
	return LPC_OK;
}

__attribute__ ((aligned(4))) const uint8_t USB_dfuConfigDescriptor[] = {
/* Configuration 1 */
USB_CONFIGUARTION_DESC_SIZE, /* bLength */
USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
WBVAL( /* wTotalLength */
1*USB_CONFIGUARTION_DESC_SIZE + 1*USB_INTERFACE_DESC_SIZE + DFU_FUNC_DESC_SIZE),
		0x01, /* bNumInterfaces */
		0x03, /* bConfigurationValue */
		0x00, /* iConfiguration */
		USB_CONFIG_SELF_POWERED /*|*//* bmAttributes */
		/*USB_CONFIG_REMOTE_WAKEUP*/, USB_CONFIG_POWER_MA(100), /* bMaxPower */
		/* Interface 0, Alternate Setting 0, DFU Class */
		USB_INTERFACE_DESC_SIZE, /* bLength */
		USB_INTERFACE_DESCRIPTOR_TYPE, /* bDescriptorType */
		0x00, /* bInterfaceNumber */
		0x00, /* bAlternateSetting */
		0x00, /* bNumEndpoints */
		USB_DEVICE_CLASS_APP, /* bInterfaceClass */
		USB_DFU_SUBCLASS, /* bInterfaceSubClass */
		0x02, /* bInterfaceProtocol */
		0x00, /* iInterface */
		/* DFU RunTime/DFU Mode Functional Descriptor */
		DFU_FUNC_DESC_SIZE, /* bLength */
		USB_DFU_DESCRIPTOR_TYPE, /* bDescriptorType */
		USB_DFU_CAN_DOWNLOAD | USB_DFU_CAN_UPLOAD | USB_DFU_MANIFEST_TOL, /* bmAttributes */
		WBVAL(0xFF00), /* wDetachTimeout */
		WBVAL(USB_DFU_XFER_SIZE), /* wTransferSize */
		WBVAL(0x0110), /* bcdDFUVersion */
		/* Terminator */
		0 /* bLength */
};

/**********************************************************************
 ** Global data
 **********************************************************************/
volatile uint32_t u32Milliseconds = 0;

USBD_API_T* pUsbApi;

/* local data */
USBD_HANDLE_T hUsb;

struct {
	volatile uint32_t block_num;
	volatile uint32_t packet_len;
	uint8_t buf[64];

	volatile uint8_t complete;
	volatile uint8_t dfu_status;
	volatile uint8_t dfu_state;
	volatile uint8_t data_pending;
	volatile uint8_t status_req;
} dfu;

void USB_IRQHandler(void) {
	uint32_t *addr = (uint32_t *) LPC_USB->EPLISTSTART;
	/*	WORKAROUND for artf32289 ROM driver BUG:
	    As part of USB specification the device should respond
	    with STALL condition for any unsupported setup packet. The host will send
	    new setup packet/request on seeing STALL condition for EP0 instead of sending
	    a clear STALL request. Current driver in ROM doesn't clear the STALL
	    condition on new setup packet which should be fixed.
	 */
	if ( LPC_USB->DEVCMDSTAT & (1<<8) ) {	/* if setup packet is received */
		addr[0] &= ~(1<<29);	/* clear EP0_OUT stall */
		addr[2] &= ~(1<<29) | (1<<31);	/* clear EP0_IN stall and ACTIVE bit*/
	}
	pUsbApi->hw->ISR(hUsb);
}

void usbclk_init() {

	LPC_SYSCON->PDRUNCFG     &= ~(1 << 8); //power up usb pll

	LPC_SYSCON->USBPLLCLKSEL = 1; //switch to external oscillator

	LPC_SYSCON->USBPLLCLKUEN = 1;
	LPC_SYSCON->USBPLLCLKUEN = 0;
	LPC_SYSCON->USBPLLCLKUEN = 1;
	while (!(LPC_SYSCON->USBPLLCLKUEN & 0x01)); //wait until updated

	LPC_SYSCON->USBPLLCTRL = 0x22; //input clk 16MHZ, out 48MHZ

	while (!(LPC_SYSCON->USBPLLSTAT & 0x01)); //wait until PLL locked
}

void usb_connect(uint8_t state) {
	LPC_GPIO->B0[21] = state;
}

void USB_pin_clk_init(void) {
	/* Enable AHB clock to the GPIO domain. */
	/* Enable AHB clock to the USB block and USB RAM. */
	usbclk_init();

	SysTick->LOAD = (SystemCoreClock / 1000) - 1;
	SysTick->CTRL =
			SysTick_CTRL_CLKSOURCE_Msk |
			SysTick_CTRL_ENABLE_Msk |
			SysTick_CTRL_TICKINT_Msk;
	SysTick->VAL = 0x00FFFFFF;

	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_SetPriority(USB_IRQn, 2);

	LPC_SYSCON->PDRUNCFG &= ~(1<<10); //Power up USBPAD
	LPC_SYSCON->SYSAHBCLKCTRL |= ((0x1 << 14) | (0x1 << 27));
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6);

	/* Pull-down is needed, or internally, VBUS will be floating. This is to
	 address the wrong status in VBUSDebouncing bit in CmdStatus register (applies to Rev A). */
	//LPC_IOCON->PIO0_3   &= ~0x1F;
//  LPC_IOCON->PIO0_3   |= ((0x1<<3)|(0x01<<0));	/* Secondary function VBUS */
	//LPC_IOCON->PIO0_3   |= (0x01<<0);			/* Secondary function VBUS */
	//LPC_IOCON->PIO0_6   &= ~0x07;
	//LPC_IOCON->PIO0_6   |= (0x01<<0);			/* Secondary function SoftConn */
	LPC_GPIO->DIR[0] |= (1 << 21);
	LPC_GPIO->B0[21] = 0;

	return;
}

/* In the M0, there is no VTOR. In the LPC range such as the LPC11U,
 * whilst the vector table may only be something like 48 entries (192 bytes, 0xC0),
 * the SYSMEMREMAP register actually remaps the memory from 0x10000000-0x100001FF
 * to adress 0x0-0x1FF. In this case, RAM can be addressed at both 0x10000000 and 0x0
 *
 * If we just copy the vectors to RAM and switch the SYSMEMMAP, any accesses to FLASH
 * above the vector table before 0x200 will actually go to RAM. So we need to provide
 * a solution where the compiler gets the right results based on the memory map
 *
 * Option 1 - We allocate and copy 0x200 of RAM rather than just the table
 *  - const data and instructions before 0x200 will be copied to and fetched/exec from RAM
 *  - RAM overhead: 0x200 - 0xC0 = 320 bytes, FLASH overhead: 0
 *
 * Option 2 - We pad the flash to 0x200 to ensure the compiler doesn't allocate anything there
 *  - No flash accesses will go to ram, as there will be nothing there
 *  - RAM only needs to be allocated for the vectors, as all other ram addresses are normal
 *  - RAM overhead: 0, FLASH overhead: 320 bytes
 *
 * Option 2 is the one to go for, as RAM is the most valuable resource
 */
void app_exec() {
	//memcpy((void*)0x10000000, (void*)0x1000, 0xC0); //copy app vector table to sram

	uint32_t * const ram_base = (uint32_t *)0x10000000;
	const uint32_t* flash = (uint32_t *)0x1000;
	uint32_t i;
	for(i = 0; i < 48; i++) {
		ram_base[i] = flash[i];
	}

	LPC_SYSCON->SYSMEMREMAP = 0x01; //set vectors at sram

	asm volatile("ldr r0, =0x10000000");
	asm volatile("ldr r0, [r0]");
	asm volatile("mov sp, r0");

	/* Load program counter with application reset vector address, located at
	   second word of application area. */
	asm volatile("ldr r0, =0x10000004");
	asm volatile("ldr r0, [r0]");
	asm volatile("mov pc, r0");

	//printf("%x %x\n", sp, pc);
//	__asm volatile ("LDR R3, [%[value]]\n"
//			"MOV R0, R3\n"
//	        "LDR R3, [%[value], #4]\n"
//	        "MOV R1, R3\n"
//			"B p\n": : [value] "r" (address));
}

void enable_wdt(uint32_t timeout_us) {
	  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<15);

	  LPC_SYSCON->WDTOSCCTRL = 0x03F; /* ~8kHz */
	  LPC_SYSCON->PDRUNCFG &= ~(0x1<<6);

	  LPC_WWDT->CLKSEL = 0x01 | (1<<31);		/* Select watchdog osc */

	  //125us is one clock at 8khz
	  LPC_WWDT->TC = timeout_us / 128;	/* once WDEN is set, the WDT will start after feeding */


	  LPC_WWDT->MOD = (1<<0) | (1<<1) | (1<<5); //WDEN | WDRESET | WDLOCK

	  LPC_WWDT->FEED = 0xAA;		/* Feeding sequence */
	  LPC_WWDT->FEED = 0x55;
}

void __default_exit() {
	NVIC_SystemReset();
}

void __early_init() {
	LPC_GPIO->DIR[0] |= (1 << 21);
	LPC_GPIO->B0[21] = 0; //pull usb-connect LOW

	LPC_PMU->GPREG3++;

	if(LPC_PMU->GPREG3 > 4) {
		LPC_PMU->GPREG3 = 0;
		return;
	}

	if(check_signature()) {
		enable_wdt(1000000);
		app_exec();
	}
}

void __late_init() {
	SystemInit();
	__enable_irq();
}


ErrorCode_t dfu_ep0(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	USB_CORE_CTRL_T* pCtrl = (USB_CORE_CTRL_T*)hUsb;
	//USBD_DFU_CTRL_T* p = (USBD_DFU_CTRL_T*)data;

	int error = 0;

//	printf("ev %d bmRequestType:%x bRequest:%x\n", event, pCtrl->SetupPacket.bmRequestType.B,
//			pCtrl->SetupPacket.bRequest);

	if(event == USB_EVT_RESET) {
		if(dfu.complete) {
			usb_connect(0);
			//app_exec(0x1000);
			//print("sys reset\n");

			reset_system();
		} else {
			//print("reset\n");
		}
	}

	if(event == USB_EVT_OUT) {
		if(pCtrl->SetupPacket.bRequest == USB_REQ_DFU_DNLOAD) {
			dfu.block_num = pCtrl->SetupPacket.wValue.W;
			dfu.packet_len = pCtrl->SetupPacket.wLength;
			memcpy(dfu.buf, pCtrl->EP0Buf, dfu.packet_len);

			//delay(10);

			dfu.data_pending = 1;
			//print("out "); putDec(pCtrl->SetupPacket.wValue.W); print("\n");
	        //pUsbApi->core->StatusInStage(hUsb);
			return LPC_OK;
		}
	}
	if(event == USB_EVT_IN) {
		//print("in "); putDec(pCtrl->SetupPacket.wLength); print("\n");
		if(pCtrl->SetupPacket.bRequest == USB_REQ_DFU_GETSTATUS) {
			dfu.status_req = 1;
		}

	}

	if(event == USB_EVT_SETUP && (pCtrl->SetupPacket.bmRequestType.B & 0x21)) {
		//if(p)
		//printf("%d\n", p->dfu_state);
		switch(pCtrl->SetupPacket.bRequest) {
		case USB_REQ_DFU_DETACH:
			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			pCtrl->EP0Data.Count = 0;
			//pUsbApi->core->DataInStage(hUsb);
			return LPC_OK;

		case USB_REQ_DFU_GETSTATUS:{
			DFU_STATUS_T dfu_req_get_status;
			//print("st\n");
			//putDec(__get_IPSR()); print("\n");

			//int a = u32Milliseconds;
	        process_dfu();
			//putDec(u32Milliseconds-a); print("\n");

			dfu_req_get_status.bwPollTimeout[0] = 8;
			dfu_req_get_status.bwPollTimeout[1] = 0;
			dfu_req_get_status.bwPollTimeout[2] = 0;

			dfu_req_get_status.bState = dfu.dfu_state;
			dfu_req_get_status.bStatus = dfu.dfu_status;
			dfu_req_get_status.iString = 0;

			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			memcpy(pCtrl->EP0Data.pData, &dfu_req_get_status, sizeof(dfu_req_get_status));
	        pCtrl->EP0Data.Count = sizeof(dfu_req_get_status);

	        //process_dfu();

	        pUsbApi->core->DataInStage(hUsb);



		}
	        return LPC_OK;

		break;
		case USB_REQ_DFU_GETSTATE:
			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			pCtrl->EP0Data.pData[0] = dfu.dfu_state;
	        pCtrl->EP0Data.Count = sizeof(dfu.dfu_state);
	        pUsbApi->core->DataInStage(hUsb);
	        return LPC_OK;

		case USB_REQ_DFU_UPLOAD:
		{
			dfu.block_num = pCtrl->SetupPacket.wValue.W;

			uint8_t* src_addr = (uint8_t*)DFU_DEST_BASE + (dfu.block_num * USB_DFU_XFER_SIZE);
		    //*pBuff = (uint8_t*)src_addr;
		    pCtrl->EP0Data.pData = pCtrl->EP0Buf;
		    if (dfu.block_num == DFU_MAX_BLOCKS) {
		    	pCtrl->SetupPacket.wLength = 0;
		    }

		    if (dfu.block_num > DFU_MAX_BLOCKS) {
		      pCtrl->SetupPacket.wLength = 0;
		      dfu.dfu_state = DFU_STATUS_errADDRESS;
		    }

		    pCtrl->EP0Data.pData = pCtrl->EP0Buf;
		    memcpy(pCtrl->EP0Data.pData, (void*)src_addr, pCtrl->SetupPacket.wLength);
		    pCtrl->EP0Data.Count = pCtrl->SetupPacket.wLength;
		    pUsbApi->core->DataInStage(hUsb);


			return LPC_OK;
		}
		case USB_REQ_DFU_DNLOAD:

			//print("dload "); putDec(pCtrl->SetupPacket.wLength); print("\n");
			if(pCtrl->SetupPacket.wLength == 0 && dfu.block_num != 0) {
				dfu.packet_len = 0;
				dfu.data_pending = 1;
				dfu.status_req = 1;

				dfu.dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
				process_dfu();
			} else {
				dfu.dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			}

			pCtrl->EP0Data.Count = pCtrl->SetupPacket.wLength;
			pCtrl->EP0Data.pData = pCtrl->EP0Buf;


	        //pUsbApi->core->DataOutStage(hUsb);
			return LPC_OK;
		break;

		case USB_REQ_DFU_CLRSTATUS:
		case USB_REQ_DFU_ABORT:
			//print("USB_REQ_DFU_ABORT\n");
			dfu.dfu_state = DFU_STATE_dfuIDLE;
			dfu.dfu_status = 0;
			dfu.packet_len = 0;
			dfu.block_num = 0;
			dfu.complete = 0;

			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			pCtrl->EP0Data.Count = 0;
	        pUsbApi->core->DataInStage(hUsb);
	        return LPC_OK;

		default:
			//print("unk req "); putDec(pCtrl->SetupPacket.bRequest); print("\n");
			break;


		}
	}
	//UARTSend("Un", 2);
	return ERR_USBD_UNHANDLED;
}

void uart_init() {
	  LPC_IOCON->PIO0_18 &= ~0x07;    /*  UART I/O config */
	  LPC_IOCON->PIO0_18 |= 0x01;     /* UART RXD */
	  LPC_IOCON->PIO0_19 &= ~0x07;
	  LPC_IOCON->PIO0_19 |= 0x01;     /* UART TXD */

	  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
	  LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

	  LPC_USART->LCR = 0x83;            /* 8 bits, no Parity, 1 Stop bit */

	  uint32_t Fdiv = ((48000000/1)/16)/115200 ;	/*baud rate */
	  LPC_USART->DLM = Fdiv / 256;
	  LPC_USART->DLL = Fdiv % 256;
	  LPC_USART->FDR = 0x10;		/* Default */

	  LPC_USART->LCR = 0x03;		/* DLAB = 0 */
	  LPC_USART->FCR = 0x07;		/* Enable and reset TX and RX FIFO. */

	  /* Read to clear the line status. */
	  int regVal = LPC_USART->LSR;

	  /* Ensure a clean start, no data in either TX or RX FIFO. */
	  while (( LPC_USART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
	  while ( LPC_USART->LSR & LSR_RDR )
	  {
		regVal = LPC_USART->RBR;	/* Dump data from RX FIFO */
	  }
}

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void) //__attribute__((naked))
{
  //asm volatile("  TST LR, #4; ");
  register long lr asm("lr");
  if(lr & 4) {
	  asm("mrs r0, msp");
  } else {
	  asm("mrs r0, psp");
  }
  asm("mov sp, r0");
  asm("bkpt");
}

void process_dfu() {
	if(dfu.dfu_state == DFU_STATE_dfuDNLOAD_IDLE && dfu.complete) {
		dfu.dfu_state = DFU_STATE_dfuIDLE;
	}

	if(dfu.dfu_state == DFU_STATE_dfuDNLOAD_SYNC && dfu.data_pending && dfu.status_req) {
		uint32_t b = dfu.block_num;
		uint32_t p = dfu.packet_len;
		uint32_t dest_addr = DFU_DEST_BASE + (b * USB_DFU_XFER_SIZE);

		//dfu.dfu_state = DFU_STATE_dfuDNBUSY;
		//print("wr b:");
//		putDec(b); print(" ");
//		putDec(p); print(" ");
		//putDec(dest_addr);
		//print("\n");
		//printbuf(dfu.buf, 64);

		if(dfu.block_num == 0) {
			print("Boot> Erasing...\n");
			prepare_sector_usb(FIRST_USER_SECTOR,MAX_USER_SECTOR-1, CCLK);
			erase_sector_usb(FIRST_USER_SECTOR, MAX_USER_SECTOR-1, CCLK);
		}
		if(dfu.block_num == 1) {
			print("Boot> Flashing...\n");
		}
		dfu.data_pending = 0;

		//while(!dfu.status_req);
		dfu.status_req = 0;
		//delay(5);

		if (dfu.packet_len != 0) {
			if (dfu.block_num >= DFU_MAX_BLOCKS)
				dfu.dfu_status = DFU_STATUS_errADDRESS;

			//printf("wr %d %d\n", dest_addr, length);
			//dump( (char*)&((*pBuff)[0]), length);
			uint8_t last = (dfu.packet_len < USB_DFU_XFER_SIZE) ? 1 : 0;
			if(write_flash((unsigned*) dest_addr, dfu.buf, dfu.packet_len,	last) == 1) {
				dfu.dfu_status = DFU_STATUS_errPROG;
			}
			//delay(40);


			if(last) {
				dfu.complete = 1;

			}
		}


		if(dfu.packet_len == 0) {
			if(!dfu.complete) {
				//force write last block
				if(write_flash((unsigned*) dest_addr, dfu.buf, 0, 1) == 1) {
					dfu.dfu_status = DFU_STATUS_errPROG;
				}
				dfu.complete = 1;
			}


		} else {
			dfu.dfu_state = DFU_STATE_dfuDNLOAD_IDLE;
		}
		//reset boot counter
		if(dfu.complete){
			if(!check_signature()) {
				dfu.dfu_status = DFU_STATUS_errVERIFY;
				print("Boot> Verification failed\n");
			}
			dfu.dfu_state = DFU_STATE_dfuDNLOAD_IDLE;
			LPC_PMU->GPREG3 = 0;
		}
	}
}


void loop() {
	while (1) {

		if((u32Milliseconds & 511) < 256) {
			LPC_GPIO->B0[1] = 1;
		} else
		if((u32Milliseconds & 511) > 256) {
			LPC_GPIO->B0[1] = 0;
		}

		//process_dfu();

		if(u32Milliseconds > 10000 && dfu.dfu_state == DFU_STATE_dfuIDLE) {
			if(user_code_present()) {
				reset_system();
			}
			u32Milliseconds = 0;
		}


	}
}

int main(void) {
	USBD_API_INIT_PARAM_T usb_param;
	USB_CORE_DESCS_T desc;
	ErrorCode_t ret;
	USB_INTERFACE_DESCRIPTOR* pIntfDesc;
	USB_COMMON_DESCRIPTOR *pD;
	uint32_t next_desc_adr, total_len = 0;

	//SystemCoreClockUpdate();

	/* Setup UART for 115.2K, 8 data bits, no parity, 1 stop bit */
	uart_init();

	UARTSend((uint8_t *) "\r\nBoot>\r\n", 9);

	if(!check_signature()) {
		print("Boot> Bad signature\n");
	}

	/* get USB API table pointer */
	pUsbApi = (USBD_API_T*) ((*(ROM **) (0x1FFF1FF8))->pUSBD);

	/* enable clocks and pinmux for usb0 */
	USB_pin_clk_init();

	/* initilize call back structures */
	memset((void*) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB_BASE;
	usb_param.mem_base = 0x10001000;
	usb_param.mem_size = 0x800;
	usb_param.max_num_ep = 2;
	//usb_param.USB_Configure_Event = USB_Configure_Event;

	/* Initialize Descriptor pointers */
	memset((void*) &desc, 0, sizeof(USB_CORE_DESCS_T));
	desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &USB_dfuConfigDescriptor[0];
	desc.high_speed_desc = (uint8_t *) &USB_dfuConfigDescriptor[0];

	/* USB Initialization */
	ret = pUsbApi->hw->Init(&hUsb, &desc, &usb_param);

	/*	WORKAROUND for artf32219 ROM driver BUG:
	    The mem_base parameter part of USB_param structure returned
	    by Init() routine is not accurate causing memory allocation issues for
	    further components.
	 */
	usb_param.mem_base = 0x10001000 + (0x800 - usb_param.mem_size);

	dfu.dfu_state = DFU_STATE_dfuIDLE;

	pUsbApi->core->RegisterClassHandler(hUsb, dfu_ep0, 0);

	init_usb_iap();

	NVIC_EnableIRQ(USB_IRQn); //  enable USB interrrupts

	pUsbApi->hw->Connect(hUsb, 1);

	LPC_GPIO->B0[21] = 1;
	LPC_GPIO->DIR[0] |= (1<<1);

	loop();
}

void reset_system() {
	print("Boot> Reset\n");
	delay(10);
	NVIC_SystemReset();
}


void SysTick_Handler(void) {
	u32Milliseconds++;

}

void delay(uint32_t ms) {
	uint32_t t = u32Milliseconds + ms;
	while(u32Milliseconds < t);
}
/**********************************************************************
 **                            End Of File
 **********************************************************************/
