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
#include <power_api.h>
#include <uart.h>
#include <sbl_iap.h>
#include <stdlib.h>

extern ErrorCode_t usb_dfu_init(USBD_HANDLE_T hUsb,
		USB_INTERFACE_DESCRIPTOR* pIntfDesc, uint32_t* mem_base,
		uint32_t* mem_size);

//extern ErrorCode_t USB_Configure_Event (USBD_HANDLE_T hUsb);
extern uint8_t USB_DeviceDescriptor[];
extern uint8_t USB_StringDescriptor[];
extern uint8_t USB_FsConfigDescriptor[];

uint32_t bad_signature = 0;

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
void putDec(int i) {
	char str[32];
	ltoa(i, str, 10);
	UARTSend(str, strlen(str));
}

void print(char* str) {
	UARTSend(str, strlen(str));
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
    unsigned int* PageAddr = (uint32_t*)(startAddr);

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
		WBVAL(0x100), /* bcdDFUVersion */
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

void USB_IRQHandler(void) {
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

	uint32_t * const ram_base = 0x10000000;
	const uint32_t* flash = 0x1000;
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
	} else {
		bad_signature = 1;
	}
}

void __late_init() {
	SystemInit();
	__enable_irq();
}



void dfu_done(void)
{
  return;
}

uint8_t dfu_state = DFU_STATE_dfuIDLE;



ErrorCode_t dfu_ep0(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
	USB_CORE_CTRL_T* pCtrl = (USB_CORE_CTRL_T*)hUsb;
	//USBD_DFU_CTRL_T* p = (USBD_DFU_CTRL_T*)data;

	static uint32_t block_num = 0;
	static uint32_t packet_len = 0;
	static uint8_t buf[64];
	static const uint8_t* pbuf = buf;
	static uint8_t complete;
	static uint8_t dfu_status;
	static DFU_STATUS_T dfu_req_get_status;

	int error = 0;

//	printf("ev %d bmRequestType:%x bRequest:%x\n", event, pCtrl->SetupPacket.bmRequestType.B,
//			pCtrl->SetupPacket.bRequest);

	if(event == USB_EVT_RESET && complete) {
		usb_connect(0);
		//app_exec(0x1000);
		//print("reset\n");

		NVIC_SystemReset();
	}

	if(event == USB_EVT_OUT) {
		if(pCtrl->SetupPacket.bRequest == USB_REQ_DFU_DNLOAD) {
			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			pCtrl->EP0Data.Count = 0;
	        pUsbApi->core->StatusInStage(hUsb);
			return LPC_OK;
		}
	}
	if(event == USB_EVT_SETUP && (pCtrl->SetupPacket.bmRequestType.B & 0x21)) {
		//if(p)
		//printf("%d\n", p->dfu_state);
		switch(pCtrl->SetupPacket.bRequest) {
		case USB_REQ_DFU_DETACH:
			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			pCtrl->EP0Data.Count = 0;
			pUsbApi->core->DataInStage(hUsb);
			return LPC_OK;
		case USB_REQ_DFU_GETSTATUS:
			//printf("getstatus %d\n", p->dfu_state);
			if(dfu_state == DFU_STATE_dfuDNLOAD_SYNC) {

				dfu_req_get_status.bwPollTimeout[0] = 255;
				dfu_req_get_status.bwPollTimeout[1] = 0;

				uint32_t dest_addr = DFU_DEST_BASE;
				dest_addr += (block_num * USB_DFU_XFER_SIZE);
				if (packet_len != 0) {
					if (block_num >= DFU_MAX_BLOCKS)
						dfu_status = DFU_STATUS_errADDRESS;

					//printf("wr %d %d\n", dest_addr, length);
					//dump( (char*)&((*pBuff)[0]), length);
					uint8_t last = (packet_len < USB_DFU_XFER_SIZE) ? 1 : 0;
					if(write_flash((unsigned*) dest_addr, buf, packet_len,	last) == 1) {
						dfu_status = DFU_STATUS_errPROG;
						error = 1;
					}
					if(last) {
						complete = 1;
					}
				}


				if(packet_len == 0) {
					if(!complete) {
						//force write last block
						if(write_flash((unsigned*) dest_addr, buf, 0, 1) == 1) {
							dfu_status = DFU_STATUS_errPROG;
							error = 1;
						}
					}
					dfu_state = DFU_STATE_dfuIDLE;
					complete = 1;

				} else {
					dfu_state = DFU_STATE_dfuDNLOAD_IDLE;
				}
				//reset boot counter
				if(complete){
					if(!check_signature()) {
						dfu_status = DFU_STATUS_errVERIFY;
						print("Boot> Verification failed\n");
					}
					LPC_PMU->GPREG3 = 0;
				}
			}

			dfu_req_get_status.bState = dfu_state;
			dfu_req_get_status.bStatus = dfu_status;
			dfu_req_get_status.iString = 0;
			dfu_req_get_status.bwPollTimeout[0] = 255;

			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			memcpy(pCtrl->EP0Data.pData, &dfu_req_get_status, sizeof(dfu_req_get_status));
	        pCtrl->EP0Data.Count = sizeof(dfu_req_get_status);
	        pUsbApi->core->DataInStage(hUsb);
	        return LPC_OK;

		break;
		case USB_REQ_DFU_GETSTATE:
			pCtrl->EP0Data.pData = pCtrl->EP0Buf;
			pCtrl->EP0Data.pData[0] = &dfu_state;
	        pCtrl->EP0Data.Count = sizeof(dfu_state);
	        pUsbApi->core->DataInStage(hUsb);
	        return LPC_OK;

		case USB_REQ_DFU_UPLOAD:
		{
			block_num = pCtrl->SetupPacket.wValue.W;

			uint8_t* src_addr = DFU_DEST_BASE + (block_num * USB_DFU_XFER_SIZE);
		    //*pBuff = (uint8_t*)src_addr;
		    pCtrl->EP0Data.pData = pCtrl->EP0Buf;
		    if (block_num == DFU_MAX_BLOCKS) {
		    	pCtrl->SetupPacket.wLength = 0;
		    }

		    if (block_num > DFU_MAX_BLOCKS) {
		      pCtrl->SetupPacket.wLength = 0;
		      dfu_state = DFU_STATUS_errADDRESS;
		    }

		    pCtrl->EP0Data.pData = pCtrl->EP0Buf;
		    memcpy(pCtrl->EP0Data.pData, (void*)src_addr, pCtrl->SetupPacket.wLength);
		    pCtrl->EP0Data.Count = pCtrl->SetupPacket.wLength;
		    pUsbApi->core->DataInStage(hUsb);

			return LPC_OK;
		}
		case USB_REQ_DFU_DNLOAD:
			//print("dload "); putDec(pCtrl->SetupPacket.wLength); print("\n");

			dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			block_num = pCtrl->SetupPacket.wValue.W;
			packet_len = pCtrl->SetupPacket.wLength;

			pCtrl->EP0Data.Count = pCtrl->SetupPacket.wLength;
			pCtrl->EP0Data.pData = buf;
	        pUsbApi->core->DataOutStage(hUsb);
			return LPC_OK;
		break;

		case USB_REQ_DFU_CLRSTATUS:
		case USB_REQ_DFU_ABORT:
			dfu_state = DFU_STATE_dfuIDLE;
			dfu_status = 0;
			packet_len = 0;
			block_num = 0;

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
/* Main Program */
ErrorCode_t usb_dfu_init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR* pIntfDesc, uint32_t* mem_base, uint32_t* mem_size)
{

  //ret = pUsbApi->dfu->init(hUsb, &dfu_param, DFU_STATE_dfuIDLE);
  pUsbApi->core->RegisterClassHandler(hUsb, dfu_ep0, 0);
  //pUsbApi->
  //pUsbApi->core->RegisterEpHandler(hUsb, 0, dfu_ep0, 0);

  init_usb_iap();
  return LPC_OK;
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
/*****************************************************************************
 **   Main Function  main()
 *****************************************************************************/
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

	if(bad_signature) {
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
	usb_param.max_num_ep = 4;
	usb_param.USB_Configure_Event = USB_Configure_Event;

	/* Initialize Descriptor pointers */
	memset((void*) &desc, 0, sizeof(USB_CORE_DESCS_T));
	desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
	desc.full_speed_desc = (uint8_t *) &USB_dfuConfigDescriptor[0];
	desc.high_speed_desc = (uint8_t *) &USB_dfuConfigDescriptor[0];

	/* USB Initialization */
	ret = pUsbApi->hw->Init(&hUsb, &desc, &usb_param);

	pUsbApi->core->RegisterClassHandler(hUsb, dfu_ep0, 0);

	init_usb_iap();

	NVIC_EnableIRQ(USB_IRQn); //  enable USB interrrupts
	//UARTSend((uint8_t *) "Connect\r\n", 9);
	/* now connect */
	pUsbApi->hw->Connect(hUsb, 1);
	LPC_GPIO->B0[21] = 1;


	LPC_GPIO->DIR[0] |= (1<<1);

	while (1) {

		if((u32Milliseconds & 511) < 256) {
			LPC_GPIO->B0[1] = 1;
		} else
		if((u32Milliseconds & 511) > 256) {
			LPC_GPIO->B0[1] = 0;
		}

		if(u32Milliseconds > 10000 && dfu_state == DFU_STATE_dfuIDLE) {
			if(user_code_present()) {
				NVIC_SystemReset();
			}
			u32Milliseconds = 0;
		}

//		if(!LPC_GPIO->B0[1]) {
//			NVIC_SystemReset();
//		}

	}
}

/**********************************************************************
 ** Function name:
 **
 ** Description:
 **
 ** Parameters:
 **
 ** Returned value:
 **********************************************************************/
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
