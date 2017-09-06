//-----------------------------------------------------------------------------
// Software that is described herein is for illustrative purposes only  
// which provides customers with programming information regarding the  
// products. This software is supplied "AS IS" without any warranties.  
// NXP Semiconductors assumes no responsibility or liability for the 
// use of the software, conveys no license or title under any patent, 
// copyright, or mask work right to the product. NXP Semiconductors 
// reserves the right to make changes in the software without 
// notification. NXP Semiconductors also make no representation or 
// warranty that such application will be suitable for the specified 
// use without further testing or modification. 
//-----------------------------------------------------------------------------

#include "sbl_iap.h"
#include "sbl_config.h"
#include <app_usbd_cfg.h>
#include <string.h>
#include "LPC11Uxx.h"            
#include <uart.h>

unsigned param_table[5];
unsigned result_table[5];

unsigned cclk;
//char flash_buf[FLASH_BUF_SIZE];
char flash_buf[FLASH_BUF_SIZE];

unsigned * flash_address;
unsigned byte_ctr;

#define iap_entry ((void (*)(unsigned [],unsigned []))(IAP_ADDRESS))

void write_data(unsigned cclk,unsigned flash_address,unsigned * flash_data_buf, unsigned count);
void find_erase_prepare_sector(unsigned cclk, unsigned flash_address);
void erase_sector_usb(unsigned start_sector,unsigned end_sector,unsigned cclk);
void prepare_sector_usb(unsigned start_sector,unsigned end_sector,unsigned cclk);

unsigned write_flash(unsigned * dst, char * src, unsigned no_of_bytes, int last)
{
  unsigned enabled_irqs;
  	__disable_irq();

	if (flash_address == (unsigned *) UPDATE_REQD) {
		/* Store flash start address */
		flash_address = (unsigned *) dst;
		memset(flash_buf, 0xFF, sizeof(flash_buf));
	}
	memcpy(&flash_buf[byte_ctr], src, no_of_bytes);
	byte_ctr = byte_ctr + no_of_bytes;
	if(last) {
		byte_ctr = FLASH_BUF_SIZE;
	}
	if (byte_ctr == FLASH_BUF_SIZE ) {
		/* We have accumulated enough bytes to trigger a flash write */
		find_erase_prepare_sector(cclk, (unsigned) flash_address);
		if (result_table[0] != CMD_SUCCESS) {
			__enable_irq();
			UARTSend("wr error0\n", 10);
			return 1;
		}
		write_data(cclk, (unsigned) flash_address, (unsigned *) flash_buf,
				FLASH_BUF_SIZE);
		if (result_table[0] != CMD_SUCCESS) {
			__enable_irq();
			UARTSend("wr error1\n", 10);
			return 1;
		}
		/* Reset byte counter and flash address */
		byte_ctr = 0;
		flash_address = (unsigned *) UPDATE_REQD;
	}
	__enable_irq();
	return (CMD_SUCCESS);
}

void find_erase_prepare_sector(unsigned cclk, unsigned flash_address)
{
    unsigned i;
    unsigned end_sector;


    end_sector = MAX_USER_SECTOR;
    for(i=0;i<=end_sector;i++)
    {
        if(flash_address < (SECTOR_0_START_ADDR + ((i + 1) * SECTOR_SIZE)))
        {
            if( flash_address == SECTOR_0_START_ADDR + (SECTOR_SIZE * i))
            {
                prepare_sector_usb(i,i,cclk);
                if (result_table[0] != CMD_SUCCESS) {
                	return;
                }
                erase_sector_usb(i,i,cclk);
                if (result_table[0] != CMD_SUCCESS) {
                	return;
                }
            }
            prepare_sector_usb(i,i,cclk);
            break;
        }
    }

}

void write_data(unsigned cclk,unsigned flash_address,unsigned * flash_data_buf, unsigned count)
{
    param_table[0] = COPY_RAM_TO_FLASH;
    param_table[1] = flash_address;
    param_table[2] = (unsigned)flash_data_buf;
    param_table[3] = count;
    param_table[4] = cclk;
    iap_entry(param_table,result_table);
}

void erase_sector_usb(unsigned start_sector,unsigned end_sector,unsigned cclk)
{
    param_table[0] = ERASE_SECTOR;
    param_table[1] = start_sector;
    param_table[2] = end_sector;
    param_table[3] = cclk;
    iap_entry(param_table,result_table);
}

void prepare_sector_usb(unsigned start_sector,unsigned end_sector,unsigned cclk)
{
    param_table[0] = PREPARE_SECTOR_FOR_WRITE;
    param_table[1] = start_sector;
    param_table[2] = end_sector;
    param_table[3] = cclk;
    iap_entry(param_table,result_table);
}

void init_usb_iap( void )
{
	cclk = CCLK;
	byte_ctr = 0;
	flash_address = (unsigned *)UPDATE_REQD;
}

int user_code_present(void) {
	unsigned *pmem, checksum, i;

	param_table[0] = BLANK_CHECK_SECTOR;
	param_table[1] = 1;
	param_table[2] = 1;
	iap_entry(param_table, result_table);
	if (result_table[0] == CMD_SUCCESS) {

		return 0;
	}
	return 1;
}
