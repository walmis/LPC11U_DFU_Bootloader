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
void blank_check_sector(int i);

unsigned write_flash(unsigned * dst, char * src, unsigned no_of_bytes, int last)
{
  unsigned enabled_irqs;

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

			//print("wr error0 "); putDec(result_table[0]); print("\n");
			return 1;
		}
		write_data(cclk, (unsigned) flash_address, (unsigned *) flash_buf,
				FLASH_BUF_SIZE);
		//putHex(flash_address); print("\n");
		if (result_table[0] != CMD_SUCCESS) {
			//print("wr error1 "); putDec(result_table[0]); print("\n");
			return 1;
		}

//		if(flash_address > 0x2000) {
//			__enable_irq();
//			print("wr cmp error"); print("\n");
//			return 1;
//		}
		if(memcmp(flash_address, flash_buf, FLASH_BUF_SIZE) != 0) {
			print("wr cmp error"); print("\n");
			return 1;
		}

		/* Reset byte counter and flash address */
		byte_ctr = 0;
		flash_address = (unsigned *) UPDATE_REQD;
	}
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
                //putHex(flash_address); print("\n");
            	prepare_sector_usb(i,i,cclk);
                if (result_table[0] != CMD_SUCCESS) {
                	return;
                }
                erase_sector_usb(i,i,cclk);
                if (result_table[0] != CMD_SUCCESS) {
                	return;
                }
                blank_check_sector(i);
                if (result_table[0] != CMD_SUCCESS) {
                	return;
                }

            }
            prepare_sector_usb(i,i,cclk);
            break;
        }
    }
    result_table[0] = CMD_SUCCESS;
}

void write_data(unsigned cclk,unsigned flash_address,unsigned * flash_data_buf, unsigned count)
{
	__disable_irq();
    param_table[0] = COPY_RAM_TO_FLASH;
    param_table[1] = flash_address;
    param_table[2] = (unsigned)flash_data_buf;
    param_table[3] = count;
    param_table[4] = cclk;
    iap_entry(param_table,result_table);
    __enable_irq();
}

void erase_sector_usb(unsigned start_sector,unsigned end_sector,unsigned cclk)
{
	__disable_irq();
    param_table[0] = ERASE_SECTOR;
    param_table[1] = start_sector;
    param_table[2] = end_sector;
    param_table[3] = cclk;
    iap_entry(param_table,result_table);
    __enable_irq();
}

void prepare_sector_usb(unsigned start_sector,unsigned end_sector,unsigned cclk)
{
	__disable_irq();
    param_table[0] = PREPARE_SECTOR_FOR_WRITE;
    param_table[1] = start_sector;
    param_table[2] = end_sector;
    param_table[3] = cclk;
    iap_entry(param_table,result_table);
    __enable_irq();

}

void init_usb_iap( void )
{
	cclk = CCLK;
	byte_ctr = 0;
	flash_address = (unsigned *)UPDATE_REQD;
}

void blank_check_sector(int i) {
	__disable_irq();
	param_table[0] = BLANK_CHECK_SECTOR;
	param_table[1] = i;
	param_table[2] = i;
	iap_entry(param_table, result_table);
	__enable_irq();
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
