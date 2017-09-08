/*
 * romdiv.c
 *
 *  Created on: Sep 8, 2017
 *      Author: walmis
 */
#include <stdint.h>
#define INCLUDE_ROM_DIV

typedef struct {
	const uint32_t usbdApiBase;				/*!< USBD API function table base address */
	const uint32_t reserved0;				/*!< Reserved */
	const uint32_t candApiBase;				/*!< CAN API function table base address */
	const uint32_t pwrApiBase;				/*!< Power API function table base address */
#if defined(INCLUDE_ROM_DIV)
	const ROM_DIV_API_T *divApiBase;		/*!< Divider API function table base address */
#else
	const uint32_t reserved1;				/*!< Reserved */
#endif
	const uint32_t reserved2;				/*!< Reserved */
	const uint32_t reserved3;				/*!< Reserved */
	const uint32_t reserved4;				/*!< Reserved */
} LPC_ROM_API_T;

/* Pointer to ROM API function address */
#define LPC_ROM_API_BASE_LOC    0x1FFF1FF8
#define LPC_ROM_API     (*(LPC_ROM_API_T * *) LPC_ROM_API_BASE_LOC)

typedef struct {
	int quot;			/*!< Quotient */
	int rem;			/*!< Reminder */
} IDIV_RETURN_T;

/**
 * @brief Structure containing unsigned integer divider return data.
 */
typedef struct {
	unsigned quot;		/*!< Quotient */
	unsigned rem;		/*!< Reminder */
} UIDIV_RETURN_T;

/**
 * @brief ROM divider API Structure.
 */
typedef struct {
	int (*sidiv)(int numerator, int denominator);							/*!< Signed integer division */
	unsigned (*uidiv)(unsigned numerator, unsigned denominator);			/*!< Unsigned integer division */
	IDIV_RETURN_T (*sidivmod)(int numerator, int denominator);				/*!< Signed integer division with remainder */
	UIDIV_RETURN_T (*uidivmod)(unsigned numerator, unsigned denominator);	/*!< Unsigned integer division with remainder */
} ROM_DIV_API_T;

/* Redirector of signed 32 bit integer divider to ROM routine */
int __aeabi_idiv(int numerator, int denominator)
{
	ROM_DIV_API_T const *pROMDiv = LPC_ROM_API->divApiBase;
	return pROMDiv->sidiv(numerator, denominator);
}

/* Redirector of unsigned 32 bit integer divider to ROM routine */
unsigned __aeabi_uidiv(unsigned numerator, unsigned denominator)
{
	ROM_DIV_API_T const *pROMDiv = LPC_ROM_API->divApiBase;
	return pROMDiv->uidiv(numerator, denominator);
}

/* Redirector of signed 32 bit integer divider with reminder to ROM routine */
__value_in_regs IDIV_RETURN_T __aeabi_idivmod(int numerator, int denominator)
{
	ROM_DIV_API_T const *pROMDiv = LPC_ROM_API->divApiBase;
	return pROMDiv->sidivmod(numerator, denominator);
}

/* Redirector of unsigned 32 bit integer divider with reminder to ROM routine */
__value_in_regs UIDIV_RETURN_T __aeabi_uidivmod(unsigned numerator, unsigned denominator)
{
	ROM_DIV_API_T const *pROMDiv = LPC_ROM_API->divApiBase;
	return pROMDiv->uidivmod(numerator, denominator);
}
