/**********************************************************************************************************************
 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  Device_TMS570LS07.c
 *      Project:  Tms570_TIFEEDriver
 *       Module:  TIFEEDriver
 *    Generator:  None
 *
 *  Description:  This file defines the layout of Bank7.
 *---------------------------------------------------------------------------------------------------------------------
 * Author:  Vishwanath Reddy
 *---------------------------------------------------------------------------------------------------------------------
 * Revision History
 *---------------------------------------------------------------------------------------------------------------------
 * Version        Date         Author               Change ID        Description
 *--------------------------------------------------------------------------------------------------------------------- 
 * 01.17.01		  30Oct2014    Vishwanath Reddy 	SDOCM00113536    Initial Version.
 *********************************************************************************************************************/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com  
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


 /*********************************************************************************************************************
 * INCLUDES
 *********************************************************************************************************************/
#include "hal_stdtypes.h"
#include "Device_TMS570LS07.h"

/* Start Device Definition */
/*SAFETYMCUSW 79 S MR:19.4 <APPROVED> "Reason - F021_CPU0_BASE_ADDRESS is a symbolic constant"*/
/*SAFETYMCUSW 95 S MR:11.1,11.4 <APPROVED> "Reason -  Casting is required here."*/
#define DEVICE_BANKCONTROLREGISTER  F021_CPU0_BASE_ADDRESS  /* Control Register Address */

/*SAFETYMCUSW 580 S MR:1.1 <APPROVED> "Reason - This is the format to use for specifying memorysections."*/
#define FEE_START_SEC_CONST_UNSPECIFIED
/*SAFETYMCUSW 338 S MR:19.1 <APPROVED> "Reason - This is the format to use for specifying memorysections."*/
#include "MemMap.h"

const Device_FlashType Device_FlashDevice =
{
   "TMS570LS07x",                      	/* Device name */
   0x00000000U,                         /* Device Engineering ID */
   Device_ErrorHandlingEcc,             /* Indicates which type of bit Error handling is on the device */   
   Device_CortexR4,					    /* Indicates the Master core type on the device */	   
   TRUE,                              	/* Indicates if the device supports Flash interrupts for processing Flash */   
   31U,                                 /* Nominal time for one write command operation in uS - This value still needs 
                                           to be characterized */
   300U,                                /* Maximum time for one write command operation in uS - This value still needs 
                                           to be characterized */       
   {                                    /* Array of Banks on the device */
	   {    /* Start of Bank Definition */
			 /*SAFETYMCUSW 440 S MR:11.3 <APPROVED> "Reason -  Casting is required here."*/
			 /*SAFETYMCUSW 95 S MR:11.1,11.4 <APPROVED> "Reason -  Casting is required here."*/
			 DEVICE_BANKCONTROLREGISTER,	   	/* Pointer to the Flash control register for this bank */
			 Fapi_FlashBank7,               	/* Core number for this bank */                
			{                           /* Array of the Sectors within a bank */
					{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector0 is enum."*/
					Fapi_FlashSector0,   /* Sector number */
					0xF0200000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100000U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector1 is enum."*/
					Fapi_FlashSector1,   /* Sector number */
					0xF0201000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100200U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector2 is enum."*/
					Fapi_FlashSector2,   /* Sector number */
					0xF0202000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100400U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector3 is enum."*/
					Fapi_FlashSector3,   /* Sector number */
					0xF0203000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100600U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector4 is enum."*/
					Fapi_FlashSector4,   /* Sector number */
					0xF0204000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100800U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector5 is enum."*/
					Fapi_FlashSector5,   /* Sector number */
					0xF0205000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100A00U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector6 is enum."*/
					Fapi_FlashSector6,   /* Sector number */
					0xF0206000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100C00U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector7 is enum."*/
					Fapi_FlashSector7,   /* Sector number */
					0xF0207000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0100E00U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector8 is enum."*/
					Fapi_FlashSector8,   /* Sector number */
					0xF0208000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101000U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector9 is enum."*/
					Fapi_FlashSector9,   /* Sector number */
					0xF0209000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101200U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector10 is enum."*/
					Fapi_FlashSector10,   /* Sector number */
					0xF020A000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101400U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector11 is enum."*/
					Fapi_FlashSector11,   /* Sector number */
					0xF020B000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101600U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector12 is enum."*/
					Fapi_FlashSector12,   /* Sector number */
					0xF020C000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101800U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector13 is enum."*/
					Fapi_FlashSector13,   /* Sector number */
					0xF020D000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101A00U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector14 is enum."*/
					Fapi_FlashSector14,   /* Sector number */
					0xF020E000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101C00U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				},  /* End of Sector Definition */
				{   /* Start of Sector Definition */
					/*SAFETYMCUSW 93 S MR:6.1,6.2,10.1,10.2,10.3,10.4 <APPROVED> "Reason -  Fapi_FlashSector15 is enum."*/
					Fapi_FlashSector15,   /* Sector number */
					0xF020F000U,         /* Starting address of the sector */
					0x00001000U,         /* Length of the sector */
					100000U,              /* Number of cycles the sector is rated for */
					0xF0101E00U,		 /* Defines the address offset to the Error Handling address */
					0x00000200U			 /* Length of the ECC for a sector */
				}  /* End of Sector Definition */				
			}
	   }  /* End of Bank Definition */
   }   /* End of Bank Array */
};  /* End of Device Definition */

/*SAFETYMCUSW 580 S MR:1.1 <APPROVED> "Reason - This is the format to use for specifying memorysections."*/
#define FEE_STOP_SEC_CONST_UNSPECIFIED
/*SAFETYMCUSW 338 S MR:19.1 <APPROVED> "Reason - This is the format to use for specifying memorysections."*/
#include "MemMap.h"

/* End of File */
