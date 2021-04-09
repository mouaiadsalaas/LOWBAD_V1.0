/****************************************************************
 *  Header File: LOWBAD_IO_PIN.h
 *  Description: LOWBAD_IO_PIN (lowbad input and output according to our IC)
 *  Date       : 02.04.2021
 *  Author     : MOUAIAD SALAAS
 *     ---------- ---------- ----------------------------
 *
 * Copyright MOUAIAD SALAAS & ALI YOLCU , 2021
 *
 * This unpublished material is proprietary to MOUAIAD SALAAS.
 * All rights reserved. The methods and
 * techniques described herein are considered trade secrets
 * and/or confidential. Reproduction or distribution, in whole
 * or in part, is forbidden except by express written permission
 * of MOUAIAD SALAAS.
 ****************************************************************/
/**************************************************************
 * Notes:
 **************************************************************/
#ifndef INCLUDE_LOWBAD_IO_PIN_H_
#include "PINOUT.h"
#include "MCP23S17.h"

#define GIO_FUNCTION_SELECT_PORT                     MCP_GPB
#define GIO_FUNCTION_SELECT_PIN                      MCP_GPB4

#define GPO_SYSTEM_ACTIVE_PORT                       MCP_GPB
#define GPO_SYSTEM_ACTIVE_PIN                        MCP_GPB3

#define GIO_ISS_PORT                                 MCP_GPB
#define GIO_ISS_PIN                                  MCP_GPB2

#define GIO_ALIGNMENT_RIGHT_BTN_PORT                 MCP_GPA
#define GIO_ALIGNMENT_RIGHT_BTN_PIN                  MCP_GPB6

#define GIO_ALIGNMENT_LEFT_BTN_PORT                      MCP_GPA
#define GIO_ALIGNMENT_LEFT_BTN_PIN                       MCP_GPB7

#define GIO_ALIGNMENT_AUTO_BTN_PORT                  MCP_GPA
#define GIO_ALIGNMENT_AUTO_BTN_PIN                   MCP_GPA7

#define GIO_ALIGNMENT_SET_BTN_PORT                  MCP_GPB
#define GIO_ALIGNMENT_SET_BTN_PIN                   MCP_GPB0

#define GIO_STOP_BTN_PORT                           MCP_GPB
#define GIO_STOP_BTN_PIN                            MCP_GPB1



#define GIO_PUMP_PORT                          NCV8403ADTRKG_RIGHT_INT1_PORT
#define GIO_PUMP_PIN                           NCV8403ADTRKG_RIGHT_INT1_PIN

#define GIO_ALIGNMENT_VALF_RIGHT_PORT          NCV8403ADTRKG_LEFT_INT1_PORT
#define GIO_ALIGNMENT_VALF_RIGHT_PIN           NCV8403ADTRKG_LEFT_INT1_PIN

#define GIO_ALIGNMENT_VALF_LEFT_PORT           VN5T006ASPTRE_BOTTOM_INT1_PORT
#define GIO_ALIGNMENT_VALF_LEFT_PIN            VN5T006ASPTRE_BOTTOM_INT1_PIN

//NOT CONNECTED YET IN THE BOARD so we connected it to 73 in 4 channel left power switch
#define GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT      VNQ5050KTRE_LEFT_INT1_PORT
#define GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN       VNQ5050KTRE_LEFT_INT1_PIN

#define GIO_ALIGNMENT_WARNING_LAMP_RED_PORT     VNQ5050KTRE_LEFT_INT3_PORT
#define GIO_ALIGNMENT_WARNING_LAMP_RED_PIN      VNQ5050KTRE_LEFT_INT3_PIN

#define GIO_OIL_PUMP_PORT                       VND5T100AJTRE_LEFT_INT2_PORT
#define GIO_OIL_PUMP_PIN                        VND5T100AJTRE_LEFT_INT2_PIN
#define INCLUDE_LOWBAD_IO_PIN_H_



#endif /* INCLUDE_LOWBAD_IO_PIN_H_ */
