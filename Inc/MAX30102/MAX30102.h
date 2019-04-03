/** \file max30102.h ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30102.h
* Description: This module is an embedded controller driver header file for MAX30102
*
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*  Modified original MAXIM source code on: 13.01.2019
*		Author: Mateusz Salamon
*		www.msalamon.pl
*		mateusz@msalamon.pl
*	Code is modified to work with STM32 HAL libraries.
*
*	Website: https://msalamon.pl/palec-mi-pulsuje-pulsometr-max30102-pod-kontrola-stm32/
*	GitHub:  https://github.com/lamik/MAX30102_STM32_HAL
*
*/
#ifndef MAX30102_H_
#define MAX30102_H_

#define MAX30102_ADDRESS 0xAE	//(0x57<<1)
//#define MAX30102_USE_INTERNAL_TEMPERATURE

#define MAX30102_MEASUREMENT_SECONDS 5
#define MAX30102_SAMPLES_PER_SECOND	100 // 50, 100, 200, 400, 800, 100, 1600, 3200 sample rating
#define MAX30102_FIFO_ALMOST_FULL_SAMPLES 17

#define MAX30102_BUFFER_LENGTH	((MAX30102_MEASUREMENT_SECONDS+1)*MAX30102_SAMPLES_PER_SECOND)

//
//	Calibration
//
#define MAX30102_IR_LED_CURRENT_LOW		0x01
#define MAX30102_RED_LED_CURRENT_LOW	0x00
#define MAX30102_IR_LED_CURRENT_HIGH	0x24
#define MAX30102_RED_LED_CURRENT_HIGH	0x24
#define MAX30102_IR_VALUE_FINGER_ON_SENSOR 1600
#define MAX30102_IR_VALUE_FINGER_OUT_SENSOR 50000
//
//	Register addresses
//
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

//
//	Interrupt Status 1 (0x00)
//	Interrupt Status 2 (0x01)
//	Interrupt Enable 1 (0x02)
//	Interrupt Enable 2 (0x03)
//
#define	INT_A_FULL_BIT			7
#define	INT_PPG_RDY_BIT			6
#define	INT_ALC_OVF_BIT			5
#define	INT_DIE_TEMP_RDY_BIT	1
#define	INT_PWR_RDY_BIT			0	// Only STATUS register

//
//	FIFO Configuration (0x08)
//
#define FIFO_CONF_SMP_AVE_BIT 			7
#define FIFO_CONF_SMP_AVE_LENGHT 		3
#define FIFO_CONF_FIFO_ROLLOVER_EN_BIT 	4
#define FIFO_CONF_FIFO_A_FULL_BIT 		3
#define FIFO_CONF_FIFO_A_FULL_LENGHT 	4

#define FIFO_SMP_AVE_1	0
#define FIFO_SMP_AVE_2	1
#define FIFO_SMP_AVE_4	2
#define FIFO_SMP_AVE_8	3
#define FIFO_SMP_AVE_16	4
#define FIFO_SMP_AVE_32	5

//
//	Mode Configuration (0x09)
//
#define MODE_SHDN_BIT		7
#define MODE_RESET_BIT		6
#define MODE_MODE_BIT		2
#define MODE_MODE_LENGTH	3

#define MODE_HEART_RATE_MODE	2
#define MODE_SPO2_MODE			3
#define MODE_MULTI_LED_MODE		7

//
//	SpO2 Configuration (0x0A)
//
#define SPO2_CONF_ADC_RGE_BIT		6
#define SPO2_CONF_ADC_RGE_LENGTH	2
#define SPO2_CONF_SR_BIT			4
#define SPO2_CONF_SR_LENGTH			3
#define SPO2_CONF_LED_PW_BIT		1
#define SPO2_CONF_LED_PW_LENGTH		2

#define	SPO2_ADC_RGE_2048	0
#define	SPO2_ADC_RGE_4096	1
#define	SPO2_ADC_RGE_8192	2
#define	SPO2_ADC_RGE_16384	3

#define	SPO2_SAMPLE_RATE_50		0
#define	SPO2_SAMPLE_RATE_100	1
#define	SPO2_SAMPLE_RATE_200	2
#define	SPO2_SAMPLE_RATE_400	3
#define	SPO2_SAMPLE_RATE_800	4
#define	SPO2_SAMPLE_RATE_1000	5
#define	SPO2_SAMPLE_RATE_1600	6
#define	SPO2_SAMPLE_RATE_3200	7

#if(MAX30102_SAMPLES_PER_SECOND == 50)
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_50
#elif((MAX30102_SAMPLES_PER_SECOND == 100))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_100
#elif((MAX30102_SAMPLES_PER_SECOND == 200))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_200
#elif((MAX30102_SAMPLES_PER_SECOND == 400))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_400
#elif((MAX30102_SAMPLES_PER_SECOND == 800))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_800
#elif((MAX30102_SAMPLES_PER_SECOND == 1000))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_1000
#elif((MAX30102_SAMPLES_PER_SECOND == 1600))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_1600
#elif((MAX30102_SAMPLES_PER_SECOND == 3200))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_3200
#else
#error "Wrong Sample Rate value. Use 50, 100, 200, 400, 800, 1000, 1600 or 3200."
#endif

#define	SPO2_PULSE_WIDTH_69			0
#define	SPO2_PULSE_WIDTH_118		1
#define	SPO2_PULSE_WIDTH_215		2
#define	SPO2_PULSE_WIDTH_411		3

//
//	Status enum
//
typedef enum{
	MAX30102_ERROR 	= 0,
	MAX30102_OK 	= 1
} MAX30102_STATUS;

//
//	Functions
//
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c);
MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led, volatile uint32_t *pun_ir_led);
MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data);
MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data);
//
//	Interrupts
//
MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status);
MAX30102_STATUS Max30102_SetIntAlmostFullEnabled(uint8_t Enable);
MAX30102_STATUS Max30102_SetIntFifoDataReadyEnabled(uint8_t Enable);
MAX30102_STATUS Max30102_SetIntAmbientLightCancelationOvfEnabled(uint8_t Enable);
#ifdef MAX30102_USE_INTERNAL_TEMPERATURE
MAX30102_STATUS Max30102_SetIntInternalTemperatureReadyEnabled(uint8_t Enable);
#endif
void Max30102_InterruptCallback(void);
//
//	FIFO Configuration
//
MAX30102_STATUS Max30102_FifoWritePointer(uint8_t Address);
MAX30102_STATUS Max30102_FifoOverflowCounter(uint8_t Address);
MAX30102_STATUS Max30102_FifoReadPointer(uint8_t Address);
MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value);
MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable);
MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value); // 17-32 samples ready in FIFO
//
//	Mode Configuration
//
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable);
MAX30102_STATUS Max30102_Reset(void);
MAX30102_STATUS Max30102_SetMode(uint8_t Mode);
//
//	SpO2 Configuration
//
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value);
MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value);
MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value);
//
//	LEDs Pulse Amplitute Configuration
//
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value);
MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value);
//
//	Usage functions
//
void Max30102_Task(void);
int32_t Max30102_GetHeartRate(void);
int32_t Max30102_GetSpO2Value(void);

#endif /* MAX30102_H_ */
