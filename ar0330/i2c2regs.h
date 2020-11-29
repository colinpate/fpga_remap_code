#ifndef ACCELEROMETER_ADXL345_H_
#define ACCELEROMETER_ADXL345_H_

#include <stdint.h>

//typedef int bool;
#define true 1
#define false 0

#define MAPPED_BASE 0xffc00000
#define MAPPED_SPAN 0x00200000

// I2C0 Controller Registers
#define I2C0_CON                ((volatile unsigned int *) 0x00006000)
#define I2C0_TAR                ((volatile unsigned int *) 0x00006004)
#define I2C0_DATA_CMD           ((volatile unsigned int *) 0x00006010)
#define I2C0_SS_SCL_HCNT        ((volatile unsigned int *) 0x00006014)
#define I2C0_SS_SCL_LCNT        ((volatile unsigned int *) 0x00006028)
#define I2C0_FS_SCL_HCNT        ((volatile unsigned int *) 0x0000601c)
#define I2C0_FS_SCL_LCNT        ((volatile unsigned int *) 0x00006020)
#define I2C0_INTR_MASK          ((volatile unsigned int *) 0x00006030)
#define I2C0_CLR_INTR           ((volatile unsigned int *) 0x00006040)
#define I2C0_TX_ABRT            ((volatile unsigned int *) 0x00006054)
#define I2C0_ENABLE             ((volatile unsigned int *) 0x0000606c)
#define I2C0_STATUS             ((volatile unsigned int *) 0x00006070)
#define I2C0_TXFLR              ((volatile unsigned int *) 0x00006074)
#define I2C0_RXFLR              ((volatile unsigned int *) 0x00006078)
#define I2C0_TX_ABRT_SOURCE     ((volatile unsigned int *) 0x00006080)
#define I2C0_ENABLE_STATUS      ((volatile unsigned int *) 0x0000609c)

// SYSMGR Pin Muxing Registers
/* GENERALIO7 (trace_d6): 
    0 : Pin is connected to GPIO/LoanIO number 55. 
    1 : Pin is connected to Peripheral signal I2C0.SDA. 
    2 : Pin is connected to Peripheral signal SPIS1.SS0. 
    3 : Pin is connected to Peripheral signal TRACE.D6. */
#define SYSMGR_GENERALIO7       ((volatile unsigned int *) 0x0010849C)
/* GENERALIO8 (trace_d7): 
    0 : Pin is connected to GPIO/LoanIO number 56. 
    1 : Pin is connected to Peripheral signal I2C0.SCL. 
    2 : Pin is connected to Peripheral signal SPIS1.MISO. 
    3 : Pin is connected to Peripheral signal TRACE.D7. */
#define SYSMGR_GENERALIO8       ((volatile unsigned int *) 0x001084A0)
/* GPLMUX55 and GPLMUX56: 
    0 : LoanIO. 
    1 : GPIO. */
#define SYSMGR_GPLMUX55         ((volatile unsigned int *) 0x001086B0)
#define SYSMGR_GPLMUX56         ((volatile unsigned int *) 0x001086B4)
/* I2C0USEFPGA: 
    0 : I2C0 uses HPS Pins. 
    1 : I2C0 uses the FPGA Inteface. */
//#define SYSMGR_I2C0USEFPGA      ((volatile unsigned int *) 0xffd08704)
#define SYSMGR_I2C2USEFPGA      ((volatile unsigned int *) 0x00108728)
     
// Reset Manager Registers
#define RSTMGR_BRGMODRST        ((volatile unsigned int *) 0x0010501c)
     

#endif /*ACCELEROMETER_ADXL345_SPI_H_*/
