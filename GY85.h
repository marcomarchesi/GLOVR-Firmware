/* 
 * File:   GY85.h
 * Author: Andrea Verdecchia
 *
 * Created on 20 settembre 2014, 16.35
 */

#ifndef GY85_H
#define	GY85_H

/**********************************************
 * I2C DRIVER DEFINITIONS
 **********************************************/

//DATA OBJECT
typedef struct
{
        unsigned char *buff;
        unsigned int n;
        unsigned char addr;
}DEVICE_DATA;

// DRIVER OBJECT
typedef struct
{
        unsigned int	cmd;
	DEVICE_DATA	*oData;
        void (*tick)(void *, unsigned char);
}DEVICE_DRV;

void Device_drv(DEVICE_DRV *, unsigned char);

#define DEVICE_DRV_DEFAULTS { 0,(DEVICE_DATA *)0,(void (*)(void *))Device_drv }

#define DEVICE_MAX_RETRY   3
#define DEVICE_ONE_BYTE    1

#define GYRO_MAX_RETRY   1000
#define ACCEL_MAX_RETRY   1000
#define COMP_MAX_RETRY   1000

#define DEVICE_IDLE        0
#define DEVICE_WRITE       1
#define DEVICE_READ        2
#define DEVICE_ERR         0xFFFF

/**********************************************
 * CALIBRATION DEFINES
 **********************************************/
#define FACTORACC 0.0392262
#define FACTORGYR 0.0011826
#define FACTORMAG 0.092

/**********************************************
 * ACCELEROMETER DEFINITIONS
 **********************************************/
#define ACCEL_DEVID             0x00
#define ACCEL_THRESH_TAP        0x1D
#define ACCEL_OFSX              0x1E
#define ACCEL_OFSY              0x1F
#define ACCEL_OFSZ              0x20
#define ACCEL_DUR               0x21
#define ACCEL_LATENT            0x22
#define ACCEL_WINDOW            0x23
#define ACCEL_THRESH_ACT        0x24
#define ACCEL_THRESH_INACT      0x25
#define ACCEL_TIME_INACT        0x26
#define ACCEL_ACT_INACT_CTL     0x27
#define ACCEL_THRESH_FF         0x28
#define ACCEL_TIME_FF           0x29
#define ACCEL_TAP_AXES          0x2A
#define ACCEL_ACT_TAP_STATUS    0x2B
#define ACCEL_BW_RATE           0x2C
#define ACCEL_POWER_CTL         0x2D
#define ACCEL_INT_ENABLE        0x2E
#define ACCEL_INT_MAP           0x2F
#define ACCEL_INT_SOURCE        0x30
#define ACCEL_DATA_FORMAT       0x31
#define ACCEL_DATAX0            0x32
#define ACCEL_DATAX1            0x33
#define ACCEL_DATAY0            0x34
#define ACCEL_DATAY1            0x35
#define ACCEL_DATAZ0            0x36
#define ACCEL_DATAZ1            0x37
#define ACCEL_FIFO_CTL          0x38
#define ACCEL_FIFO_STATUS       0x39

#define ACCEL_ADDRESS           0x53

#define A_DRDY  PORTDbits.RD1


/**********************************************
 * GYROSCOPE DEFINITIONS
 **********************************************/
#define GYRO_WHO_AM_I       0x00
#define GYRO_SMPLRT_DIV     0x15
#define GYRO_DLPF_FS        0x16
#define GYRO_INT_CFG        0x17
#define GYRO_INT_STATUS     0x1A
#define GYRO_TEMP_OUT_H     0x1B
#define GYRO_TEMP_OUT_L     0x1C
#define GYRO_XOUT_H         0x1D
#define GYRO_XOUT_L         0x1E
#define GYRO_YOUT_H         0x1F
#define GYRO_YOUT_L         0x20
#define GYRO_ZOUT_H         0x21
#define GYRO_ZOUT_L         0x22
#define GYRO_PWR_MGM        0x3E

#define GYRO_ADDRESS       0x68

#define G_DRDY  PORTDbits.RD2


/**********************************************
 * COMPASS DEFINITIONS
 **********************************************/
#define COMP_CONF_A     0x00
#define COMP_CONF_B     0x01
#define COMP_MODE       0x02
#define COMP_OUT_X_H    0x03
#define COMP_OUT_X_L    0x04
#define COMP_OUT_Z_H    0x05
#define COMP_OUT_Z_L    0x06
#define COMP_OUT_Y_H    0x07
#define COMP_OUT_Y_L    0x08
#define COMP_STAT       0x09
#define COMP_ID_A       0x0A
#define COMP_ID_B       0x0B
#define COMP_ID_C       0x0C

#define COMP_ADDRESS       0x1E

#define C_DRDY  PORTDbits.RD0


/**********************************************
 * ACCELEROMETER FUNCTIONS
 **********************************************/
void Accel_Write_Value(unsigned char, unsigned char);
void Accel_Read_XYZValue(int *);
int Accel_Read_XValue(void);
int Accel_Read_YValue(void);
int Accel_Read_ZValue(void);


/**********************************************
 * GYROSCOPE FUNCTIONS
 **********************************************/
void Gyro_Write_Value(unsigned char, unsigned char);
void Gyro_Read_XYZValue(int *);
int Gyro_Read_XValue(void);
int Gyro_Read_YValue(void);
int Gyro_Read_ZValue(void);
int Get_Temperature(void);


/**********************************************
 * COMPASS FUNCTIONS
 **********************************************/
void Comp_Write_Value(unsigned char, unsigned char);
void Comp_Read_XYZValue(int *);
int Comp_Read_XValue(void);
int Comp_Read_YValue(void);
int Comp_Read_ZValue(void);


/**********************************************
 * GY85 FUNCTIONS
 **********************************************/
void GY85_Init(void);
void GY85_Read_Rough(int *);
void GY85_Read_Real(float *);

#endif	/* GY85_H */

