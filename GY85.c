/*
 * File:   GY85.c
 * Author: Andrea Verdecchia
 *
 * Created on 20 settembre 2014, 16.35
 */

#include "main.h"

//The "Done" variable should be used in the Interrupt routine of I2C module.
unsigned int Done;
float acc_matrix[4][3] = {{0.9841,-0.0098,0.0006},{0.0061,0.9683,0.0018},{0.0052,0.0037,0.9824},{-0.1792,-0.0318,1.5089}};
float gyr_x_offset = -0.008694, gyr_y_offset = 0.028165, gyr_z_offset = 0.002352, mag_x_factor = 1, mag_y_factor = 1, mag_z_factor = 1, mag_x_offset = 0, mag_y_offset = 0, mag_z_offset = 0;

DEVICE_DRV devicedrv = DEVICE_DRV_DEFAULTS;
DEVICE_DATA wData;
DEVICE_DATA rData;

/******************************************************************************
 * I2C Serial Device, STATE-MACHINE BASED DRIVER
 ******************************************************************************/
void Device_drv(DEVICE_DRV *device, unsigned char DEVICE_ADDRESS)
{
    static int state=0, cntr=0, rtrycntr=0;

    switch(state)
    {
        case 0:
            if((device->cmd == DEVICE_WRITE)||(device->cmd == DEVICE_READ))
		state=1;
            break;

	/************************************
	 * Control/Address Phase            
	 ************************************/
        case 1:
            // Start Condition
            I2C1CONbits.SEN=1;
            state=state+1;
            break;

        case 2:
            // Start Byte with device select ID
	    if(Done==1)
            {
		Done=0;
          	state=state+1;
  		I2C1TRN=(DEVICE_ADDRESS<<1);             //Device ID is 0x00D6.
            }
            break;

        case 3:
            // Send address byte of register, if ack is received. Else Retry
            if(Done==1)
            {
		Done=0;
  		if(I2C1STATbits.ACKSTAT==1)
                {		// Ack Not received, Retry
                    if(rtrycntr < DEVICE_MAX_RETRY)
			state=17;
                    else
			state=15;			// Flag error and exit
		}
                else
                {
                    rtrycntr=0;

                    if(device->oData->n>1)
                    {
                        I2C1TRN=((device->oData->addr)|0x80);
                        state=state+1;
                    }
                    else
                    {
                        I2C1TRN=(device->oData->addr);
                        state=state+1;
                    }
		}
            }
            break;

        case 4:
            // Read or Write
            if(Done==1)
            {
		Done=0;
		if(I2C1STATbits.ACKSTAT==1)
                {		// Ack Not received, Flag error and exit
                    state=15;
		}
                else
                {
                    if(device->cmd == DEVICE_WRITE)
                        state=state+1;
                    if(device->cmd == DEVICE_READ)
                        state=7;
		}
            }
            break;

	/*************************************
	 * Write Data Phase
	 *************************************/
        case 5:
            // Send data
            I2C1TRN=*(device->oData->buff + cntr);
            state=state+1;
            cntr=cntr+1;
            break;

        case 6:
            // Look for end of data or no Ack
            if(Done==1)
            {
		Done=0;
		state=state-1;
		if(I2C1STATbits.ACKSTAT==1)
                {		// Ack Not received, Flag error and exit
                    state=15;
		}
                else
                {
                    if(cntr == device->oData->n)
                        state=13;   				// Close the Frame
		}
            }
            break;

	/************************************
	 * Read Data Phase
	 ************************************/
        case 7:
            // Repeat Start
            I2C1CONbits.RSEN=1;
            state=state+1;
            break;

        case 8:
            // Re-send ID byte with W/R=R
	    if(Done==1)
            {
		Done=0;
          	state=state+1;
                I2C1TRN=((DEVICE_ADDRESS)<<1)|(0x0001);
            }
            break;

        case 9:
            // Check, if control byte went ok
	    if(Done==1)
            {
		Done=0;
          	state=state+1;
                if(I2C1STATbits.ACKSTAT==1) 		// Ack Not received, Flag error and exit
                    state=15;
            }
            break;

        case 10:
            // Receive Enable
            I2C1CONbits.RCEN=1;
            state++;
            break;

        case 11:
            // Receive data
	    if(Done==1)
            {
		Done=0;
		state=state+1;
                *(device->oData->buff+cntr)=I2C1RCV;
		cntr++;
                if(cntr == device->oData->n)
                {
                    I2C1CONbits.ACKDT=1;		// No MACK
		}
                else
                {
                    I2C1CONbits.ACKDT=0;		// MACK
		}
                I2C1CONbits.ACKEN=1;
            }
            break;

        case 12:
	    if(Done==1)
            {
		Done=0;
		if(cntr == device->oData->n)
                    state=state+1;
           	else
                    state=state-2;
            }
            break;

	/************************************
	 * Stop Sequence
	 ************************************/
        case 13:
            I2C1CONbits.PEN=1;
            state++;
            break;

        case 14:
 	    if(Done==1)
            {
		Done=0;
		state=0;
		cntr=0;
		device->cmd=DEVICE_IDLE;
            }
            break;

	/*************************************
	 * Set Error
	 *************************************/
        case 15:
            I2C1CONbits.PEN=1;
            state++;
            break;

        case 16:
 	    if(Done==1)
            {
		Done=0;
		state=0;
		rtrycntr=0;
		cntr=0;
		device->cmd=DEVICE_ERR;
            }
            break;

	/*************************************
	 * Retry
	 *************************************/
        case 17:
            I2C1CONbits.PEN=1;
            state++;
            rtrycntr++;
            break;

        case 18:
 	    if(Done==1)
            {
		Done=0;
		state=0;
		cntr=0;
            }
            break;
    }
}


/*********************************************************************
 * GYROSCOPE FUNCTIONS
 *********************************************************************/
// WRITE VALUE FUNCTION
void Gyro_Write_Value(unsigned char reg, unsigned char conf)
{
    unsigned char wBuff = conf;

    wData.buff=&wBuff;
    wData.n=1;
    wData.addr=reg;

    // Write Data
    devicedrv.oData = &wData;
    devicedrv.cmd = DEVICE_WRITE;

    while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
    {
        devicedrv.tick(&devicedrv,GYRO_ADDRESS);
    }

    if(devicedrv.cmd == DEVICE_ERR)
    {
        printf("\n\tGYROSCOPE: WRITE ERROR");
    }
}

// READ X-Y-Z VALUE FUNCTION
void Gyro_Read_XYZValue(int xyz_data[3])
{
    unsigned int retry = 0;
    unsigned char rBuff[6] = {0,0,0,0,0,0};

    rData.buff=rBuff;
    rData.n=6;
    rData.addr=GYRO_XOUT_H;

    while(retry<GYRO_MAX_RETRY)
    {
        if(G_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,GYRO_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tGYROSCOPE: READ VALUE ERROR");
            }

            xyz_data[0] = ((int)(rBuff[0]<<8)|rBuff[1]);
            xyz_data[1] = ((int)(rBuff[2]<<8)|rBuff[3]);
            xyz_data[2] = ((int)(rBuff[4]<<8)|rBuff[5]);

            return;
        }
        else
        {
            retry++;
            if(retry==GYRO_MAX_RETRY)
            {
                printf("\n\tGYROSCOPE: ATTEMPTS TO READ XYZ OUT");
                break;
            }
        }
    }
    return;
}

// READ X VALUE FUNCTION
int Gyro_Read_XValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=GYRO_XOUT_H;

    while(retry<GYRO_MAX_RETRY)
    {
        if(G_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,GYRO_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tGYROSCOPE: READ VALUE ERROR");
            }

            return (int)(rBuff[0]<<8)|rBuff[1];
        }
        else
        {
            retry++;
            if(retry==GYRO_MAX_RETRY)
            {
                printf("\n\tGYROSCOPE: ATTEMPTS TO READ X OUT");
                break;
            }
        }
    }
    return 0;
}

// READ Y VALUE FUNCTION
int Gyro_Read_YValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=GYRO_YOUT_H;

    while(retry<GYRO_MAX_RETRY)
    {
        if(G_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,GYRO_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tGYROSCOPE: READ VALUE ERROR");
            }

            return (int)(rBuff[0]<<8)|rBuff[1];
        }
        else
        {
            retry++;
            if(retry==GYRO_MAX_RETRY)
            {
                printf("\n\tGYROSCOPE: ATTEMPTS TO READ Y OUT");
                break;
            }
        }
    }
    return 0;
}

// READ Z VALUE FUNCTION
int Gyro_Read_ZValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=GYRO_ZOUT_H;

    while(retry<GYRO_MAX_RETRY)
    {
        if(G_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,GYRO_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tGYROSCOPE: READ VALUE ERROR");
            }

            return (int)(rBuff[0]<<8)|rBuff[1];
        }
        else
        {
            retry++;
            if(retry==GYRO_MAX_RETRY)
            {
                printf("\n\tGYROSCOPE: ATTEMPTS TO READ Z OUT");
                break;
            }
        }
    }
    return 0;
}

// READ TEMPERATURE FUNCTION
int Get_Temperature(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=GYRO_TEMP_OUT_H;

    while(retry<GYRO_MAX_RETRY)
    {
        if(G_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,GYRO_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tGYROSCOPE: READ VALUE ERROR");
            }

            return (int)(rBuff[0]<<8)|rBuff[1];
        }
        else
        {
            retry++;
            if(retry==GYRO_MAX_RETRY)
            {
                printf("\n\tGYROSCOPE: ATTEMPTS TO READ TEMPERATURE OUT");
                break;
            }
        }
    }
    return 0;
}


/*********************************************************************
 * ACCELEROMETER FUNCTIONS
 *********************************************************************/
// WRITE VALUE FUNCTION
void Accel_Write_Value(unsigned char reg, unsigned char conf)
{
    unsigned char wBuff = conf;

    wData.buff=&wBuff;
    wData.n=1;
    wData.addr=reg;

    // Read Data
    devicedrv.oData = &wData;
    devicedrv.cmd = DEVICE_WRITE;

    while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
    {
        devicedrv.tick(&devicedrv,ACCEL_ADDRESS);
    }

    if(devicedrv.cmd == DEVICE_ERR)
    {
        printf("\n\tACCELEROMETER: WRITE ERROR");
    }
}

// READ X-Y-Z VALUE FUNCTION
void Accel_Read_XYZValue(int xyz_data[3])
{
    unsigned int retry = 0;
    unsigned char rBuff[6] = {0,0,0,0,0,0};

    rData.buff=rBuff;
    rData.n=6;
    rData.addr=ACCEL_DATAX0;

    while(retry<ACCEL_MAX_RETRY)
    {
        if(A_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,ACCEL_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tACCELEROMETER: READ VALUE ERROR");
            }

            xyz_data[0] = (int)((rBuff[1]<<8)|rBuff[0]);
            xyz_data[1] = (int)((rBuff[3]<<8)|rBuff[2]);
            xyz_data[2] = (int)((rBuff[5]<<8)|rBuff[4]);

            return;
        }
        else
        {
            retry++;
            if(retry==ACCEL_MAX_RETRY)
            {
                printf("\n\tACCELEROMETER: ATTEMPTS TO READ XYZ OUT");
                break;
            }
        }
    }
    return;
}

// READ X VALUE FUNCTION
int Accel_Read_XValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=ACCEL_DATAX0;

    while(retry<ACCEL_MAX_RETRY)
    {
        if(A_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,ACCEL_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tACCELEROMETER: READ VALUE ERROR");
            }

            return (int)(rBuff[1]<<8)|rBuff[0];
        }
        else
        {
            retry++;
            if(retry==ACCEL_MAX_RETRY)
            {
                printf("\n\tACCELEROMETER: ATTEMPTS TO READ X OUT");
                break;
            }
        }
    }
    return 0;
}

// READ Y VALUE FUNCTION
int Accel_Read_YValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=ACCEL_DATAY0;

    while(retry<ACCEL_MAX_RETRY)
    {
        if(A_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,ACCEL_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tACCELEROMETER: READ VALUE ERROR");
            }

            return (int)(rBuff[1]<<8)|rBuff[0];
        }
        else
        {
            retry++;
            if(retry==ACCEL_MAX_RETRY)
            {
                printf("\n\tACCELEROMETER: ATTEMPTS TO READ Y OUT");
                break;
            }
        }
    }
    return 0;
}

// READ Z VALUE FUNCTION
int Accel_Read_ZValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=ACCEL_DATAZ0;

    while(retry<ACCEL_MAX_RETRY)
    {
        if(A_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,ACCEL_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tACCELEROMETER: READ VALUE ERROR");
            }

            return (int)(rBuff[1]<<8)|rBuff[0];
        }
        else
        {
            retry++;
            if(retry==ACCEL_MAX_RETRY)
            {
                printf("\n\tACCELEROMETER: ATTEMPTS TO READ Z OUT");
                break;
            }
        }
    }
    return 0;
}


/*********************************************************************
 * COMPASS FUNCTIONS
 *********************************************************************/
// WRITE VALUE FUNCTION
void Comp_Write_Value(unsigned char reg, unsigned char conf)
{
    unsigned char wBuff = conf;

    wData.buff=&wBuff;
    wData.n=1;
    wData.addr=reg;

    // Write Data
    devicedrv.oData = &wData;
    devicedrv.cmd = DEVICE_WRITE;

    while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
    {
        devicedrv.tick(&devicedrv,COMP_ADDRESS);
    }

    if(devicedrv.cmd == DEVICE_ERR)
    {
        printf("\n\tCOMPASS: WRITE ERROR");
    }
}

// READ X-Y-Z VALUE FUNCTION
void Comp_Read_XYZValue(int xyz_data[3])
{
    unsigned int retry = 0;
    unsigned char rBuff[6] = {0,0,0,0,0,0};

    rData.buff=rBuff;
    rData.n=6;
    rData.addr=COMP_OUT_X_H;

    while(retry<COMP_MAX_RETRY)
    {
        if(C_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,COMP_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tCOMPASS: READ VALUE ERROR");
            }

            xyz_data[0] = (int)(rBuff[0]<<8)|rBuff[1];
            xyz_data[2] = (int)(rBuff[2]<<8)|rBuff[3];
            xyz_data[1] = (int)(rBuff[4]<<8)|rBuff[5];

            return;
        }
        else
        {
            retry++;
            if(retry==COMP_MAX_RETRY)
            {
                printf("\n\tCOMPASS: ATTEMPTS TO READ XYZ OUT");
                break;
            }
        }
    }
    return;
}

// READ X VALUE FUNCTION
int Comp_Read_XValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=COMP_OUT_X_H;

    while(retry<COMP_MAX_RETRY)
    {
        if(C_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,COMP_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tCOMPASS: READ VALUE ERROR");
            }

            return (int)(rBuff[0]<<8)|rBuff[1];
        }
        else
        {
            retry++;
            if(retry==COMP_MAX_RETRY)
            {
                printf("\n\tCOMPASS: ATTEMPTS TO READ X OUT");
                break;
            }
        }
    }
    return 0;
}

// READ Y VALUE FUNCTION
int Comp_Read_YValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=COMP_OUT_Y_H;

    while(retry<COMP_MAX_RETRY)
    {
        if(C_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,COMP_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tCOMPASS: READ VALUE ERROR");
            }

            return (int)(rBuff[0]<<8)|rBuff[1];
        }
        else
        {
            retry++;
            if(retry==COMP_MAX_RETRY)
            {
                printf("\n\tCOMPASS: ATTEMPTS TO READ Y OUT");
                break;
            }
        }
    }
    return 0;
}

// READ Z VALUE FUNCTION
int Comp_Read_ZValue(void)
{
    unsigned int retry = 0;
    unsigned char rBuff[2] = {0,0};

    rData.buff=rBuff;
    rData.n=2;
    rData.addr=COMP_OUT_Z_H;

    while(retry<COMP_MAX_RETRY)
    {
        if(C_DRDY)
        {
            // Read Data
            devicedrv.oData = &rData;
            devicedrv.cmd = DEVICE_READ;

            while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
            {
                devicedrv.tick(&devicedrv,COMP_ADDRESS);
            }

            if(devicedrv.cmd == DEVICE_ERR)
            {
                printf("\n\tCOMPASS: READ VALUE ERROR");
            }

            return (int)(rBuff[0]<<8)|rBuff[1];
        }
        else
        {
            retry++;
            if(retry==COMP_MAX_RETRY)
            {
                printf("\n\tCOMPASS: ATTEMPTS TO READ Z OUT");
                break;
            }
        }
    }
    return 0;
}


/*********************************************************************
 * GY85 FUNCTIONS
 *********************************************************************/
void GY85_Init(void)
{
    //Gyroscope initialization
    Gyro_Write_Value(GYRO_DLPF_FS,0x1C); //Range 2000°/s, 1kHz sample rate and 20Hz Bandwidth
    Gyro_Write_Value(GYRO_SMPLRT_DIV,0x00); //Fsample 1000Hz, Bandwidth 5Hz
    Gyro_Write_Value(GYRO_INT_CFG,0x31); //Interrupt on data ready in latch mode
    Gyro_Write_Value(GYRO_PWR_MGM,0x00); //

    //Accelerometer initialization
    Accel_Write_Value(ACCEL_DATA_FORMAT, 0x0B); //Full resolution, maintain 4 mg/LSB scale factor, 16g range
    Accel_Write_Value(ACCEL_BW_RATE, 0x09); //Output Data Rate 50Hz,  Bandwidth 25Hz
    Accel_Write_Value(ACCEL_INT_ENABLE, 0x80); //Interrupt on Data Ready
    Accel_Write_Value(ACCEL_INT_MAP, 0x00); //Interrupt mapped on INT1 pin
    Accel_Write_Value(ACCEL_POWER_CTL, 0x08); //Mesure Mode ON

    //Compass initialization
    Comp_Write_Value(COMP_CONF_A, 0x18); //Output Data Rate 75Hz
    Comp_Write_Value(COMP_CONF_B, 0x20); //
    Comp_Write_Value(COMP_MODE, 0x00); //
}

void GY85_Read_Rough(int gy85_data_rough[9])
{
    Accel_Read_XYZValue(gy85_data_rough);
    Gyro_Read_XYZValue(gy85_data_rough + 3);
    Comp_Read_XYZValue(gy85_data_rough + 6);
}

void GY85_Read_Real(float gy85_data_real[9])
{
    signed int gy85_data_rough[9];

    GY85_Read_Rough(gy85_data_rough);

    gy85_data_real[0] = ((gy85_data_rough[0] * FACTORACC) * acc_matrix[0][0]) + ((gy85_data_rough[1] * FACTORACC) * acc_matrix[1][0]) + ((gy85_data_rough[2] * FACTORACC) * acc_matrix[2][0]) + acc_matrix[3][0];
    gy85_data_real[1] = ((gy85_data_rough[0] * FACTORACC) * acc_matrix[0][1]) + ((gy85_data_rough[1] * FACTORACC) * acc_matrix[1][1]) + ((gy85_data_rough[2] * FACTORACC) * acc_matrix[2][1]) + acc_matrix[3][1];
    gy85_data_real[2] = ((gy85_data_rough[0] * FACTORACC) * acc_matrix[0][2]) + ((gy85_data_rough[1] * FACTORACC) * acc_matrix[1][2]) + ((gy85_data_rough[2] * FACTORACC) * acc_matrix[2][2]) + acc_matrix[3][2];

    gy85_data_real[3] = (gy85_data_rough[3] * FACTORGYR) + gyr_x_offset;
    gy85_data_real[4] = (gy85_data_rough[4] * FACTORGYR) + gyr_y_offset;
    gy85_data_real[5] = (gy85_data_rough[5] * FACTORGYR) + gyr_z_offset;

    gy85_data_real[6] = ((gy85_data_rough[6] * FACTORMAG) * mag_x_factor) + mag_x_offset;
    gy85_data_real[7] = ((gy85_data_rough[7] * FACTORMAG) * mag_y_factor) + mag_y_offset;
    gy85_data_real[8] = ((gy85_data_rough[8] * FACTORMAG) * mag_z_factor) + mag_z_offset;
}