/*
 * File:   MPU9250.c
 * Author: Andrea Verdecchia
 *
 * Created on 30 aprile 2015, 16.35
 */

#include "main.h"

//The "Done" variable should be used in the Interrupt routine of I2C module.
unsigned int Done;
float acc_matrix[4][3] = {{1,0,0},{0,1,0},{0,0,1},{0,0,0}};
float gyr_x_offset = 0, gyr_y_offset = 0, gyr_z_offset = 0, mag_x_factor = 1, mag_y_factor = 1, mag_z_factor = 1, mag_x_offset = 0, mag_y_offset = 0, mag_z_offset = 0;

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
            //printf("\ncase 0");
            if((device->cmd == DEVICE_WRITE)||(device->cmd == DEVICE_READ))
		state=1;
            break;

	/************************************
	 * Control/Address Phase
	 ************************************/
        case 1:
            // Start Condition
            //printf("\ncase 1");
            I2C1CONbits.SEN=1;
            state=state+1;
            break;

        case 2:
            // Start Byte with device select ID
            //printf("\ncase 2");
	    if(Done==1)
            {
		Done=0;
          	state=state+1;
  		I2C1TRN=(DEVICE_ADDRESS<<1);
            }
            break;

        case 3:
            // Send address byte of register, if ack is received. Else Retry
            //printf("\ncase 3");
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
            //printf("\ncase 4");
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
            //printf("\ncase 5");
            I2C1TRN=*(device->oData->buff + cntr);
            state=state+1;
            cntr=cntr+1;
            break;

        case 6:
            // Look for end of data or no Ack
            //printf("\ncase 6");
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
            //printf("\ncase 7");
            I2C1CONbits.RSEN=1;
            state=state+1;
            break;

        case 8:
            // Re-send ID byte with W/R=R
            //printf("\ncase 8");
	    if(Done==1)
            {
		Done=0;
          	state=state+1;
                I2C1TRN=((DEVICE_ADDRESS)<<1)|(0x0001);
            }
            break;

        case 9:
            // Check, if control byte went ok
            //printf("\ncase 9");
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
            //printf("\ncase 10");
            I2C1CONbits.RCEN=1;
            state++;
            break;

        case 11:
            // Receive data
            //printf("\ncase 11");
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
            //printf("\ncase 12");
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
            //printf("\ncase 13");
            I2C1CONbits.PEN=1;
            state++;
            break;

        case 14:
            //printf("\ncase 14");
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
            //printf("\ncase 15");
            I2C1CONbits.PEN=1;
            state++;
            break;

        case 16:
            //printf("\ncase 16");
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
            //printf("\ncase 17");
            I2C1CONbits.PEN=1;
            state++;
            rtrycntr++;
            break;

        case 18:
            //printf("\ncase 18");
 	    if(Done==1)
            {
		Done=0;
		state=0;
		cntr=0;
            }
            break;
    }
}

int I2C_write_byte(unsigned char device_add, unsigned char reg, unsigned char conf)
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
        devicedrv.tick(&devicedrv,device_add);
    }

    if(devicedrv.cmd == DEVICE_ERR)
        return 1;
    else
        return 0;
}

int I2C_write(unsigned char device_add, unsigned char reg, unsigned char *conf, unsigned char num_byte)
{
    wData.buff=conf;
    wData.n=num_byte;
    wData.addr=reg;

    // Write Data
    devicedrv.oData = &wData;
    devicedrv.cmd = DEVICE_WRITE;

    while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
    {
        devicedrv.tick(&devicedrv,device_add);
    }

    if(devicedrv.cmd == DEVICE_ERR)
        return 1;
    else
        return 0;
}

int I2C_read_byte(unsigned char device_add, unsigned char reg, unsigned char *data)
{
    rData.buff=data;
    rData.n=1;
    rData.addr=reg;

    // Read Data
    devicedrv.oData = &rData;
    devicedrv.cmd = DEVICE_READ;

    while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
    {
        devicedrv.tick(&devicedrv,device_add);
    }

    if(devicedrv.cmd == DEVICE_ERR)
        return 1;
    else
        return 0;
}

int I2C_read(unsigned char device_add, unsigned char reg, unsigned char *data, unsigned char num_byte)
{
    rData.buff = data;
    rData.n = num_byte;
    rData.addr = reg;

    // Read Data
    devicedrv.oData = &rData;
    devicedrv.cmd = DEVICE_READ;

    while((devicedrv.cmd != DEVICE_IDLE)&&(devicedrv.cmd != DEVICE_ERR))
    {
        devicedrv.tick(&devicedrv,device_add);
    }

    if(devicedrv.cmd == DEVICE_ERR)
        return 1;
    else
        return 0;
}

/*********************************************************************
 * MPU9250 FUNCTIONS
 *********************************************************************/

// WRITE TO MPU9250 FUNCTION
void MPU9250_write_reg(unsigned char reg, unsigned char conf)
{
    if(I2C_write_byte(MPU9250_ADD, reg, conf))
        printf("\n\tMPU9250_Write(): WRITE ERROR");

    return;
}

// WRITE TO AK8963 FUNCTION
void AK8963_write_reg(unsigned char reg, unsigned char conf)
{
    if(I2C_write_byte(AK8963_ADD, reg, conf))
        printf("\n\tAK8963_Write(): WRITE ERROR");

    return;
}

// READ MEASURE DATA FUNCTION
void MPU9250_Read_Measure_Rough(int measure_data[10])
{
    unsigned int retry = 0;
    unsigned char rBuff[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//    MPU9250_write_reg(I2C_SLV0_ADDR,AK8963_ADD|0x80); //Set the I2C slave addres of AK8963 and set for read.
//    MPU9250_write_reg(I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
//    MPU9250_write_reg(I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer
//    __delay_ms(1);

    while(retry<RW_MAX_RETRY)
    {
        if(DATA_RDY)
        {
            if(I2C_read(MPU9250_ADD, ACCEL_XOUT_H, rBuff, 20))
                printf("\n\tMPU9250_Read_Measure_Rough(): READ ERROR");

            // Data format: measure_data = { acc x | acc y | acc z | gyr x | gyr y | gyr z | mag x | mag y | mag z | temp }
            measure_data[0] = ((int)(rBuff[0]<<8)|rBuff[1]);
            measure_data[1] = ((int)(rBuff[2]<<8)|rBuff[3]);
            measure_data[2] = ((int)(rBuff[4]<<8)|rBuff[5]);
            measure_data[3] = ((int)(rBuff[8]<<8)|rBuff[9]);
            measure_data[4] = ((int)(rBuff[10]<<8)|rBuff[11]);
            measure_data[5] = ((int)(rBuff[12]<<8)|rBuff[13]);
            measure_data[6] = ((int)(rBuff[15]<<8)|rBuff[14]);
            measure_data[7] = ((int)(rBuff[17]<<8)|rBuff[16]);
            measure_data[8] = ((int)(rBuff[19]<<8)|rBuff[18]);
            measure_data[9] = ((int)(rBuff[6]<<8)|rBuff[7]);

            return;
        }
        else
        {
            retry++;
            if(retry==RW_MAX_RETRY)
            {
                printf("\n\tMPU9250: ATTEMPTS TO READ OUT");
                break;
            }
        }
    }
    return;
}

void MPU9250_Read_Measure_Real(float measure_data[10])
{
    signed int data_rough[10];

    MPU9250_Read_Measure_Rough(data_rough);

    measure_data[0] = ((data_rough[0] * FACTORACC) * acc_matrix[0][0]) + ((data_rough[1] * FACTORACC) * acc_matrix[1][0]) + ((data_rough[2] * FACTORACC) * acc_matrix[2][0]) + acc_matrix[3][0];
    measure_data[1] = ((data_rough[0] * FACTORACC) * acc_matrix[0][1]) + ((data_rough[1] * FACTORACC) * acc_matrix[1][1]) + ((data_rough[2] * FACTORACC) * acc_matrix[2][1]) + acc_matrix[3][1];
    measure_data[2] = ((data_rough[0] * FACTORACC) * acc_matrix[0][2]) + ((data_rough[1] * FACTORACC) * acc_matrix[1][2]) + ((data_rough[2] * FACTORACC) * acc_matrix[2][2]) + acc_matrix[3][2];

    measure_data[3] = (data_rough[3] * FACTORGYR) + gyr_x_offset;
    measure_data[4] = (data_rough[4] * FACTORGYR) + gyr_y_offset;
    measure_data[5] = (data_rough[5] * FACTORGYR) + gyr_z_offset;

    measure_data[6] = ((data_rough[6] * FACTORMAG) * mag_x_factor) + mag_x_offset;
    measure_data[7] = ((data_rough[7] * FACTORMAG) * mag_y_factor) + mag_y_offset;
    measure_data[8] = ((data_rough[8] * FACTORMAG) * mag_z_factor) + mag_z_offset;

    measure_data[9] = data_rough[9] * FACTORTEM;

}

void MPU9250_Init(void)
{
//    MPU9250_write_reg(PWR_MGMT_1, 0x80); //Reset Device
//    __delay_ms(200);
//    MPU9250_write_reg(PWR_MGMT_1, 0x01); //Clock Source
//    __delay_ms(200);
//    MPU9250_write_reg(INT_PIN_CFG, 0x02);
//    __delay_ms(10);
    //AK8963_write_reg(AK8963_CNTL1, 0x01);
    //__delay_ms(10);

//    AK8963_write_reg(AK8963_CNTL2, 0x01); //Set magnetometer to 16-bit mode and ContinuousMeasurement mode 2 (100Hz)
//    __delay_ms(1);
//    AK8963_write_reg(AK8963_CNTL1, 0x16); //Set magnetometer to 16-bit mode and ContinuousMeasurement mode 2 (100Hz)
//    __delay_ms(1);

    MPU9250_write_reg(PWR_MGMT_1, 0x80); //Reset Device
    __delay_ms(200);
    MPU9250_write_reg(PWR_MGMT_1, 0x01); //Clock Source
    __delay_ms(200);
    MPU9250_write_reg(PWR_MGMT_2, 0x00); //Enable Acc and Gyro
    __delay_ms(10);
    MPU9250_write_reg(SMPLRT_DIV, 0x01); //SAMPLE RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV) set to 500Hz
    __delay_ms(10);
    MPU9250_write_reg(CONFIG, 0x01); //Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
    __delay_ms(10);
    MPU9250_write_reg(GYRO_CONFIG, 0x18); //Gyro Full Scale = 2000dps, Fchoice_b = '00'
    __delay_ms(10);
    MPU9250_write_reg(ACCEL_CONFIG, 0x18); //Accel Full Scale = 16g
    __delay_ms(10);
    MPU9250_write_reg(ACCEL_CONFIG_2, 0x01); //DLPF set Accel bandwidth 184Hz, accel_fchoice_b = '0'
    __delay_ms(10);
    MPU9250_write_reg(INT_PIN_CFG, 0x32);
    __delay_ms(10);
    MPU9250_write_reg(INT_ENABLE, 0x01);
    __delay_ms(100);

//    MPU9250_write_reg(USER_CTRL, 0x20);
//    __delay_ms(1);
//    MPU9250_write_reg(I2C_MST_CTRL, 0x4D);
//    __delay_ms(1);

//    AK8963_write_reg(AK8963_CNTL2, 0x01); //Set magnetometer to 16-bit mode and ContinuousMeasurement mode 2 (100Hz)
//    __delay_ms(1);
//    AK8963_write_reg(AK8963_CNTL1, 0x16); //Set magnetometer to 16-bit mode and ContinuousMeasurement mode 2 (100Hz)
//    __delay_ms(1);

//    MPU9250_write_reg(I2C_SLV0_ADDR, AK8963_ADD);
//    __delay_ms(1);
//    MPU9250_write_reg(I2C_SLV0_REG, AK8963_CNTL2);
//    __delay_ms(1);
//    MPU9250_write_reg(I2C_SLV0_DO, 0x01); //Reset AK8963
//    __delay_ms(1);
//    MPU9250_write_reg(I2C_SLV0_CTRL, 0x81);
//    __delay_ms(1);
//    MPU9250_write_reg(I2C_SLV0_REG, AK8963_CNTL1);
//    __delay_ms(1);
//    MPU9250_write_reg(I2C_SLV0_DO, 0x16); //Set AK8963 in 16-bit mode and Continuous Measurement 2 mode
//    __delay_ms(1);
//    MPU9250_write_reg(I2C_SLV0_CTRL, 0x81);


//    // MPU9250 initializzation
//    MPU9250_write_reg(SMPLRT_DIV, 0x09); //SAMPLE RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV) set to 100Hz
//    MPU9250_write_reg(CONFIG, 0x01); //FSYNC disabled, replace old data in FIFO whe new is available, DLPF bw = 184Hz
//    MPU9250_write_reg(GYRO_CONFIG, 0x18); //Gyro Full Scale = 2000dps, Fchoice_b = 0b00
//    MPU9250_write_reg(ACCEL_CONFIG, 0x18); //Accel Full Scale = 16g
//    MPU9250_write_reg(ACCEL_CONFIG_2, 0x01); //Accel_fchoice = 1, A_DLPF bw = 184Hz
//    MPU9250_write_reg(I2C_SLV0_ADDR, 0x8C); //Transfer is a read, Address slave0 is 0x0C
//    MPU9250_write_reg(I2C_SLV0_REG, AK8963_HXL); //Slave 0 register address from where to begin data transfer
//    MPU9250_write_reg(I2C_SLV0_CTRL, 0xD6); //Enable reading data from slave0, Swap bytes, 6-Byte to read
//    MPU9250_write_reg(INT_PIN_CFG, 0x30);
//
//    MPU9250_Mag_Write(AK8963_CNTL1, 0x16); //Set magnetometer to 16-bit mode and ContinuousMeasurement mode 2 (100Hz)
//
//    MPU9250_write_reg(INT_PIN_CFG, 0x30);
//    MPU9250_write_reg(INT_ENABLE, 0x01);
//    MPU9250_write_reg(USER_CTRL, 0x20);
//    MPU9250_write_reg(PWR_MGMT_1, 0x00);
//    MPU9250_write_reg(I2C_MST_CTRL, 0xCD);
    
}