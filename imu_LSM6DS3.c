/************************************************************************/
// File:			imu_LSM6DS3.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver for LSM6DS3 inertial measurement unit
//                  Rewritten from Arduino library to work for AVR
//           https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
//
/************************************************************************/


#include <avr/io.h>

/*Pin defines*/
#include "defines.h"
#include "imu_LSM6DS3.h"

/* SPI driver */
#include "spi.h"

/* Settings for the IMU */

//Error checking
uint16_t allOnesCounter;
uint16_t nonSuccessCounter;

//This struct holds the settings the driver uses to do calculations
// Set as static to tell compiler it will only be used from this file
static struct SensorSettings {
    //Gyro settings
    uint8_t gyroEnabled;
    uint16_t gyroRange;
    uint16_t gyroSampleRate;
    uint16_t gyroBandWidth;

    uint8_t gyroFifoEnabled;
    uint8_t gyroFifoDecimation;

    //Accelerometer settings
    uint8_t accelEnabled;
    uint8_t accelODROff;
    uint16_t accelRange;
    uint16_t accelSampleRate;
    uint16_t accelBandWidth;
    
    uint8_t accelFifoEnabled;
    uint8_t accelFifoDecimation;
    
    //Temperature settings
    uint8_t tempEnabled;
    
    //Non-basic mode settings
    uint8_t commMode;
    
    //FIFO control data
    uint16_t fifoThreshold;
    int16_t fifoSampleRate;
    uint8_t fifoModeWord;
};


//Instantiate IMU settings
struct SensorSettings settings;

// IMU init function
// Set as static to tell compiler it will only be used from this file
status_t sIMU_Init(){
    //Set settings
    settings.gyroEnabled = 1;   //Can be 0 or 1
    settings.gyroRange = 500;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
    settings.gyroSampleRate = 1666;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
    settings.gyroBandWidth = 400;  //Hz.  Can be: 50, 100, 200, 400;
    settings.gyroFifoEnabled = 0;  //Set to include gyro in FIFO
    settings.gyroFifoDecimation = 0;  //set 1 for on /1

    settings.accelEnabled = 1;
    settings.accelODROff = 1;
    settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
    settings.accelSampleRate = 13;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
    settings.accelBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;
    settings.accelFifoEnabled = 0;  //Set to include accelerometer in the FIFO
    settings.accelFifoDecimation = 0;  //set 1 for on /1

    settings.tempEnabled = 1;

    //Select interface mode
    settings.commMode = 1;  //Can be modes 1, 2 or 3

    //FIFO control data
    settings.fifoThreshold = 3000;  //Can be 0 to 4096 (16 bit bytes)
    settings.fifoSampleRate = 10;  //default 10Hz
    settings.fifoModeWord = 0;  //Default off

    allOnesCounter = 0;
    nonSuccessCounter = 0;
    status_t returnError = IMU_SUCCESS;
    /* Initialize SPI */
    vSPI_MasterInit();
    /* Set chip select pin output, high */
    DDR_SPI |= (1<<IMU_SS);
    PORTB |= (1<<IMU_SS);
    
    // Busy wait for a few ms to let IMU initialize
    volatile uint8_t temp = 0;
    for( uint16_t i = 0; i < 10000; i++ ){
        temp++;
    }
    //Check the ID register to determine if the operation was a success.
    uint8_t readCheck;
    do{
        sIMU_readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
    } while (readCheck != 0x69);
    
    
    return returnError;
}

//****************************************************************************//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t sIMU_readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length){
    status_t returnError = IMU_SUCCESS;

    //define pointer that will point to the external space
    uint8_t i = 0;
    uint8_t c = 0;
    uint8_t tempFFCounter = 0;


    // take the chip select low to select the device:
    PORTB &= ~(1<<IMU_SS);

    // send the device the register you want to read:
    ui8SPI_MasterTransmit(offset | 0x80);  //Ored with "read request" bit
    
    while ( i < length ){ // slave may send less than requested
        c = ui8SPI_MasterTransmit(0x00); // receive a byte as character
        if( c == 0xFF ){
            //May have problem
            tempFFCounter++;
        }
        *outputPointer = c;
        outputPointer++;
        i++;
    }
    if( tempFFCounter == i ){
        //Ok, we've recieved all ones, report
        returnError = IMU_ALL_ONES_WARNING;
    }
    // take the chip select high to de-select:
    PORTB |= (1<<IMU_SS);
    return returnError;
}

//****************************************************************************//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t sIMU_readRegister(uint8_t* outputPointer, uint8_t offset ){
    //Return value
    uint8_t result;
    status_t returnError = IMU_SUCCESS;

    // take the chip select low to select the device:
    PORTB &= ~(1<<IMU_SS);
    // send the device the register you want to read:
    ui8SPI_MasterTransmit(offset | 0x80);  //Ored with "read request" bit
    // send a value of 0 to read the first byte returned:
    result = ui8SPI_MasterTransmit(0x00);
    // take the chip select high to de-select:
    PORTB |= (1<<IMU_SS);
    
    if( result == 0xFF ){
        //we've recieved all ones, report
        returnError = IMU_ALL_ONES_WARNING;
    }
    *outputPointer = result;
    return returnError;
}



//****************************************************************************//
//
//  sIMU_readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t sIMU_readRegisterInt16( int16_t* outputPointer, uint8_t offset ){
    uint8_t myBuffer[2];
    status_t returnError = sIMU_readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
    // Changed 	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
    int16_t tempBuffer = (myBuffer[1] << 8);
    int16_t output = (int16_t)myBuffer[0] | tempBuffer;
    
    *outputPointer = output;
    return returnError;
}

//****************************************************************************//
//
//  sIMU_writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t sIMU_writeRegister(uint8_t offset, uint8_t dataToWrite ){
    status_t returnError = IMU_SUCCESS;

    // take the chip select low to select the device:
    PORTB &= ~(1<<IMU_SS);
    // send the device the register you want to read:
    ui8SPI_MasterTransmit(offset);
    // send a value of 0 to read the first byte returned:
    ui8SPI_MasterTransmit(dataToWrite);
    // decrement the number of bytes left to read:
    // take the chip select high to de-select:
    PORTB |= (1<<IMU_SS);
    return returnError;
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as 
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
status_t sIMU_begin(){
    //Check the settings structure values to determine how to setup the device
    uint8_t dataToWrite = 0;  //Temporary variable

    //Initialize the IMU and the SPI driver
    status_t returnError = sIMU_Init();

    //Setup the accelerometer******************************
    dataToWrite = 0; //Start Fresh!
    if ( settings.accelEnabled == 1) {
        //Build config reg
        //First patch in filter bandwidth
        switch (settings.accelBandWidth) {
            case 50:
            dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
            break;
            case 100:
            dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
            break;
            case 200:
            dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
            break;
            default:  //set default case to max passthrough
            case 400:
            dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
            break;
        }
        //Next, patch in full scale
        switch (settings.accelRange) {
            case 2:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
            break;
            case 4:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
            break;
            case 8:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
            break;
            default:  //set default case to 16(max)
            case 16:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
            break;
        }
        //Lastly, patch in accelerometer ODR
        switch (settings.accelSampleRate) {
            case 13:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
            break;
            case 26:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
            break;
            case 52:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
            break;
            default:  //Set default to 104
            case 104:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
            break;
            case 208:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
            break;
            case 416:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
            break;
            case 833:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
            break;
            case 1660:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
            break;
            case 3330:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
            break;
            case 6660:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
            break;
            case 13330:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
            break;
        }
    }
    else
    {
        //dataToWrite already = 0 (powerdown);
    }

    //Now, write the patched together data
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

    //Set the ODR bit
    sIMU_readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
    dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
    if ( settings.accelODROff == 1) {
        dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
    }
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);

    //Setup the gyroscope**********************************************
    dataToWrite = 0; //Start Fresh!
    if ( settings.gyroEnabled == 1) {
        //Build config reg
        //First, patch in full scale
        switch (settings.gyroRange) {
            case 125:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
            break;
            case 245:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
            break;
            case 500:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
            break;
            case 1000:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
            break;
            default:  //Default to full 2000DPS range
            case 2000:
            dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
            break;
        }
        //Lastly, patch in gyro ODR
        switch (settings.gyroSampleRate) {
            case 13:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
            break;
            case 26:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
            break;
            case 52:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
            break;
            default:  //Set default to 104
            case 104:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
            break;
            case 208:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
            break;
            case 416:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
            break;
            case 833:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
            break;
            case 1660:
            dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
            break;
        }
    }
    else{
        //dataToWrite already = 0 (powerdown);
    }
    //Write the byte
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

    //Setup the internal temperature sensor
    if ( settings.tempEnabled == 1) {
    }

    //Return WHO AM I reg
    uint8_t result;
    sIMU_readRegister(&result, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

    return returnError;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t i16IMU_readRawAccelX( void )
{
    int16_t output;
    status_t errorLevel = sIMU_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_XL );
    if( errorLevel != IMU_SUCCESS ){
        if( errorLevel == IMU_ALL_ONES_WARNING ){
            allOnesCounter++;
        }
        else{
            nonSuccessCounter++;
        }
    }
    return output;
}
float fIMU_readFloatAccelX( void )
{
    float output = fIMU_calcAccel(i16IMU_readRawAccelX());
    return output;
}

int16_t i16IMU_readRawAccelY( void )
{
    int16_t output;
    status_t errorLevel = sIMU_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_XL );
    if( errorLevel != IMU_SUCCESS )
    {
        if( errorLevel == IMU_ALL_ONES_WARNING )
        {
            allOnesCounter++;
        }
        else
        {
            nonSuccessCounter++;
        }
    }
    return output;
}
float fIMU_readFloatAccelY( void )
{
    float output = fIMU_calcAccel(i16IMU_readRawAccelY());
    return output;
}

int16_t i16IMU_readRawAccelZ( void )
{
    int16_t output;
    status_t errorLevel = sIMU_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_XL );
    if( errorLevel != IMU_SUCCESS )
    {
        if( errorLevel == IMU_ALL_ONES_WARNING )
        {
            allOnesCounter++;
        }
        else
        {
            nonSuccessCounter++;
        }
    }
    return output;
}

float fIMU_readFloatAccelZ( void ){
    float output = fIMU_calcAccel(i16IMU_readRawAccelZ());
    return output;
}

float fIMU_calcAccel( int16_t input ){
    float output = (float)input * 0.061 * (settings.accelRange >> 1) / 1000;
    return output;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
int16_t i16IMU_readRawGyroX( void ){
    int16_t output;
    status_t errorLevel = sIMU_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_G );
    if( errorLevel != IMU_SUCCESS ){
        if( errorLevel == IMU_ALL_ONES_WARNING ){
            allOnesCounter++;
        }
        else{
            nonSuccessCounter++;
        }
    }
    return output;
}

float fIMU_readFloatGyroX( void ){
    float output = fIMU_calcGyro(i16IMU_readRawGyroX());
    return output;
}

int16_t i16IMU_readRawGyroY( void ){
    int16_t output;
    status_t errorLevel = sIMU_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_G );
    if( errorLevel != IMU_SUCCESS ){
        if( errorLevel == IMU_ALL_ONES_WARNING ){
            allOnesCounter++;
        }
        else{
            nonSuccessCounter++;
        }
    }
    return output;
}
float fIMU_readFloatGyroY( void ){
    
    float output = fIMU_calcGyro(i16IMU_readRawGyroY());
    return output;
}

int16_t i16IMU_readRawGyroZ( void ){
    int16_t output;
    status_t errorLevel = sIMU_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_G );
    if( errorLevel != IMU_SUCCESS ){
        if( errorLevel == IMU_ALL_ONES_WARNING ){
            allOnesCounter++;
        }
        else{
            nonSuccessCounter++;
        }
    }
    
    return output;
}
float fIMU_readFloatGyroZ(){
    float output = fIMU_calcGyro(i16IMU_readRawGyroZ());
    
    return output;
}

float fIMU_calcGyro( int16_t input ){
    uint8_t gyroRangeDivisor = settings.gyroRange / 125;
    if ( settings.gyroRange == 245 ) {
        gyroRangeDivisor = 2;
    }
    
    float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;
    return output;
}


//****************************************************************************//
//
//  FIFO section
//
//****************************************************************************//
void vIMU_fifoBegin(){
    //CONFIGURE THE VARIOUS FIFO SETTINGS
    //
    //
    //This section first builds a bunch of config words, then goes through
    //and writes them all.

    //Split and mask the threshold
    uint8_t thresholdLByte = settings.fifoThreshold & 0x00FF;
    uint8_t thresholdHByte = (settings.fifoThreshold & 0x0F00) >> 8;
    //Pedo bits not configured (ctl2)

    //CONFIGURE FIFO_CTRL3
    uint8_t tempFIFO_CTRL3 = 0;
    if (settings.gyroFifoEnabled == 1){
        //Set up gyro stuff
        //Build on FIFO_CTRL3
        //Set decimation
        tempFIFO_CTRL3 |= (settings.gyroFifoDecimation & 0x07) << 3;

    }
    if (settings.accelFifoEnabled == 1){
        //Set up accelerometer stuff
        //Build on FIFO_CTRL3
        //Set decimation
        tempFIFO_CTRL3 |= (settings.accelFifoDecimation & 0x07);
    }

    //CONFIGURE FIFO_CTRL4  (nothing for now-- sets data sets 3 and 4
    uint8_t tempFIFO_CTRL4 = 0;


    //CONFIGURE FIFO_CTRL5
    uint8_t tempFIFO_CTRL5 = 0;
    switch (settings.fifoSampleRate) {
        default:  //set default case to 10Hz(slowest)
        case 10:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz;
        break;
        case 25:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz;
        break;
        case 50:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz;
        break;
        case 100:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz;
        break;
        case 200:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz;
        break;
        case 400:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz;
        break;
        case 800:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz;
        break;
        case 1600:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz;
        break;
        case 3300:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz;
        break;
        case 6600:
        tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz;
        break;
    }
    //Hard code the fifo mode here:
    tempFIFO_CTRL5 |= settings.fifoModeWord = 6;  //set mode:

    //Write the data
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL2, thresholdHByte);
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

}
void vIMU_fifoClear(){
    //Drain the fifo data and dump it
    while( (ui16IMU_fifoGetStatus() & 0x1000 ) == 0 ) {
        i16IMU_fifoRead();
    }
}

int16_t i16IMU_fifoRead(){
    //Pull the last data from the fifo
    uint8_t tempReadByte = 0;
    uint16_t tempAccumulator = 0;
    sIMU_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);
    tempAccumulator = tempReadByte;
    sIMU_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);
    tempAccumulator |= ((uint16_t)tempReadByte << 8);

    return tempAccumulator;
}

uint16_t ui16IMU_fifoGetStatus(){
    //Return some data on the state of the fifo
    uint8_t tempReadByte = 0;
    uint16_t tempAccumulator = 0;
    sIMU_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS1);
    tempAccumulator = tempReadByte;
    sIMU_readRegister(&tempReadByte, LSM6DS3_ACC_GYRO_FIFO_STATUS2);
    tempAccumulator |= (tempReadByte << 8);

    return tempAccumulator;

}
void vIMU_fifoEnd(){
    // turn off the fifo
    sIMU_writeRegister(LSM6DS3_ACC_GYRO_FIFO_STATUS1, 0x00);  //Disable
}
