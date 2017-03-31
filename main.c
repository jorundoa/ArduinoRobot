/************************************************************************/
// File:			main.c
// Author:			Erlend Ese, NTNU Spring 2016
//                  Credit is given where credit is due.
// Purpose:
// AVR Robot with FreeRTOS implementation in the collaborating robots
// project.
// In FreeRTOS each thread of execution is known as tasks
// and are written as C functions
//
// MCU: Arduino Mega with ATmega2560, programmed with USB
//
// TASKS IMPLEMENTED
// Communication:               vMainCommunicationTask
// Sensors:                     vMainSensorTowerTask
// Motor control                vMainMovementTask
// Robot control:               vMainPoseControllerTask
// Position estimator:          vMainPoseEstimatorTask
// Stack overflow handling:     vApplicationStackOverflowHook
//
// See FreeRTOSConfig.h for scheduler settings
// See defines.h for all definitions
//
// GLOBAL VARIABLES:
// See line 84
//
// HARDWARE SETUP
//  Servo pin:  Port B pin 5
//  Sensor pins: Port F pin 0 - 4
//  Motor pins: Port C, pin 5 & 7 and Port H pin 3. Port A pin 2 & 4 and Port E pin 3.
//  Encoder pins: Port D pin 2 & 3
//
// TIMERS USED:
//  Timer0  motorRightPWM   (OCR0B)
//  Timer0  motorLeftPWM    (OCR0A)
//  Timer1  ServoPWM        (OCR1A)
//  Timer3 FreeRTOS time slicer
//
// Interrupt routines located after main function
/************************************************************************/

/* KERNEL INCLUDES */
#include "FreeRTOS.h" /* Must come first */
#include "task.h"     /* RTOS task related API prototypes */
#include "semphr.h"   /* Semaphore related API prototypes */
#include "queue.h"    /* RTOS queue related API prototypes */

/* AVR INCLUDES    */
#include <stdlib.h>             // For itoa();
#include <string.h>             // For stringstuff
#include <util/atomic.h>        // For atomic operation
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

/* Semaphore handles */
SemaphoreHandle_t xScanLock;
SemaphoreHandle_t xPoseMutex;
SemaphoreHandle_t xUartMutex;
SemaphoreHandle_t xTickMutex;
SemaphoreHandle_t xControllerBSem;
SemaphoreHandle_t xCommandReadyBSem;

/* Queues */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t actuationQ = 0;

/* CUSTOM LIBRARIES    */
#include "defines.h"
#include "LED.h"
#include "servo.h"
#include "motor.h"
#include "distSens_g2d12.h"
#include "usart.h"
#include "spi.h"
#include "twi_master.h"
#include "imu_LSM6DS3.h"
#include "com_HMC5883L.h"
#include "functions.h"

/* GLOBAL VARIABLES */
// To store ticks from encoder, changed in ISR and motor controller
volatile uint8_t gISR_rightWheelTicks = 0;
volatile uint8_t gISR_leftWheelTicks = 0;

// Flag to indicate connection status. Interrupt can change handshook status
volatile uint8_t gHandshook = FALSE;
volatile uint8_t gPaused = FALSE;
volatile unsigned char gData_in[20]; // Input buffer for UART
volatile uint8_t gStoreString = FALSE; // Flag to process UART data
volatile unsigned char gData_count; // Counter for UART input buffer

// Global robot pose
float gTheta_hat = 0;
int16_t gX_hat = 0;
int16_t gY_hat = 0;

// Global encoder tick values, could probably be replaced by a queue
volatile int16_t gRightWheelTicks = 0;
volatile int16_t gLeftWheelTicks = 0;

/* STRUCTURE */
struct sPolar{
    float heading;
    int16_t distance;
};

/*#define DEBUG*/

#ifdef DEBUG
#warning DEBUG IS ACTIVE
#endif

/*#define tictoc*/
#ifdef tictoc
    // Pin for timing tasks, use tic/toc - PINH5 (Arduino 8) is available
    #define usetictoc DDRH |= (1<<PINH5)
    #define tic PORTH |= (1<<PINH5)
    #define toc PORTH &= ~(1<<PINH5)
#endif

/*  Communication task */
void vMainCommunicationTask( void *pvParameters ){
    // Setup for the communication task
    struct sPolar Setpoint = {0}; // Struct for setpoints from server
    uint16_t handShakeCounter = 0;
    char command_in[20]; // Buffer for recieved messages
    if (nRFconnected){
        // indicate we are connected at initalization
        vLED_singleHigh(ledGREEN);
        // And not handshook
        vLED_singleHigh(ledYELLOW);
    }
    #ifdef DEBUG
    printf("Communication OK\n");
    #endif

    while(1){
        if(gHandshook){
            if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY) == pdTRUE){
                // We have a new command from the server, copy it to the memory
                vTaskSuspendAll ();       // Temporarily disable context switching
                ATOMIC_BLOCK(ATOMIC_FORCEON){
                    // Copy contents of data_in into command_in
                    memcpy(command_in, &gData_in, sizeof(gData_in));
                }
                xTaskResumeAll ();      // Enable context switching

                // Process recieved USART commands
                if (strstr(command_in, "{}")){
                vUSART_sendS("\t\n");
            }
            if (strstr(command_in, UPDATE)){
                // Update command
                vFunc_ParseUpdate(command_in, &Setpoint.heading, &Setpoint.distance);
                // Ensure max values are not exceeded
                if (Setpoint.distance > 320){
                    Setpoint.distance = 320;
                }
                else if (Setpoint.distance < -320){
                    Setpoint.distance = -320;
                }
                Setpoint.heading *= DEG2RAD; // Convert received set point to radians
                vFunc_Inf2pi(&Setpoint.heading);
                
                /* Relay new coordinates to position controller*/
                /* Wait 500 ms to allow compass to converge if necessary (Experimental) */
                 vTaskDelay(500 / portTICK_PERIOD_MS); // Commented back in due to fixed encoder
                
                xQueueSend(poseControllerQ, &Setpoint, 100);
            }
            else if(strstr(command_in, STATUS_PAUSE)){
                // Stop sending update messages
                #ifdef DEBUG
                    printf("Robot paused!\n");
                #endif
                gPaused = TRUE;
                vLED_singleHigh(ledYELLOW);
                // Stop controller
                Setpoint.distance = 0;
                Setpoint.heading = 0;
                xQueueSend(poseControllerQ, &Setpoint, 100);
            }
            else if(strstr(command_in, STATUS_UNPAUSE)){
                // Robot is finished
                #ifdef DEBUG
                    printf("Robot unpaused!\n");
                #endif
                vLED_singleLow(ledYELLOW);
                gPaused = FALSE;
                // Stop sending update messages
            }
        // Command is processed
        } // if (gCommand_ready) end
    }// if(ghandshook) end
    else if ((nRFconnected == TRUE) && (gHandshook == FALSE)){
        if (xSemaphoreTake(xCommandReadyBSem, 0) == pdTRUE){
            // We have a new command from the server, copy it to the memory
            vTaskSuspendAll ();       // Temporarily disable context switching
            ATOMIC_BLOCK(ATOMIC_FORCEON){
                // Copy contents of data_in into command_in
                memcpy(command_in, &gData_in, sizeof(gData_in));
            }
            xTaskResumeAll ();      // Enable context switching
            if(strstr(command_in, STATUS_CONFIRM)){
                // Handshake is verified!
                #ifdef DEBUG
                printf("{S,CON}\n");
                #endif
                gHandshook = TRUE; // Set start flag true
                vLED_singleLow(ledYELLOW);
            }
        }
        // Send a handshake every 1 sec:
        vLED_singleHigh(ledYELLOW);
            
        if (handShakeCounter >= 50){
            vLED_toggle(ledYELLOW);
        }
        if (handShakeCounter >= 1000){
            vLED_singleHigh(ledGREEN);
            vUSART_sendHandshake();
            vLED_toggle(ledYELLOW);
            handShakeCounter = 0;
        }
        else{
            handShakeCounter++;
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    else if (!(nRFconnected)){
        // Show on leds that we are advertising
        vLED_singleHigh(ledGREEN);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        vLED_singleLow(ledGREEN);
        vTaskDelay(1800 / portTICK_PERIOD_MS);
    }
    }// While(1) end
}// vMainComtask end

/*  Sensor tower task */
void vMainSensorTowerTask( void *pvParameters){
    /* Task init */
    #ifdef DEBUG
        printf("Tower OK\n");
    #endif 
        
    float thetahat = 0;
    int16_t xhat = 0;
    int16_t yhat = 0;
    
    uint8_t rotationDirection = moveCounterClockwise;
    uint8_t servoStep = 0;
    uint8_t servoResolution = 1;
    uint8_t robotMovement = moveStop;
    
    uint8_t idleCounter = 0;
    int16_t previous_left = 0;
    int16_t previous_right = 0;
    // Initialize the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    
    while(1){
        // Loop
        if ((gHandshook == TRUE) && (gPaused == FALSE)){
            // xLastWakeTime variable with the current time.
            xLastWakeTime = xTaskGetTickCount();
            // Set scanning resolution depending on which movement the robot is executing.
            if (xQueueReceive(scanStatusQ, &robotMovement,150 / portTICK_PERIOD_MS) == pdTRUE){
                // Set servo step length according to movement, 
                // Note that the iterations are skipped while robot is rotating (see further downbelow)
                switch (robotMovement)
                {
                    case moveStop:
                        servoStep *= servoResolution;
                        servoResolution = 1;
                        idleCounter = 1;
                    break;
                    case moveForward:
                    case moveBackward:
                        servoResolution = 5;
                        servoStep /= servoResolution;
                        idleCounter = 0;
                    break;
                    case moveClockwise:
                    case moveCounterClockwise:
                        // Iterations are frozen while rotating, see further down
                        idleCounter = 0;
                    break;
                    default:
                        idleCounter = 0;
                    break;
                }
            }
            vServo_setAngle(servoStep*servoResolution);
            vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_PERIOD_MS); // Wait total of 200 ms for servo to reach set point
            
            uint8_t forwardSensor = ui8DistSens_readCM(distSensFwd);
            uint8_t leftSensor = ui8DistSens_readCM(distSensLeft);
            uint8_t rearSensor = ui8DistSens_readCM(distSensRear);
            uint8_t rightSensor = ui8DistSens_readCM(distSensRight);
            
            xSemaphoreTake(xPoseMutex,40 / portTICK_PERIOD_MS);
                thetahat = gTheta_hat;
                xhat = gX_hat;
                yhat = gY_hat;
            xSemaphoreGive(xPoseMutex);
            
            // Experimental
            if ((idleCounter > 10) && (robotMovement == moveStop)){
                // If the robot stands idle for 1 second, send 'status:idle' in case the server missed it.
                printf(STATUS_IDLE"\n");
                idleCounter = 1;
            }
            else if ((idleCounter >= 1) && (robotMovement == moveStop)){
                idleCounter++;
            }             

            // Send updates to server
			
			//Commented out to decrease messages, 
            //vUSART_sendUpdate(xhat/10,yhat/10,thetahat*RAD2DEG,servoStep*servoResolution,forwardSensor,leftSensor,rearSensor,rightSensor);
            
            
            // Low level anti collision
            uint8_t objectX;
            if ((servoStep*servoResolution) <= 30) objectX = forwardSensor;// * cos(servoStep*5);
            else if((servoStep*servoResolution) >= 60) objectX = rightSensor;// * cos(270 + servoStep*5);
            else objectX = 0;
            
            if ((objectX > 0) && (objectX < 2)){
                // Stop controller
                struct sPolar Setpoint = {0, 0};
                xQueueSend(poseControllerQ, &Setpoint, 100);
            }
            
            // Iterate in a increasing/decreasing manner and depending on the robots movement
            if ((servoStep*servoResolution <= 90) && (rotationDirection == moveCounterClockwise) && (robotMovement < moveClockwise)){
                servoStep++;
            }
            else if ((servoStep*servoResolution > 0) && (rotationDirection == moveClockwise) && (robotMovement < moveClockwise)){
                servoStep --;
            }
            
            if ((servoStep*servoResolution >= 90) && (rotationDirection == moveCounterClockwise)){
                rotationDirection = moveClockwise;
            }
            else if ((servoStep*servoResolution <= 0) && (rotationDirection == moveClockwise)){
                rotationDirection = moveCounterClockwise;
            }    
        }
		/* Commented out due to fixed encoder
        else if ((gHandshook == TRUE) && (gPaused == TRUE)){
            // This 'else if' is only included to watch encoder values while robot is paused. 
            // Can safely be removed when the encoder has been replaced or repaired
            vServo_setAngle(0);
            idleCounter = 0;
            int16_t leftwheelticks = 0;
            int16_t rightwheelticks = 0;
            xSemaphoreTake(xTickMutex, 20 / portTICK_PERIOD_MS);
                leftwheelticks = gLeftWheelTicks;
                rightwheelticks = gRightWheelTicks;
            xSemaphoreGive(xTickMutex);
            if ((leftwheelticks != previous_left) ||  (rightwheelticks != previous_right)){
                printf("%i %i\n",leftwheelticks,rightwheelticks);
                previous_left = leftwheelticks;
                previous_right = rightwheelticks;               
            }
            
            vTaskDelay(200 / portTICK_PERIOD_MS);
        */
        else{ // Disconnected or unconfirmed
            vServo_setAngle(0);
            // Reset servo incrementation
            rotationDirection = moveCounterClockwise;
            servoStep = 0;
            idleCounter = 0;
            vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }// While end
}

/*  Calculates new settings for the movement task */
void vMainPoseControllerTask( void *pvParameters ){
    #ifdef DEBUG
        printf("PoseController OK\n");
        uint8_t tellar = 0;
    #endif
    /* Task init */    
    struct sPolar Setpoint = {0}; // Updates from server
    struct sPolar Error = {0}; // Error values
    struct sPolar Epsilon = {0.05235,1}; // Acceptable error {rad,cm} 
    struct sPolar oldVal = {0};
    struct sPolar referenceModel = {0};
      
    uint8_t correctHeading = TRUE;
    uint8_t correctDistance = TRUE;
    uint8_t rotDir = moveStop;
    uint8_t moveDir = moveStop;
    
    float thetahat = 0;
    float integrator = 0;
    int16_t xhat = 0;
    int16_t yhat = 0;
    int16_t xInit = 0;
    int16_t yInit = 0;
    
    
    while(1){
        // Checking if server is ready
        if (gHandshook){
            if (xSemaphoreTake(xControllerBSem, portMAX_DELAY) == pdTRUE){    // Wait for synchronization from estimator
                // Get robot pose
                xSemaphoreTake(xPoseMutex,portMAX_DELAY);
                    thetahat = gTheta_hat;
                    xhat = gX_hat;
                    yhat = gY_hat;
                xSemaphoreGive(xPoseMutex);
                       
                // Check if a new update is recieved
                if (xQueueReceive(poseControllerQ, &Setpoint, 0) == pdTRUE){
                    xQueueReceive(poseControllerQ, &Setpoint, 20 / portTICK_PERIOD_MS); // Receive theta and radius set points from com task, wait for 20ms if necessary
                    // New set points, reset PID controller variables
                    oldVal.heading = 0;
                    oldVal.distance = 0;
                    integrator = 0;
                    
                    if (Setpoint.heading != 0){
                        // Received setpoint is in (-pi,pi), use sign to decide rotation direction
                        if (Setpoint.heading > 0){
                            rotDir = moveCounterClockwise;
                        }
                        else{
                            rotDir = moveClockwise;
                        }
                        Setpoint.heading += thetahat; // Adjust set point relative to current heading
                        // Initialize reference model
                        referenceModel.heading = thetahat;
                        correctHeading = FALSE;
                    }
                    else if (Setpoint.heading == 0){
                        correctHeading = TRUE;
                        rotDir = moveStop;
                        uint8_t actuation = 0;
                        xQueueSend(movementQ, &rotDir, 0);
                        xQueueSend(actuationQ, &actuation, 0);                        
                    }
                    
                    if (Setpoint.distance != 0){
                        // Use sign to decide direction
                        if (Setpoint.distance > 0){
                            moveDir = moveForward;
                        }
                        else {
                            moveDir = moveBackward;
                        }
                        // Initalize X and Y
                        xInit = xhat;
                        yInit = yhat;
                        // Initialize reference model
                        int16_t xSquaredCm = xhat * xhat / 100;
                        int16_t ySquaredCm = yhat * yhat / 100;
                        referenceModel.distance = sqrt(xSquaredCm + ySquaredCm); 
                        correctDistance = FALSE;
                    }
                    else if (Setpoint.distance == 0){
                        correctDistance = TRUE;
                        moveDir = moveStop;
                        uint8_t actuation = 0;
                        xQueueSend(movementQ, &moveDir, 0);
                        xQueueSend(actuationQ, &actuation, 0);  
                    }
                    
                    if ((Setpoint.distance == 0) && (Setpoint.heading == 0)){
                        printf(STATUS_IDLE"\n");
                    }
                } // if (xQueueReceive(poseControllerQ, &Setpoint, 0) == pdTRUE) end
                // No new updates from server, update position errors:
                if(correctHeading == FALSE){
                    float referenceDiff  = (Setpoint.heading - referenceModel.heading);
                    vFunc_Inf2pi(&referenceDiff);
                       
                    referenceModel.heading = referenceModel.heading + (referenceDiff) / 10;                    
                    Error.heading = fabs(referenceModel.heading - thetahat);
                    vFunc_Inf2pi(&Error.heading);
                    
                    // We use cutoff to stop the robot we need to check the condition in a separate variable without the reference model.
                    float headingError = Setpoint.heading - thetahat;
                    vFunc_Inf2pi(&headingError);
                    
                    if (fabs(headingError) <= Epsilon.heading){
                        rotDir = moveStop;
                        correctHeading = TRUE;
                        Setpoint.heading = 0;
                        uint8_t actuation = 0;
                        integrator = 0;
                        xQueueSend(movementQ, &rotDir, 0);
                        if (correctDistance == TRUE){
                            xQueueSend(actuationQ, &actuation, 0);
                            printf(STATUS_IDLE"\n");
                        }
                    }
                    else{
                        float dHeading = thetahat - oldVal.heading; 
                        vFunc_Inf2pi(&dHeading);
                        
                        dHeading = fabs(dHeading) / 0.03; // Divide by sample time in seconds and get positive value
                        
                        integrator += Error.heading;
                        
                        if (integrator >= 25) integrator = 25;
                        
                        float pidOutput = (Error.heading + integrator - dHeading);
                        
                        if (pidOutput > 25) pidOutput = 25;
                        else if (pidOutput < 0) pidOutput = 0;
                        
                        uint8_t actuation = (uint8_t)pidOutput;
                        
                        xQueueSend(movementQ, &rotDir, 0);
                        xQueueSend(actuationQ, &actuation, 0);
                        oldVal.heading = thetahat;                        
                    }
                }
                else if (correctDistance == FALSE){
                    int16_t diffX = (xInit - xhat) / 10;
                    int16_t diffY = (yInit - yhat) / 10;
                    
                    diffX *= diffX;
                    diffY *= diffY;

                    referenceModel.distance = referenceModel.distance + (abs(Setpoint.distance) - referenceModel.distance) / 10;
                    Error.distance = referenceModel.distance - sqrt((diffX + diffY));
                    
                    // Since we use cutoff to stop the robot we need to check the condition in a separate variable without the reference model.
                    int16_t errorDistance = abs(Setpoint.distance) - sqrt((diffX + diffY));
                    
                    if (errorDistance <= Epsilon.distance){
                        moveDir = moveStop;
                        correctDistance = TRUE;
                        Setpoint.distance = 0;
                        integrator = 0;
                        uint8_t actuation = 0;
                        printf(STATUS_IDLE"\n"); 
                        xQueueSend(movementQ, &moveDir, 0);
                        xQueueSend(actuationQ, &actuation, 0);                        
                    }
                    else{
                        float dXY = (sqrt((diffX + diffY)) - oldVal.distance) / 0.030; // Divide by sample time in seconds and get positive value
                        
                        integrator += Error.distance;                        
                        if (integrator >= 25) integrator = 25;
                        
                        int16_t pidOutput = 4 * Error.distance + (int16_t)integrator - (int16_t)dXY;
                        
                        if (pidOutput > 25) pidOutput = 25;
                        else if (pidOutput < 0) pidOutput = 0;
                        
                        uint8_t actuation = (uint8_t)pidOutput;
                        
                        xQueueSend(movementQ, &moveDir, 0);
                        xQueueSend(actuationQ, &actuation, 0);
                        oldVal.distance = sqrt((diffX + diffY));
                    }
                }
            } // No semaphore available, task is blocking
        } //if(gHandshook) end
        else{
            // Reset controller
            correctHeading = TRUE;
            correctDistance = TRUE;
            moveDir = moveStop;
            xQueueSend(movementQ, &moveDir, 200 / portTICK_PERIOD_MS);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }// while (1)
}

/* Pose estimator task */
void vMainPoseEstimatorTask( void *pvParameters ){
    int16_t previous_ticksLeft = 0;
    int16_t previous_ticksRight = 0;  
    
    const TickType_t xDelay = PERIOD_ESTIMATOR_MS;
    float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0f;
    
    float kalmanGain = 0.5;
    
    float predictedTheta = 0.0;
    float predictedX = 0.0;
    float predictedY = 0.0;
    
    float gyroOffset = 0.0;
    float compassOffset = 0.0;
    
    // Found by using calibration task
    int16_t xComOff = 11; 
    int16_t yComOff = -78;
    
    float variance_gyro = 0.0482f; // [rad] calculated offline, see report
    float variance_encoder = (2.0f * WHEEL_FACTOR_MM) / (WHEELBASE_MM / 2.0f); // approximation, 0.0257 [rad]
    
    float variance_gyro_encoder = (variance_gyro + variance_encoder) * period_in_S; // (Var gyro + var encoder) * timestep
    float covariance_filter_predicted = 0;
    
    #define CONST_VARIANCE_COMPASS 0.3490f
    

    float gyroWeight = 0.5;//encoderError / (encoderError + gyroError);
    uint8_t robot_is_turning = 0;
    
    
    #ifdef DEBUG
        printf("Estimator OK");
        printf("[%i]",PERIOD_ESTIMATOR_MS);
        printf("ms\n");   
        uint8_t printerTellar = 0;     
    #endif
    
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    while(1){
        // Loop
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );  
        if (gHandshook){ // Check if we are ready    
            int16_t leftWheelTicks = 0;
            int16_t rightWheelTicks = 0;
            
            // Get encoder data, protect the global tick variables
            xSemaphoreTake(xTickMutex, 15 / portTICK_PERIOD_MS);
                leftWheelTicks = gLeftWheelTicks;
                rightWheelTicks = gRightWheelTicks;
            xSemaphoreGive(xTickMutex);
            
            float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
            float dRight =(float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
            previous_ticksLeft = leftWheelTicks;
            previous_ticksRight = rightWheelTicks;
                       
            float dRobot = (dLeft + dRight) / 2;           
            float dTheta = (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
            
            
            /* PREDICT */
            // Get gyro data:
            float gyrZ = (fIMU_readFloatGyroZ() - gyroOffset);
            //dTheta = gyrZ * period_in_S * DEG2RAD; [COMMENT]I believe this line is not supposed to be here. Residual from broken encoders?
            
            // If the robot is not really rotating we don't include the gyro measurements, to avoid the trouble with drift while driving in a straight line
            if(fabs(gyrZ) < 10){ 
                gyroWeight = 0; // Disregard gyro while driving in a straight line
                robot_is_turning = FALSE; // Don't update angle estimates
                }
            else {
                robot_is_turning = TRUE;
                gyroWeight = 0.85; // Found by experiment, after 20x90 degree turns, gyro seems 85% more accurate than encoders
                
            }
            
            gyrZ *= period_in_S * DEG2RAD; // Scale gyro measurement      
            
			
			
			
			
            // Fuse heading from sensors to predict heading:
            dTheta =  (1 - gyroWeight) * dTheta + gyroWeight * gyrZ;
            
            
            // Estimate global X and Y pos
            // Todo; Include accelerator measurements to estimate position and handle wheel slippage
            predictedX = predictedX + (dRobot * cos(predictedTheta + 0.5 * dTheta)); // [COMMENT]What is the 0.5?
            predictedY = predictedY + (dRobot * sin(predictedTheta + 0.5 * dTheta));

            // Predicted (a priori) state estimate for theta
            predictedTheta += dTheta;
                  
            // Predicted (a priori) estimate covariance
            covariance_filter_predicted += variance_gyro_encoder;
            
            /* UPDATE */
            // Get compass data: ( Request and recheck after 6 ms?)
            int16_t xCom, yCom, zCom;
            vCOM_getData(&xCom, &yCom, &zCom);
            // Add calibrated bias
            xCom += xComOff;
            yCom += yComOff;
            // calculate heading
            float compassHeading;
            compassHeading = atan2(yCom, xCom) - compassOffset ; // returns -pi, pi
            // Update predicted state:    
            float error = (compassHeading - predictedTheta);
            vFunc_Inf2pi(&error);
            
            // Handling of compass is removed due to broken encoders, so we estimate heading with gyro and always update with compass
            //kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
            ///* Commented back in due to fixed encoder
            if (fabs(error) > (0.8727*period_in_S)){ // 0.8727 rad/s is top speed while turning
                // If we have a reading over this, we can safely ignore the compass
                // Ignore compass while driving in a straight line
                kalmanGain = 0;
                vLED_singleLow(ledYELLOW);
            }
            else if ((robot_is_turning == FALSE) && (dRobot == 0)){
                // Updated (a posteriori) state estimate
                kalmanGain = covariance_filter_predicted / (covariance_filter_predicted + CONST_VARIANCE_COMPASS);
                vLED_singleHigh(ledYELLOW);
            }
            else{
                kalmanGain = 0;
                vLED_singleLow(ledYELLOW);
            }            
            //*/
           
            predictedTheta  += kalmanGain*(error);
			vFunc_Inf2pi(&predictedTheta);            
            
            // Updated (a posteriori) estimate covariance
            covariance_filter_predicted = (1 - kalmanGain) * covariance_filter_predicted;  

			//DEBUG START
			static int16_t messageCounter = 0;
			if (messageCounter == 2){
				messageCounter = 0;
				//printf("%.0f %.0f %.2f\n",predictedX,predictedY,predictedTheta);
				
				}else{
				messageCounter = messageCounter + 1;
			}
			//DEBUG END

            // Update pose
            xSemaphoreTake(xPoseMutex, 15 / portTICK_PERIOD_MS);
                gTheta_hat = predictedTheta;
                gX_hat = predictedX;
                gY_hat = predictedY;
            xSemaphoreGive(xPoseMutex);
            // Send semaphore to controller
            xSemaphoreGive(xControllerBSem);
        }
        else{
            // Not connected, getting heading and gyro bias
            uint16_t i;
            uint16_t samples = 100;
            float gyro = 0;
            for (i = 0; i<=samples; i++){
                gyro+= fIMU_readFloatGyroZ();
            }
            
            int16_t xCom, yCom, zCom;
            vCOM_getData(&xCom, &yCom, &zCom);
            xCom += xComOff;
            yCom += yComOff;
            
            
            // Initialize pose to 0 and reset offset variables
            predictedX = 0;
            predictedY = 0;
            predictedTheta = 0;
            
            compassOffset = atan2(yCom, xCom);    
            gyroOffset = gyro / (float)i;               
        }
    } // While(1) end
}

/* Handles request from position controller and sets motor pins. */
/* Frequency set by PERIOD_MOTOR_MS in defines.h */
void vMainMovementTask( void *pvParameters ){
    /* Task init */
    uint8_t lastMovement = 0;
    uint8_t movement = 0;
    uint8_t actuation = 0;
    
    int16_t bias_LeftWheelTick = 0;
    int16_t bias_RightWheelTick = 0;
    
    
    // PI control Variabels 
    uint8_t previous_leftEncoderVal = 0;
    uint8_t previous_rightEncoderVal = 0;
    int16_t errorL_I = 0;
    int16_t errorR_I = 0;
    int16_t rightVelocitySP = 0;
    
    
    const TickType_t xDelay = PERIOD_MOTOR_MS;
    // Initialise the xLastWakeTime variable with the current time.
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    #ifdef DEBUG
    printf("Movement OK\n");
    #endif
    
    int16_t leftWheelTicks = 0;
    int16_t rightWheelTicks = 0;
    
    uint8_t leftEncoderVal = 0;
    uint8_t rightEncoderVal = 0;
    
    uint8_t gLeftWheelDirection = 0;
    uint8_t gRightWheelDirection = 0;
    
    
    
    while(1){
        vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS );  
        // Handle encoder ticks
        ATOMIC_BLOCK(ATOMIC_FORCEON){
            leftEncoderVal = gISR_leftWheelTicks;
            gISR_leftWheelTicks = 0;
            rightEncoderVal = gISR_rightWheelTicks;
            gISR_rightWheelTicks = 0;
        }
        vMotorEncoderLeftTickFromISR(gLeftWheelDirection, &leftWheelTicks, leftEncoderVal);
        vMotorEncoderRightTickFromISR(gRightWheelDirection, &rightWheelTicks, rightEncoderVal);
        
        xSemaphoreTake(xTickMutex,1 / portTICK_PERIOD_MS);
            gLeftWheelTicks = leftWheelTicks;
            gRightWheelTicks = rightWheelTicks;
        xSemaphoreGive(xTickMutex);
            
        if (gHandshook){ // Check if we are connected and good to go
            xQueueReceive(movementQ, &movement, 0);
            xQueueReceive(actuationQ, &actuation, 0);
            if (movement != lastMovement){
                bias_LeftWheelTick = leftWheelTicks;
                bias_RightWheelTick = rightWheelTicks;
                lastMovement = movement;                   
                xQueueSend(scanStatusQ, &lastMovement, 0); // Send the current movement to the scan task
            }
            int16_t tmp_leftWheelTicks = 0;
            int16_t tmp_rightWheelTicks = 0;
            
            tmp_leftWheelTicks = leftWheelTicks - bias_LeftWheelTick;
            tmp_rightWheelTicks = rightWheelTicks - bias_RightWheelTick;

            /* Saturate values */
            if (actuation <= 0) actuation = 0;
            else if (actuation > 25) actuation = 25; 
            // Use right motor as "Master" and left as "Slave" 
            //uint8_t rightOutput = actuation * 8 + 40; // scale actuation from 48-240
            
            // PI controller for motors
            
			rightVelocitySP = actuation;
            
            int8_t leftVelocity = leftEncoderVal - previous_leftEncoderVal;
            previous_leftEncoderVal = tmp_leftWheelTicks;
            
            int8_t rightVelocity = rightEncoderVal - previous_rightEncoderVal;
            previous_rightEncoderVal = rightEncoderVal;
            
            int16_t errorR = (rightVelocitySP - rightVelocity);
            //int16_t errorL = (rightVelocity - leftVelocity);
            int16_t errorL = abs(abs(tmp_rightWheelTicks) - abs(tmp_leftWheelTicks));                      
            
            errorR_I += errorR;
            errorL_I += errorL;
            
            if (errorL_I > 255) errorL_I = 255;
            if (errorR_I > 255) errorR_I = 255;
            
            int16_t rightOutput = 2 * errorR + 2 * errorR_I;
            int16_t leftOutput = rightOutput + 10 * errorL + errorL_I;
			
			
			//Debugging stuff begin
			
			static int16_t previous_ticksLeft = 0;
			static int16_t previous_ticksRight = 0;
			static float totalRobot = 0;
			static float totalTheta = 0;
			
			float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
			float dRight =(float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
			previous_ticksLeft = leftWheelTicks;
			previous_ticksRight = rightWheelTicks;
			
			float dRobot = (dLeft + dRight) / 2;
			float dTheta = (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
			
			totalRobot += dRobot;
			totalTheta += dTheta;
			
			
			float period_in_S = PERIOD_MOTOR_MS / 1000.0f;
			static float predictedTheta = 0.0;
			
			
			float gyrZ = fIMU_readFloatGyroZ() - 0.6728; //Offset:  - 0.6728 degrees
			gyrZ *= period_in_S * DEG2RAD; // Scale gyro measurement
			predictedTheta += gyrZ;
			
			static float accelY = 0;
			
			accelY = (fIMU_readFloatAccelY() - 0.02);
			
			static float dacclIntr;
			static float acclIntr;
			
			if (fabs(accelY) > 0.01 ){	
				acclIntr += accelY;	
				dacclIntr += acclIntr;
			}
		
			static int16_t messageCounter = 0;
			if (messageCounter == 3){
				messageCounter = 0;
				
				float yAccl = fIMU_readFloatAccelY();
				float zGyro = fIMU_readFloatGyroX();
				//printf("%i %i\n",leftOutput, rightOutput);
				//printf("L: %i R: %i\n", leftWheelTicks, rightWheelTicks);
				//printf("Lvlcty: %i Rvlcty: %i", )
				//printf("R: %i L:  %i\n",rightOutput,leftOutput);
				//printf("%i %i \n",tmp_leftWheelTicks,tmp_rightWheelTicks);
			}
			
			else{
				messageCounter = messageCounter + 1;
			}
			
			
			//Debugging stuff end
			
           
            if (leftOutput < 0) leftOutput = 0;
            else if (leftOutput > 255) leftOutput = 255;

            if (rightOutput < 0) rightOutput = 0;
            else if (rightOutput > 255) rightOutput = 255;
            //*/
            //uint8_t leftOutput = 60; Commented out due to fixed encoder
            //uint8_t rightOutput = 78;
            
            // Send values to motor control switch. 
            // Note that both comparing ticks AND using a PI controller is suboptimal and should be changed in further work.
            // Set wheelticks to 0 to disable tickcomparison in function
            tmp_leftWheelTicks = 0;
            tmp_rightWheelTicks = 0;
            vMotorMovementSwitch(movement, leftOutput, rightOutput, tmp_leftWheelTicks, tmp_rightWheelTicks, &gLeftWheelDirection, &gRightWheelDirection); // Remove tmp_ticks if robot oscillates too much 
        }                
        else{ // Not connected, stop & do nothing
            vMotorBrakeLeft();
            vMotorBrakeRight();
        }
    }// While(1) end
}


//#define COMPASS_CALIBRATE

#ifdef COMPASS_CALIBRATE
void compassTask(void *par){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("Compass running\n");
    int16_t xComOff = 0;
    int16_t yComOff = 0;
    while(1){
        vTaskDelay(100 / portTICK_PERIOD_MS);
    if (gHandshook){
        int16_t xComMax = -4000, yComMax = -4000;
        int16_t xComMin = 4000, yComMin = 4000;
        int16_t xCom, yCom, zCom;   
// wait until you start moving     
//         while(fabs(zGyr) < 20){
//             zGyr = fIMU_readFloatGyroZ();
//             
//             vTaskDelay(15/portTICK_PERIOD_MS);
//         }
        printf("Rotating in\n...3\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("...2\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("...1\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uint8_t movement;
        movement = moveCounterClockwise;
        xQueueSend(movementQ, &movement, 10);
        
        float heading = 0;
        float gyroHeading = 0;
        float encoderHeading = 0;
        
        gLeftWheelTicks = 0;
        gRightWheelTicks = 0;
        float previous_ticksLeft = 0;
        float previous_ticksRight = 0;
// Storing values for printing later
//         uint8_t tellar = 0;
//         float tabellG[200];
//         float tabellE[200];
        
        TickType_t xLastWakeTime;
        const TickType_t xDelay = 50;
        int counter = 0;
        // Initialise the xLastWakeTime variable with the current time.
        xLastWakeTime = xTaskGetTickCount(); 
        while(heading < 359){
            vTaskDelayUntil(&xLastWakeTime, xDelay / portTICK_PERIOD_MS);
            int16_t leftWheelTicks = 0;
            int16_t rightWheelTicks = 0;
            ATOMIC_BLOCK(ATOMIC_FORCEON){
                leftWheelTicks = gLeftWheelTicks;
                rightWheelTicks = gRightWheelTicks;
            }
            float dLeft = (float)(leftWheelTicks - previous_ticksLeft) * WHEEL_FACTOR_MM; // Distance left wheel has traveled since last sample
            float dRight =(float)(rightWheelTicks - previous_ticksRight) * WHEEL_FACTOR_MM; // Distance right wheel has traveled since last sample
            if (counter >= 4){
                //printf("%i %i\n",leftWheelTicks,rightWheelTicks);
                counter = 0;
            }
            else counter++;
            
            previous_ticksLeft = leftWheelTicks;
            previous_ticksRight = rightWheelTicks;
            float dTheta = RAD2DEG * (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula
            
//            float zGyr = 0.054*(fIMU_readFloatGyroZ() + 1.700); // add offset bias
// Storing values for printing            
//             gyroHeading += zGyr;
//             encoderHeading += dTheta;
//             tabellE[tellar] = dTheta;
//             tabellG[tellar] = zGyr;
//             tellar++;
            heading += dTheta;
            
            vCOM_getData(&xCom, &yCom, &zCom);
            
            if(xCom > xComMax) xComMax = xCom;
            if(yCom > yComMax) yComMax = yCom;
            
            if(xCom < xComMin) xComMin = xCom;
            if(yCom < yComMin) yComMin = yCom;
        }
        movement = moveClockwise;
        xQueueSend(movementQ, &movement, 10);
        movement = moveStop;
        xQueueSend(movementQ, &movement, 10);
// Printing said values
//         int i = 0;
//         for (i = 0; i < tellar; i++){
//             printf("%.1f, %.1f\n",tabellG[i], tabellE[i]);
//             vTaskDelay(200 / portTICK_PERIOD_MS);
//         }
//         printf("gyro %.2f, encoder %.2f \n", gyroHeading, encoderHeading);
        
// Printing new xy cal values
        vUSART_sendS("Old XY-offset values:");
        char dobbel[8];
        itoa(xComOff, dobbel, 10);
        vUSART_sendS(dobbel);
        itoa(yComOff, dobbel, 10);
        vUSART_sendS(",\t ");
        vUSART_sendS(dobbel);
        vUSART_sendS("\n");
        xComOff = ((xComMax - xComMin)/2) - xComMax;
        yComOff = ((yComMax - yComMin)/2) - yComMax;
        
        printf("New XY-offset values:");
        itoa(xComOff, dobbel, 10);
        vUSART_sendS(dobbel);
        itoa(yComOff, dobbel, 10);
        vUSART_sendS(",\t ");
        vUSART_sendS(dobbel);
        vUSART_sendS("\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    else
    vTaskDelay(200 / portTICK_PERIOD_MS); 
        
        
    }
}
#endif

/*  In case of stack overflow, disable all interrupts and handle it  */
void vApplicationStackOverflowHook(TaskHandle_t *pxTask, signed char *pcTaskName){
    cli();
    /*  Handle overflow */
    #ifdef DEBUG
        vUSART_sendS("Overflow\n");
    #endif
    while(1){
        vLED_toggle(ledRED);
        //ledPORT ^= (1<<ledGREEN) | (1<<ledYELLOW) | (1<<ledRED);
    }// While(1) end
}

/*  Main function   */
int main(void){
    /* Setup - Initialize all settings before tasks  */
    /* Initialize LED, pins defined in LED.h   */
    vLED_init();
    vLED_singleHigh(ledRED); // Set red LED on to indicate INIT is ongoing
    /* Initialize USART driver, NB! baud is dependent on nRF51 dongle */
    vUSART_init();
    
    // If the MCU resets, the cause can be seen in MCUSR register
    // See page 56 in the data sheet
    #ifdef DEBUG
        vUSART_sendS("Reboot.\nStatus register: 0b");
        char printer[8];
        uint8_t reg = MCUSR;
        itoa(reg,printer,2);
        vUSART_sendS(printer);
        vUSART_sendS("\n");
        MCUSR = 0; // Reset MCUSR
    #endif
    #ifdef tictoc
        usetictoc;
        vUSART_sendS("tictoc!\n");
        tic;
    #endif
    
    /* Initialize servo for sensor tower to zero degrees */
    vServo_init(0);
    /* Initialize sensors */
    vDistSens_init();
    /* Initialize motor controller */
    vMotor_init();
    /* Initialize Inertial Measurement Unit (IMU) and SPI  */
    #ifdef DEBUG
        vUSART_sendS("IMU init..\n");
    #endif
    sIMU_begin(); 
   
    /* Initialize compass */
    /* Connected with I2C, if the chip has no power, MCU will lock. */
    #ifdef DEBUG
            vUSART_sendS("Compass init..\n");
    #endif
    
    vCOM_init();
    
    /* Initialize RTOS utilities  */
    movementQ = xQueueCreate(2,sizeof(uint8_t)); // For sending movements to vMainMovementTask
    poseControllerQ = xQueueCreate(1, sizeof(struct sPolar)); // For setpoints to controller
    scanStatusQ = xQueueCreate(1,sizeof(uint8_t)); // For robot status
    actuationQ = xQueueCreate(2,sizeof(uint8_t)); // To send variable actuation to motors
    
    xPoseMutex = xSemaphoreCreateMutex(); // Global variables for robot pose. Only updated from estimator, accessed from many
    xUartMutex = xSemaphoreCreateMutex(); // Protected printf with a mutex, may cause fragmented bytes if higher priority task want to print as well
    xTickMutex = xSemaphoreCreateMutex(); // Global variable to hold robot tick values
    
    xControllerBSem = xSemaphoreCreateBinary(); // Estimator to Controller synchronization
    xCommandReadyBSem = xSemaphoreCreateBinary(); // uart ISR to comm task sync
    
    // Todo: Check return variable to ensure RTOS utilities were successfully initialized before continue
    xTaskCreate(vMainMovementTask, "Movement", 500, NULL, 4, NULL); // Independent task, uses ticks from ISR
    xTaskCreate(vMainCommunicationTask, "Comm", 500, NULL, 3, NULL); // Dependant on ISR from UART, sends instructions to other tasks
    
    #ifndef COMPASS_CALIBRATE // If compass calibration task is running dont use these tasks
        xTaskCreate(vMainPoseControllerTask, "PoseCon", 500, NULL, 2, NULL); // Dependant on estimator, sends instructions to movement task
        xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 500, NULL, 5, NULL); // Independent task, uses ticks from ISR
        xTaskCreate(vMainSensorTowerTask,"Tower",500, NULL, 1, NULL); // Independent task, but use pose updates from estimator
    #endif

    #ifdef COMPASS_CALIBRATE
        vUSART_sendS("\n \t WARNING \t !\n");
        vUSART_sendS("COMPASS CALIBRATION!\n");
        vUSART_sendS("{S,CON} to begin\n");
        xTaskCreate(compassTask, "compasscal", 3500, NULL, 3, NULL); // Task used for compass calibration, dependant on communication and movement task
    #endif
    
    

    sei();
    vLED_singleLow(ledRED);
    #ifdef DEBUG
    vUSART_sendS("Starting scheduler ....\n");
    #endif
    /*  Start scheduler */
    vTaskStartScheduler();

    /*  MCU is out of RAM if the program comes here */
    while(1){
        cli();
        vUSART_sendS("RAM fail\n");
    }
}


/*
 Interrupt Service Routines
 Int2: Left wheel optical encoder
 Int3: Right wheel optical encoder
 Int4: nRF dongle status pin, NB set up in motor.c
 USART0 RX vector: USART with nRF dongle
*/

/* Handle tick from left wheel encoder */
// If ticks generate overly many interrupts, you can 
// connect to T0 and T5 pins and set up  hardware timer overflow to 
// prescale the ticks
ISR(leftWheelCount){
    gISR_leftWheelTicks++;
}

/* Handle tick from right wheel encoder */
ISR(rightWheelCount){
    gISR_rightWheelTicks++;
}

/* Handle change of connection status */
ISR(nRF51_status){
    if (nRFconnected){
        // indicate we are connected
        vLED_singleHigh(ledGREEN);
        vLED_singleHigh(ledYELLOW);
    }
    else{
        // We are not connected or lost connection, reset handshake flag
        gHandshook = FALSE;
        gPaused = FALSE;
        vLED_singleLow(ledGREEN);
        vLED_singleLow(ledYELLOW);
        vLED_singleLow(ledRED);
        xSemaphoreGiveFromISR(xCommandReadyBSem,0); // Let uart parser reset if needed
    }
    xSemaphoreGiveFromISR(xControllerBSem,0); // let the controller reset if needed    
}

/* Handle rx complete */
ISR(USART2_RX_vect){
    // Receive buffer
    gData_in[gData_count] = UDR2;
    // All messages start with a {, so if we don't get an {, ignore the message
    if (gData_in[gData_count] == '{'){
        // Initialize data counter
        gData_count = 0;
        gStoreString = TRUE;
    }
    if (gStoreString){
        // Check for end of line, or if the message is too long parse it anyways
        if (gData_in[gData_count] == '\n' || gData_count >= sizeof(gData_in)){
            xSemaphoreGiveFromISR(xCommandReadyBSem,0);
            // Reset to 0, ready to go again
            gData_count = 0;
            gStoreString = FALSE;
        }
        else{
            gData_count++;
        }
    }
}    