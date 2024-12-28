/**
 * @file main.c
 *
 * @brief Main source code for the BLE_UART program.
 *
 * This file contains the main entry point for the BLE_UART program,
 * which is used to demonstrate the BLE_UART driver.
 *
 * It interfaces with the Adafruit Bluefruit LE UART Friend Bluetooth Low Energy (BLE) module, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/2479
 *
 * @author Aaron Nanas
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/BLE_UART.h"
#include "inc/Timer_A0_Interrupt.h" // Added TIMER_A0 fot concurrency [MT]
#include "inc/Motor.h" // Added Motor Function [MT]
#include "inc/MPU_6050.h" // Added Gyro Function [MT]

//-----------------------------------------------------------------------------D E F I N  I T I O N S---------------------------------------------------------------------------------------

// Definitions
#define LEFT_TURN_RATE 2000
#define RIGHT_TURN_RATE 2000

//-----------------------------

//Global Variables
char BLE_UART_Buffer[BLE_UART_BUFFER_SIZE] = {0};

float RateRoll, RatePitch;
float RateYaw = 0;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;
float KalmanAngleRoll = 0,
        KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0,
        KalmanUncertaintyAnglePitch = 2 * 2;
float KalmanAngleYaw = 0,
        KalmanUncertaintyAngleYaw = 2 * 2;
float Kalman1DOutput[] = {0, 0, 0};


//********************************************* K A L M A N -- F I L T E R ********************************************************


void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
    {
        KalmanState = KalmanState + 0.004 * KalmanInput;
        KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
        float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
        KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
        KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

        Kalman1DOutput[0] = KalmanState;
        Kalman1DOutput[1] = KalmanUncertainty;
    }

void gyro_signals(void) {

     float* gyro_data_xyz = MPU_6050_Get_Adjusted_XYZ_Gyroscope();
     float* Accelerometer_data_xyz = MPU_6050_Get_Adjusted_XYZ_Acceleration();


    RateRoll = gyro_data_xyz[0];
    RatePitch = gyro_data_xyz[1];
    RateYaw = gyro_data_xyz[2];

    AccX = Accelerometer_data_xyz[0];
    AccY = Accelerometer_data_xyz[1];
    AccZ = Accelerometer_data_xyz[2];

    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
    AngleYaw = atan2(AccX, AccZ) * 1 / (3.142 / 180);
}

void Bot_Orientation()
{
        gyro_signals();
        RateRoll -= RateCalibrationRoll;
        RatePitch -= RateCalibrationPitch;
        RateYaw -= RateCalibrationYaw;
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        KalmanAngleRoll = Kalman1DOutput[0];
        KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        KalmanAnglePitch = Kalman1DOutput[0];
        KalmanUncertaintyAnglePitch = Kalman1DOutput[1];


        kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, RateYaw, AngleYaw);
        KalmanAngleYaw = Kalman1DOutput[0];
        KalmanUncertaintyAngleYaw = Kalman1DOutput[2];

        //Print Yaw Values to Bluefruit Connect application
//        char ble_data[50];
//        sprintf(ble_data, "YAW: %.2f", KalmanAngleYaw);
//        BLE_UART_OutString(ble_data);

        //Print Yaw Values to the serial monitor
        //printf("YAW Angle [%.2f] %.2f\n", KalmanAngleYaw);



}
//---------------------------------------------------------------------------F  U  N  C  T  I  O  N  S---------------------------------------------------------------------------------------

/**
 * @brief
 *
 * @param direction  The character to determine the direction the robot will rotate
 * @param angle      The value of the angle the robot will rotate
 *
 *
 * @return None
 */
void RotateBot(char direction, float angle) {

    KalmanAngleYaw = 0;
    if (direction == 'l') {
        // Start Rotations Motor Left
        Motor_Left(LEFT_TURN_RATE, LEFT_TURN_RATE);
        while(KalmanAngleYaw <= angle){
            //Uncomment to view Yaw Values
            printf("Yaw: %.2f \n", KalmanAngleYaw);

            //Constantly poll the bots orientation while turning
            Bot_Orientation();

        }
    }
    else if (direction == 'r') {

        Motor_Right(RIGHT_TURN_RATE, RIGHT_TURN_RATE);
        while(KalmanAngleYaw >= -angle){

            //Uncomment to view Yaw Values
            printf("Yaw: %.2f \n", KalmanAngleYaw);

            //Constantly poll the bots orientation while turning
            Bot_Orientation();

        }
    }

    // Stop the motors
    Motor_Stop();
}


void Process_BLE_UART_Data(char BLE_UART_Buffer[]) {
    if (Check_BLE_UART_Data(BLE_UART_Buffer, "donut")) {
        Motor_Right(7000, 7000);
        Clock_Delay1ms(3000);

            for(int i=0;i<4;i++)
            {
                Motor_Left(5000,5000);
                Clock_Delay1ms(200);
                Motor_Right(5000,5000);
                Clock_Delay1ms(200);
            }
         Motor_Left(7000, 7000);
         Clock_Delay1ms(3000);
        Motor_Stop();
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "f")) {
        Motor_Forward(6300,6500);
            if (Check_BLE_UART_Data(BLE_UART_Buffer, "s"))
               {
            Motor_Stop();
               }
    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "b")) {
        Motor_Backward(1500,1800);
            if (Check_BLE_UART_Data(BLE_UART_Buffer, "s"))
               {
                    Motor_Stop();
               }

    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "l") || Check_BLE_UART_Data(BLE_UART_Buffer, "r")) {

        // direction and angle
        char user_input_direction = BLE_UART_Buffer[0];
        float user_input_angle = atoi(&BLE_UART_Buffer[2]); //

        // rotate the bot
        RotateBot(user_input_direction, user_input_angle);

    } else if (Check_BLE_UART_Data(BLE_UART_Buffer, "s")) {
        Motor_Stop();
    }
}

void User_In_Command (void)
{

    int string_size = BLE_UART_InString(BLE_UART_Buffer, BLE_UART_BUFFER_SIZE);

           printf("BLE UART Data: ");
                 for (int i = 0; i < string_size; i++)
                    {
                        printf("%c", BLE_UART_Buffer[i]);
                    }

                            printf("\n");

                                Process_BLE_UART_Data(BLE_UART_Buffer);
}


//***********************************************************************************************************  M A I N  **********************************************************************************************************************************
int main(void)
{

    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock_
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
    LED1_Init();
    LED2_Init();

    // Initialize the buttons on the MSP432 microcontroller
    Buttons_Init();

    // Initialize the EUSCI_A0_UART module
    EUSCI_A0_UART_Init_Printf();

    // Initialize the Adafruit Bluefruit LE UART Friend module
    BLE_UART_Init();

    // Initialize the Gyro Module
    Clock_Delay1ms(1000); // Delay added before initialization of Gyro. This is because the bot must be physically touched to turn it on. Taking finger off the power button skews the gyro zeroing calibration.
    MPU_6050_Init();

    // Enable the interrupts used by the modules
    EnableInterrupts();

    // Provide a short delay after initialization and reset the BLE module
    Clock_Delay1ms(1000);
    BLE_UART_Reset();

    // Send a message to the BLE module to check if the connection is stable
    BLE_UART_OutString("Django Bot Cuh\r\n");
    Clock_Delay1ms(1000);

    //Motor Initialization
    Motor_Init();

    // Super-Loop
    while(1)
    {

           User_In_Command();


    }
}


