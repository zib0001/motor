/*
 * motor_control.c
 *
 *  Created on: Oct 28, 2024
 *      Author: user
 */
#include "motor_control.h"
#include "main.h"

int GetEncoderPosition(void)
{
	//encoder value
printf("encode");
return 1;
}

uint16_t ConvertAngleToSteps(float targetAngle) {

	static uint16_t stepsPerRevolution= 400;
    // Ensure the targetAngle is within 0-360 degrees


    // Calculate the number of steps required for the given angle
    uint16_t steps = (uint16_t)((targetAngle / 360.0f) * stepsPerRevolution);

    return steps;
}



void StepMotor_Pulse(StepperMotor_t *motor, uint16_t steps, uint8_t direction) {
    // Get the initial encoder position
    motor->encoderPosition = GetEncoderPosition();

    DEBUG_PRINT("StepMotor_Pulse");
    // Set the direction
    if (direction == 1) {
        HAL_GPIO_WritePin(motor->DIR_Port, motor->DIR_Pin, GPIO_PIN_SET);  // Forward
    } else {
        HAL_GPIO_WritePin(motor->DIR_Port, motor->DIR_Pin, GPIO_PIN_RESET);  // Reverse
    }

    // Perform the steps
    for (uint16_t i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(motor->STEP_Port, motor->STEP_Pin, GPIO_PIN_SET);
        osDelay(motor->stepDelay);
        HAL_GPIO_WritePin(motor->STEP_Port, motor->STEP_Pin, GPIO_PIN_RESET);
        osDelay(motor->stepDelay);
    }

    // Get the final encoder position
    motor->encoderPosition = GetEncoderPosition();
}


void ProcessMotorCommand(uint8_t *commandBuffer, uint8_t length, MotorCommand_t *motorCommand)
{
    if (length != 6)
    {
        // Invalid command length, handle the error (e.g., ignore or report)
        printf("Invalid command length\n");
        return;
    }

    // Extract the motor ID (1 byte)
    uint8_t motorId = commandBuffer[0];

    // Extract the target angle (2 bytes, MSB first, 16-bit integer)
    uint16_t targetAngle = (commandBuffer[1] << 8) | commandBuffer[2];

    // Extract the step delay (2 bytes, MSB first, 16-bit integer)
    uint16_t stepDelay = (commandBuffer[3] << 8) | commandBuffer[4];

    // Extract the direction (1 byte, 0 = clockwise, 1 = counterclockwise)
    uint8_t direction = commandBuffer[5];

    // Print the extracted values (for debugging)
    printf("Motor ID: %d, Target Angle: %d, Step Delay: %d, Direction: %d\n",
           motorId, targetAngle, stepDelay, direction);

    // Assuming you have a function to map motor ID to motor struct
       motorCommand->targetAngle = targetAngle;
       motorCommand->stepDelay = stepDelay;
       motorCommand->Direction = direction;
}
