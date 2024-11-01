/*
 * Motor_control.h
 *
 *  Created on: Oct 28, 2024
 *      Author: user
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"

typedef struct {
    GPIO_TypeDef *STEP_Port;
    uint16_t STEP_Pin;
    GPIO_TypeDef *DIR_Port;
    uint16_t DIR_Pin;
    uint32_t stepDelay;  // Delay between steps in milliseconds
    int32_t encoderPosition;  // Encoder position for feedback
} StepperMotor_t;

typedef struct {
   StepperMotor_t  *motorId;         // Motor ID (1-6)
   int targetAngle;       // Target angle to move to
   int  stepDelay;      // Delay between steps (speed control)
   uint8_t Direction;
} MotorCommand_t;



uint16_t ConvertAngleToSteps(float targetAngle);
int GetEncoderPosition(void);
void StepMotor_Pulse(StepperMotor_t *motor, uint16_t steps, uint8_t direction);
void ProcessMotorCommand(uint8_t *commandBuffer, uint8_t length, MotorCommand_t *motorCommand);

#endif /* INC_MOTOR_CONTROL_H_ */
