#include "main.h"

/* External TIM handle declaration */
extern TIM_HandleTypeDef htim2;

/**
 * @brief Sets PWM duty cycle for a timer channel
 * @param htim Timer handle
 * @param Channel Timer channel
 * @param dutyCycle Duty cycle (0-100)
 */
static void PWM_SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t dutyCycle) {
    uint16_t pulse = (__HAL_TIM_GET_AUTORELOAD(htim) * dutyCycle) / 100;
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}

/**
 * @brief Initialize motor control
 * @retval None
 */
void Motor_Init(void) {
    /* Start PWM on both channels */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Left motor
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Right motor
    
    /* Initialize motors to stopped state */
    PWM_SetDutyCycle(&htim2, TIM_CHANNEL_1, 0);
    PWM_SetDutyCycle(&htim2, TIM_CHANNEL_2, 0);
}

/**
 * @brief Control left motor power
 * @param power Power level (-100 to 100)
 *        Positive values: forward, Negative values: reverse, 0: stop
 * @retval None
 */
void LeftMotor(int16_t power) {
    /* Limit power to valid range */
    if (power > 100) power = 100;
    if (power < -100) power = -100;
    
    /* Set PWM based on power value */
    if (power >= 0) {
        /* Forward direction */
        PWM_SetDutyCycle(&htim2, TIM_CHANNEL_1, power);
    } else {
        /* Reverse direction - implement if you have direction control */
        /* For now, just use absolute value of power */
        PWM_SetDutyCycle(&htim2, TIM_CHANNEL_1, -power);
        
        /* If you have direction control pins, you would set them here */
        /* Example: HAL_GPIO_WritePin(LEFT_DIR_GPIO_Port, LEFT_DIR_Pin, GPIO_PIN_SET); */
    }
}

/**
 * @brief Control right motor power
 * @param power Power level (-100 to 100)
 *        Positive values: forward, Negative values: reverse, 0: stop
 * @retval None
 */
void RightMotor(int16_t power) {
    /* Limit power to valid range */
    if (power > 100) power = 100;
    if (power < -100) power = -100;
    
    /* Set PWM based on power value */
    if (power >= 0) {
        /* Forward direction */
        PWM_SetDutyCycle(&htim2, TIM_CHANNEL_2, power);
    } else {
        /* Reverse direction - implement if you have direction control */
        /* For now, just use absolute value of power */
        PWM_SetDutyCycle(&htim2, TIM_CHANNEL_2, -power);
        
        /* If you have direction control pins, you would set them here */
        /* Example: HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, GPIO_PIN_SET); */
    }
}

/**
 * @brief Stop both motors
 * @retval None
 */
void StopMotors(void) {
    PWM_SetDutyCycle(&htim2, TIM_CHANNEL_1, 0);
    PWM_SetDutyCycle(&htim2, TIM_CHANNEL_2, 0);
}

/**
 * @brief Adjust motor speeds to turn toward target bearing
 * @param currentBearing Current heading in degrees (0-359)
 * @param targetBearing Target heading in degrees (0-359)
 * @param baseSpeed Base speed for forward movement (0-100)
 * @retval None
 */
void AdjustHeading(float currentBearing, float targetBearing, uint8_t baseSpeed) {
    // Calculate the angle difference (-180 to +180 degrees)
    float angleDiff = targetBearing - currentBearing;

    // IF angleDiff is below 10 degrees, stop the motors
    if (fabs(angleDiff) < 10.0f) {
        StopMotors();
        return;
    }
    
    // Normalize to -180 to +180 range
    if (angleDiff > 180.0f) {
        angleDiff -= 360.0f;
    } else if (angleDiff < -180.0f) {
        angleDiff += 360.0f;
    }
    
    // Calculate turn intensity based on angle difference (0.0 to 1.0)
    float turnIntensity = fabs(angleDiff) / 180.0f;
    if (turnIntensity > 1.0f) turnIntensity = 1.0f;
    
    // Calculate motor speeds
    int16_t leftSpeed, rightSpeed;
    
    if (angleDiff > 0) {
        // Need to turn right
        leftSpeed = baseSpeed;
        rightSpeed = baseSpeed * (1.0f - turnIntensity * 2.0f);
        if (rightSpeed < -baseSpeed) rightSpeed = -baseSpeed;
    } else {
        // Need to turn left
        rightSpeed = baseSpeed;
        leftSpeed = baseSpeed * (1.0f - turnIntensity * 2.0f);
        if (leftSpeed < -baseSpeed) leftSpeed = -baseSpeed;
    }

    if (abs(leftSpeed) < baseSpeed / 10) leftSpeed = 0;
    if (abs(rightSpeed) < baseSpeed / 10) rightSpeed = 0;
    
    // Apply motor speeds
    LeftMotor(leftSpeed);
    RightMotor(rightSpeed);
}
