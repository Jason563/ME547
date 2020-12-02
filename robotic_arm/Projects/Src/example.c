/**
  ******************************************************************************
  * @file       example.c
  * @date       01/10/2014 12:00:00
  * @brief      Example functions for the X-NUCLEO-IHM02A1
******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "example.h"


/**
  * @addtogroup MicrosteppingMotor_Exampled
  * @{
  */

/**
  * @addtogroup Example
  * @{
  */

/**
  * @defgroup   ExamplePrivateFunctions
  * @brief      Example Private Functions.
  * @{
  */

/**
  * @}
  */ /* End of ExamplePrivateFunctions */

/**
  * @addtogroup ExamplePrivateFunctions
  * @brief      Example Private Functions.
  * @{
  */

/**
  * @addtogroup ExampleExportedFunctions
  * @brief      Example Exported Functions.
  * @{
  */

/**
  * @brief  Example no.1 for X-NUCLEO-IHM02A1.
  * @note	Perform a complete motor axis revolution as MPR_1 equal movements,
  *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
  *			At the end of each movement there is a delay of DELAY_1 ms.
  *     	After each motor has performed a complete revolution there is a
  *			delay of DELAY_2 ms.
  *			Now all motors for each X-NUCLEO-IHM02A1 will start at the same
  *			time.
  *			They are going to run at INIT_SPEED for DELAY_3 ms.
  *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardStop
  *			at the same time.
  *			Perform a complete motor axis revolution as MPR_2 equal movements,
  *			for each L6470 mounted on all stacked X-NUCLEO-IHM02A1.
  *			At the end of each movement there is a delay of DELAY_1 ms.
  *			After that all motors for each X-NUCLEO-IHM02A1 will get a HardHiZ
  *			at the same time.
  */
	
#define  PERIOD_VALUE       (uint32_t)(666 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/2)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*37.5/100) /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE/4)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*12.5/100)
	
	
	
	#define Ulna 208 //in mm
	#define Humerus 310 //in mm
	#define z_offset 187 //z offset in mm
	
	#define M_PI 3.14159265358979323846
	/*
	#define MAX_ULAN_MOTOR_ANGLE 180 //degrees
	#define MAX_HUMERUS_MOTOR_ANGLE 90 //degrees
	#define MIN_ULAN_MOTOR_ANGLE 90 //degrees
	#define MIN_HUMERUS_MOTOR_ANGLE 20 //degrees
	*/
	
	#define HOME_SPEED 30000//12800
	
	#define STEPS_PER_ROTATION 351488 
	#define GRIPPER_STEPS_PER_ROTATION 25600
	
	#define ALPHA_CORRECTION_RATIO 1.50
	#define BETA_CORRECTION_RATIO 1.50
	#define GAMMA_CORRECTION_RATIO 1.50
	
	#define GAMMA_STARTING_ANGLE 0.5307787
	
	//#define HOME_AXIS
	//#define SERVO_CONTROL
	
	#define HUMERUS_MAX_PIN GPIO_PIN_8
	#define ULNA_MIN_PIN GPIO_PIN_9
	#define WRIST_MID_PIN GPIO_PIN_0
	#define CLAW_PIN GPIO_PIN_6


	float calc_alpha(float x, float y);
	float calc_beta(float x, float y, float z);
	float calc_gamma(float x, float y, float z);
	float calc_L(float x, float y, float z);
	float angle_to_steps(float angle);
	eL6470_DirId_t find_Dir_beta(float steps);
	eL6470_DirId_t find_Dir_gama(float steps);
	void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);
	void user_pwm_setvalue(uint16_t value);
	static void SystemClock_Config(void);
	int gripper_angle_to_steps(float angle);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim4;
	static UART_HandleTypeDef huart2;

void MicrosteppingMotor_Example_01(float x, float y, float z, float gripperAngle, int gripperState)
{
	uint8_t id;
	HAL_Init();
	
	uint8_t buffer[10];
	uint8_t data[6];
	
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= HUMERUS_MAX_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull 		 	= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= ULNA_MIN_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	__GPIOA_CLK_ENABLE();
	GPIO_InitStructure.Pin   			= WRIST_MID_PIN;
	GPIO_InitStructure.Mode  			= GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull  			= GPIO_NOPULL;
	GPIO_InitStructure.Speed 			= GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;
  
	uint8_t board1 = EXPBRD_ID(0);
	uint8_t board2 = EXPBRD_ID(1);
	uint8_t board3 = EXPBRD_ID(2);
	
  /* Setup each X-NUCLEO-IHM02A1 Expansion Board ******************************/
  
  /* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
  MotorParameterDataGlobal = GetMotorParameterInitData();
  
  for (id = 0; id < EXPBRD_MOUNTED_NR; id++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
    MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
  }
	//while(1){
		/*
		for(int x = 0; x<6; x++){
			HAL_USART_Receive(&huart2, buffer, 4, 1000);
				data[x] = *buffer;
		}*/
		
		
	if(gripperState==0)
	{
			//HOME WRIST
			StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(0));
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_FWD_ID, 25600/20);
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(0), BUSY_ID)==0);
			StepperMotorBoardHandle->Command->SoftStop(board3, L6470_ID(0));
			HAL_Delay(1000);
			StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, 25600/10);
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(0), BUSY_ID)==0);
			HAL_Delay(1000);
			StepperMotorBoardHandle->Command->GoHome(board3,L6470_ID(0));
		
			//HOME HUMERUS
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_FWD_ID, angle_to_steps(M_PI/2));
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);
			StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
			StepperMotorBoardHandle->Command->Run(board1, L6470_ID(0), L6470_DIR_FWD_ID, HOME_SPEED);
			while(!HAL_GPIO_ReadPin(GPIOA, HUMERUS_MAX_PIN));
			StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0));
			StepperMotorBoardHandle->Command->Run(board1, L6470_ID(0), L6470_DIR_REV_ID, HOME_SPEED);
			while(HAL_GPIO_ReadPin(GPIOA, HUMERUS_MAX_PIN));
			StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(0));
			StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), L6470_DIR_REV_ID, angle_to_steps(1.5708)*BETA_CORRECTION_RATIO);
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(0), BUSY_ID)==0);
	
			//HOME ULNA
			StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_FWD_ID, HOME_SPEED);
			while(!HAL_GPIO_ReadPin(GPIOA, ULNA_MIN_PIN));
			StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board1, L6470_ID(1), L6470_DIR_REV_ID, HOME_SPEED);
			while(HAL_GPIO_ReadPin(GPIOA, ULNA_MIN_PIN));
			StepperMotorBoardHandle->Command->HardStop(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
			StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), L6470_DIR_REV_ID, angle_to_steps(1.0472)*BETA_CORRECTION_RATIO);
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);
			
			//HOME WRIST ANGLE
			StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_FWD_ID, HOME_SPEED);
			while(!HAL_GPIO_ReadPin(GPIOA, WRIST_MID_PIN)){};
			StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->Run(board2, L6470_ID(1), L6470_DIR_REV_ID, HOME_SPEED);
			while(HAL_GPIO_ReadPin(GPIOA, WRIST_MID_PIN)){};
			StepperMotorBoardHandle->Command->HardStop(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(1));
			StepperMotorBoardHandle->Command->Move(board2, L6470_ID(1), L6470_DIR_REV_ID, angle_to_steps(0.523599)*BETA_CORRECTION_RATIO);
			while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(1), BUSY_ID)==0);
	}
	
	BSP_L6470_SetParam(board3, L6470_ID(1),L6470_STEP_MODE_ID, HALF_STEP);
	BSP_L6470_SetParam(board3, L6470_ID(1),L6470_KVAL_HOLD_ID, 200);
	BSP_L6470_SetParam(board3, L6470_ID(1),L6470_KVAL_ACC_ID, 200);
	BSP_L6470_SetParam(board3, L6470_ID(1),L6470_KVAL_DEC_ID, 200);
	BSP_L6470_SetParam(board3, L6470_ID(1),L6470_KVAL_RUN_ID, 200);
	/***************************************************************************/
	StepperMotorBoardHandle->Command->ResetPos(board2, L6470_ID(0));
	StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(0));
	StepperMotorBoardHandle->Command->ResetPos(board1, L6470_ID(1));
	StepperMotorBoardHandle->Command->ResetPos(board3, L6470_ID(1));
	
	z -= 55;
	
	float alphaSteps = angle_to_steps(calc_alpha(x, y))/ALPHA_CORRECTION_RATIO;
	float betaSteps = angle_to_steps(calc_beta(x, y, z))*BETA_CORRECTION_RATIO;
	float gamaSteps = angle_to_steps(calc_gamma(x, y, z) - GAMMA_STARTING_ANGLE)*GAMMA_CORRECTION_RATIO;
	
	StepperMotorBoardHandle->Command->Move(board3, L6470_ID(0), L6470_DIR_REV_ID, gripper_angle_to_steps(gripperAngle));
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(0), BUSY_ID)==0);
	
	StepperMotorBoardHandle->Command->Move(board2, L6470_ID(0), find_Dir_beta(alphaSteps),fabs(alphaSteps));
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0);
	
	StepperMotorBoardHandle->Command->Move(board1, L6470_ID(0), find_Dir_beta(betaSteps),fabs(betaSteps));
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(0), BUSY_ID)==0);
	
	StepperMotorBoardHandle->Command->Move(board1, L6470_ID(1), find_Dir_beta(gamaSteps),fabs(gamaSteps));
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(0), BUSY_ID)==0);
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0);
	
	HAL_Delay(10000);
	
	if(gripperState==0){
		StepperMotorBoardHandle->Command->Move(board3, L6470_ID(1), L6470_DIR_REV_ID, 400/3);
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(1), BUSY_ID)==0);
	}
	if(gripperState==1){
		StepperMotorBoardHandle->Command->Move(board3, L6470_ID(1), L6470_DIR_FWD_ID, 400/3);
		while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(1), BUSY_ID)==0);
	}
	
	StepperMotorBoardHandle->Command->GoHome(board2, L6470_ID(0));
	StepperMotorBoardHandle->Command->GoHome(board1, L6470_ID(1));
	StepperMotorBoardHandle->Command->GoHome(board1, L6470_ID(0));
	StepperMotorBoardHandle->Command->GoHome(board3, L6470_ID(0));
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(1), BUSY_ID)==0);
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board1, L6470_ID(0), BUSY_ID)==0);
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board2, L6470_ID(0), BUSY_ID)==0);
	while(StepperMotorBoardHandle->Command->CheckStatusRegisterFlag(board3, L6470_ID(0), BUSY_ID)==0);
	HAL_Delay(2000);
	
}
//}

float calc_alpha(float x, float y){
	float alpha = atan(y/x);
	if (x < 0) {
		alpha -= M_PI;
	}
	return alpha;
}

float calc_beta(float x, float y, float z){
	
	float temp_L = calc_L(x, y, z);
	
	float beta =  acos((pow(Ulna, 2) - pow(Humerus, 2) - pow(temp_L, 2))/(-2*Humerus*temp_L));
	
	float beta_inverse = (M_PI/2)-beta - asin(z/temp_L);
	
	return beta_inverse;
}

float calc_gamma(float x, float y, float z){
	float temp_L = calc_L(x, y, z);
	
	/*To Do: add gamma calculations, it was done incorrectly */
	float gama = acos((pow(temp_L, 2) - pow(Ulna, 2) - pow(Humerus, 2))/(-2*Ulna*Humerus));
	
	float delta_2 = gama - calc_beta(x, y, z);
	
	return  delta_2;
}

float calc_L(float x, float y, float z){
	return (float)sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

float angle_to_steps(float angle){
	float steps = STEPS_PER_ROTATION*angle/(2*M_PI);
	return steps;
}

eL6470_DirId_t find_Dir_beta(float steps){
	if(steps>=0){
		return L6470_DIR_FWD_ID;
	}
	else{
		return L6470_DIR_REV_ID;
	}
}

int gripper_angle_to_steps(float angle){
	int steps = GRIPPER_STEPS_PER_ROTATION*angle/(2*M_PI);
	return steps;
}


/**
  * @}
  */ /* End of ExamplePrivateFunctions */

/**
  * @}
  */ /* End of Example */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
