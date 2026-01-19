/*
 * User_Usart.c
 *
 *  Created on: Feb 17, 2025
 *      Author: taegy
 */


#include "UART.h"

OperationStep currentStep = {STEP_WAIT_FOR_COMMAND};
Position_t currentPosition = {0};
uint8_t command[COMMAND_MAX_SIZE+2] = {0};
bool Rx_flag = false;

void UART_Transmit_EPS32(char *str){
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)str, strlen(str));
}

void UART_Receive_ESP32(){
	Rx_flag = false;
//	memset(command, 0, COMMAND_MAX_SIZE+2);
	HAL_UART_Receive(&huart2, (uint8_t *)command, COMMAND_MAX_SIZE+2,100);
}

void UART_Command_Vaildate(uint8_t length, OperationStep Step){
	if (strlen((char *)command) != length){
		Rx_flag = false;
		return;
	}

	if (Step == STEP_WAIT_LIFT_UP){
		if (command[0] == '2') Rx_flag = true;
	}
	else if (Step == STEP_WAIT_LIFT_DOWN){
		if (command[0] == '3') Rx_flag = true;
	}
	else if (Step == STEP_WAIT_FOR_COMMAND){
		if ((command[0] >= 'A' && command[0] <= 'R') || (command[0] >= 'a' && command[0] <= 'r'))	Rx_flag = true;
	}

	return;
}

//void UART_Command_Vaildate(uint8_t length, OperationStep Step){
//	if (strlen((char *)command) != length){
//		Rx_flag = false;
//		return;
//	}
//
//	if (length == 1){
//		if (Step == STEP_WAIT_LIFT_UP){
//			if (command[0] == '2') Rx_flag = true;
//			else Rx_flag = false;
//		}
//		else if (Step == STEP_WAIT_LIFT_DOWN){
//			if (command[0] == '3') Rx_flag = true;
//			else Rx_flag = false;
//		}
//		return;
//	}
//
//	if (length == 2){
//		if (command[0] < '0' || command[0] > '3' || command[1] < '0' || command[1] > '9'){
//			Rx_flag = false;
//		}
//		else Rx_flag = true;
//		return;
//	}
//}

void UART_Command_Parse(){
// ------------------------------------------------------------------------------------------------
	if (command[0] >= 'a' && command[0] <= 'r'){
		currentPosition.io_type = 1;
	}
	else {
		currentPosition.io_type = 0;
	}
// ------------------------------------------------------------------------------------------------
	if ((command[0] >= 'A' && command[0] <= 'I') || (command[0] >= 'a' && command[0] <= 'i')){
		currentPosition.side = 0;
	}
	else {
		currentPosition.side = 1;
	}
// ------------------------------------------------------------------------------------------------
  if ((command[0] >= 'A' && command[0] <= 'C') || (command[0] >= 'J' && command[0] <= 'L') ||
  		(command[0] >= 'a' && command[0] <= 'c') || (command[0] >= 'j' && command[0] <= 'l')){
    currentPosition.floor = 1;
	}
  else if ((command[0] >= 'D' && command[0] <= 'F') || (command[0] >= 'M' && command[0] <= 'O') ||
  				(command[0] >= 'd' && command[0] <= 'f') || (command[0] >= 'm' && command[0] <= 'o')){
    currentPosition.floor = 2;
	}
  else if ((command[0] >= 'G' && command[0] <= 'I') || (command[0] >= 'P' && command[0] <= 'R') ||
  				(command[0] >= 'g' && command[0] <= 'i') || (command[0] >= 'p' && command[0] <= 'r')){
    currentPosition.floor = 3;
	}
// ------------------------------------------------------------------------------------------------
  if (command[0] == 'A' || command[0] == 'a' ||
  		command[0] == 'D' || command[0] == 'd' ||
			command[0] == 'G' || command[0] == 'g' ||
			command[0] == 'J' || command[0] == 'j' ||
			command[0] == 'M' || command[0] == 'm' ||
			command[0] == 'P' || command[0] == 'p'){
  	currentPosition.cell = 1;
	}
  else if (command[0] == 'B' || command[0] == 'b' ||
					command[0] == 'E' || command[0] == 'e' ||
					command[0] == 'H' || command[0] == 'h' ||
					command[0] == 'K' || command[0] == 'k' ||
					command[0] == 'N' || command[0] == 'n' ||
					command[0] == 'Q' || command[0] == 'q'){
  	currentPosition.cell = 2;
	}
  else if (command[0] == 'C' || command[0] == 'c' ||
					command[0] == 'F' || command[0] == 'f' ||
					command[0] == 'I' || command[0] == 'i' ||
					command[0] == 'L' || command[0] == 'l' ||
					command[0] == 'O' || command[0] == 'o' ||
					command[0] == 'R' || command[0] == 'r'){
  	currentPosition.cell = 3;
	}
// ------------------------------------------------------------------------------------------------
}
