#ifndef __COMMAND_H
#define __COMMAND_H

#include "sys.h"
#include "w25n.h"
#include "usart.h"
#include "bmi055.h"
#include "computation.h"

#define acc_g 9.7911

void Command_Execute(USART_TypeDef* USARTx);
void Commanad_ChipErase(void);
void Command_Bmi055StartWork(void);
void Command_Bmi055StopWork(void);
void Command_DataOutput(void);
void Command_StatusCheck(void);

#endif
