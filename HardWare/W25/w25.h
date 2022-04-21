#ifndef __W25Q_H
#define __W25Q_H			    
#include "sys.h" 
#include "spi.h"
#include "delay.h"	
		   
#define	W25Q_CS 		PBout(14)  		//W25Q的片选信号

//指令表
#define W25Q_WriteEnable		0x06 
#define W25Q_WriteDisable		0x04 
#define W25Q_ReadStatusReg		0x05 
#define W25Q_WriteStatusReg		0x01 
#define W25Q_ReadData			0x03 
#define W25Q_FastReadData		0x0B 
#define W25Q_FastReadDual		0x3B 
#define W25Q_PageProgram		0x02 
#define W25Q_BlockErase			0xD8 
#define W25Q_SectorErase		0x20 
#define W25Q_ChipErase			0xC7 
#define W25Q_PowerDown			0xB9 
#define W25Q_ReleasePowerDown	0xAB 
#define W25Q_DeviceID			0xAB 
#define W25Q_ManufactDeviceID	0x90 
#define W25Q_JedecDeviceID		0x9F 

void W25Q_Init(void);
u16  W25Q_ReadID(void);  	    		//读取FLASH ID
u8	 W25Q_ReadSR(void);        		//读取状态寄存器 
void W25Q_Write_SR(u8 sr);  			//写状态寄存器
void W25Q_Write_Enable(void);  		//写使能 
void W25Q_Write_Disable(void);		//写保护
void W25Q_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void W25Q_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);   //读取flash
void W25Q_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);//写入flash
void W25Q_Erase_Chip(void);    	  	//整片擦除
void W25Q_Erase_Sector(u32 Dst_Addr);	//扇区擦除
void W25Q_Wait_Busy(void);           	//等待空闲
//void W25Q_PowerDown(void);        	//进入掉电模式
void W25Q_WAKEUP(void);				//唤醒
#endif
















