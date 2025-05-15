#ifndef _BSP_hwt101_h
#define _BSP_hwt101_h

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

extern int hwt_flag;
int fputc(int c, FILE* stream);
int fputs(const char *restrict s, FILE* restrict stream);
int puts(const char* _ptr);

bool isNegativeHex(uint8_t hexValue);
void initRingBuff_hwt101(void);
void writeRingBuff_hwt101(uint8_t data);
void deleteRingBuff_hwt101(uint16_t size);
uint8_t read1BFromRingBuff_hwt101(uint16_t position);
uint16_t getRingBuffLenght_hwt101();
void data_parsing_hwt101(void);
void gyro_offset_Init();
void hwt_init(void);

extern float gyro_offset_Zdata;
extern int gyro_z;


#define jsize() getRingBuffLenght_hwt101()
#define code_j() initRingBuff_hwt101()
#define jdelete(x) deleteRingBuff_hwt101(x)
#define j(x) read1BFromRingBuff_hwt101(x)



#endif
