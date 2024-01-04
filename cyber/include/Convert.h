#pragma once
#include "stdint.h"
#include "string.h"

//0xA001 = REV(0x8005)
#define _CRC16_1b(x) ((((x) & 0x01)?0xA001:0u) ^ ((x) >> 1))
#define _CRC16_4b(x) _CRC16_1b(_CRC16_1b(_CRC16_1b(_CRC16_1b(x))))
#define _CRC16_8b(x) _CRC16_1b(_CRC16_1b(_CRC16_1b(_CRC16_1b(_CRC16_1b(_CRC16_1b(_CRC16_1b(_CRC16_1b(x))))))))

#define __CRC16_1(x) _CRC16_4b((x))
//#define __CRC16_1(x) _CRC16_8b((x))
#define __CRC16_4(x) __CRC16_1((x)*4+0),__CRC16_1((x)*4+1),__CRC16_1((x)*4+2),__CRC16_1((x)*4+3)
#define __CRC16_16(x) __CRC16_4((x)*4+0),__CRC16_4((x)*4+1),__CRC16_4((x)*4+2),__CRC16_4((x)*4+3)
#define __CRC16_64(x) __CRC16_16((x)*4+0),__CRC16_16((x)*4+1),__CRC16_16((x)*4+2),__CRC16_16((x)*4+3)
#define __CRC16_256(x) __CRC16_64((x)*4+0),__CRC16_64((x)*4+1),__CRC16_64((x)*4+2),__CRC16_64((x)*4+3)

//0xEDB88320u = REV(0x04C11DB7u)
#define _CRC32_1b(x) ((((x) & 0x01)?0xEDB88320u:0u) ^ ((x) >> 1))
#define _CRC32_8b(x) _CRC32_1b(_CRC32_1b(_CRC32_1b(_CRC32_1b(_CRC32_1b(_CRC32_1b(_CRC32_1b(_CRC32_1b(x))))))))

#define __CRC32_1(x) _CRC32_8b((x))
#define __CRC32_4(x) __CRC32_1((x)*4+0),__CRC32_1((x)*4+1),__CRC32_1((x)*4+2),__CRC32_1((x)*4+3)
#define __CRC32_16(x) __CRC32_4((x)*4+0),__CRC32_4((x)*4+1),__CRC32_4((x)*4+2),__CRC32_4((x)*4+3)
#define __CRC32_64(x) __CRC32_16((x)*4+0),__CRC32_16((x)*4+1),__CRC32_16((x)*4+2),__CRC32_16((x)*4+3)
#define __CRC32_256(x) __CRC32_64((x)*4+0),__CRC32_64((x)*4+1),__CRC32_64((x)*4+2),__CRC32_64((x)*4+3)

const
uint32_t TABLE_CRC16_LE[16] = {
	__CRC16_16(0u)
};

const
uint32_t TABLE_CRC32_LE[16] = {
	__CRC32_16(0u)
};

class Convert
{
public:
	//shortת char[]
	static int short2char(short value, char array[], int start);
	static int int2char(int value,char array[],int start);
	static unsigned char CheckSum(char dest[], int startIndex, int length);
	static char GetCheckSum(char dest[], int startIndex, int length);
	static int ascii2int(unsigned char ascii[], int length);
	static int hex2int_big(unsigned char ascii[], int length);
	static int hex2int_little(unsigned char ascii[], int length);
	static void hex2ascii(unsigned char hex, char[]);
	static char* byte2Char(char dest[], int startIndex, int length);
	static int byte2Int(char array[], int start);
	static short byte2Short(char array[], int start);
	static long byte2Long(char array[], int start);
	static double byte2Double(char array[], int start);
	static float byte2Float(char b[], int start);
	static float intBitsToFloat(int i);
	static int floatToRawIntBits(float f);
	static bool isEqual(char meta[], char compar[], int length);
	static bool isEmpty(char meta[], int length);
	static bool startWith(char meta[],char compar[]);
	static bool endWith(char meta[], char compar[], int index);
	static int float2char(float value, char array[], int start);
	static int double2char(double value, char array[], int start);
	static uint32_t user_crc32_le(uint32_t crc_in, void* mem, uint32_t size);
	static uint16_t user_crc16_le(uint16_t crc_in, void* mem, uint32_t size);
	static char AsciiToHex(unsigned char* pAscii, unsigned char* pHex, int nLen);
};