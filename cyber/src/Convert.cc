#include "../include/Convert.h"
#include <string.h>
int Convert::int2char(int value, char array[], int start) 
{
	int xval;
	int i;
	int j = 0;
	unsigned char *S;
	xval = value;
	S = (unsigned char*)(&xval);
	for (i = 3; i >= 0; i--)
	{
		array[j + start] = *(S + i);
		j++;
	}
	return 4;
}  

int Convert::short2char(short value, char array[], int start) 
{
	int length = 2;
	for (int i = start; i < start + 2; i++) {
		int offSet = 2 - (i - start) - 1;
		char loop = char((char)(value >> 8 * offSet) & 0xFF);
		array[i] = loop;
	}
	return 2;
}


unsigned char Convert::CheckSum(char dest[], int startIndex, int length)
{
	int sum = 0;
	for (int i = startIndex; i < startIndex + length; i++)
	{
		sum += dest[i];
	}

	return sum;
}



char Convert::GetCheckSum(char dest[], int startIndex, int length)
{
	char sum = 0;
	for (int i = startIndex; i < startIndex + length; i++)
	{
		sum = char(sum^dest[i]);
	}

	return sum;
}

void Convert::hex2ascii(unsigned char hex, char ascii[])
{
	char b = hex >> 4;
	char d = (0X0F)&hex;

	if (b<10)
	{
		b = '0' + b;
	}
	else
	{
		b = 'A' + b - 10;
	}
	if (d<10)
	{
		d = '0' + d;
	}
	else
	{
		d = 'A' + d - 10;
	}
	ascii[0] = b;
	ascii[1] = d;

}

char Convert::AsciiToHex(unsigned char* pAscii, unsigned char* pHex, int nLen)
{
	int nHexLen = nLen / 2;
	unsigned char Nibble[2] = { 0 };
	int i = 0;
	int j = 0;

	if (nLen % 2)
	{
		return 1;
	}

	for (i = 0; i < nHexLen; i++)
	{
		Nibble[0] = *pAscii++;
		Nibble[1] = *pAscii++;
		for (j = 0; j < 2; j++)
		{
			if (Nibble[j] <= 'F' && Nibble[j] >= 'A')
				Nibble[j] = Nibble[j] - 'A' + 10;
			else if (Nibble[j] <= 'f' && Nibble[j] >= 'a')
				Nibble[j] = Nibble[j] - 'a' + 10;
			else if (Nibble[j] >= '0' && Nibble[j] <= '9')
				Nibble[j] = Nibble[j] - '0';
			else
				return 1;//Nibble[j] = Nibble[j] - 'a' + 10;

		}	// for (int j = ...)
		pHex[i] = Nibble[0] << 4;	// Set the high nibble
		pHex[i] |= Nibble[1];	//Set the low nibble
	}	// for (int i = ...)
	return 0;
}


int Convert::ascii2int(unsigned char ascii[], int length)
{
	int high = 0;
	int low = 0;
	int result = 0;
	int temp = 0;
	for (int i = 0; i < length/2; i++)
	{
		result = result << 8;
		if (ascii[i * 2] <= '9')
		{
			high = ascii[i * 2] - '0';
		}
		else
		{
			high = ascii[i * 2] - 'A' + 10;
		}

		if (ascii[i * 2 + 1] <= '9')
		{
			low = ascii[i * 2 + 1] - '0';
		}
		else
		{
			low = ascii[i * 2 + 1] - 'A' + 10;
		}
		temp = low + 16 * high; //
		result += temp; //����8λ
	}
	return result;
}

int Convert::hex2int_big(unsigned char ascii[], int length)
{
	int result = 0;
	for (int i = 0; i < length; i++)
	{
		result = result << 8;
		result |= ascii[i];
	}
	return result;
}
int Convert::hex2int_little(unsigned char ascii[], int length)
{
	int result = 0;
	for (int i = 0; i < length; i++)
	{
		result |= (unsigned int)ascii[i] << (i * 8);
	}

	result = (result << (32 - 8 * length)) >> (32 - 8 * length);
	return result;
}
char* Convert::byte2Char(char dest[], int startIndex, int length)
{
	char* c = new char(length);
	for (int i = 0; i < length; i++)
	{
		c[i] = ((char)dest[(startIndex + i)]);
	}

	return c;
}

int Convert::byte2Int(char array[], int start)
{
	int length = 4;
	int result = 0;
	for (int i = start; i < start + 4; i++)
	{
		char loop = array[i];
		int offSet = 4 - (i - start) - 1;
		result += ((loop & 0xFF) << 8 * offSet);
	}
	return result;
}


short Convert::byte2Short(char array[], int start)
{
	int length = 2;
	short result = 0;
	for (int i = start; i < start + 2; i++)
	{
		char loop = array[i];
		int offSet = 2 - (i - start) - 1;
		result = (short)(result + ((loop & 0xFF) << 8 * offSet));
	}
	return result;
}
 long Convert::byte2Long(char array[], int start)
{
	long num = 0L;
	for (int ix = start; ix < start + 8; ix++)
	{
		num <<= 8;
		num |= array[ix] & 0xFF;
	}
	return num;
}

double Convert::byte2Double(char array[], int start)
{
	long l = byte2Long(array, start);
	//return Double.longBitsToDouble(l);
	union
	{
		long l;
		double d;
	} u;
	u.l = l;
	return u.d;
}

float Convert::byte2Float(char b[], int start)
{
	int i = byte2Int(b, start);
	return intBitsToFloat(i);
}

float Convert::intBitsToFloat(int i)
{
	union
	{
		int i;
		float f;
	} u;
	u.i = i;
	return u.f;
}

int Convert::floatToRawIntBits(float f)
{
	union
	{
		int i;
		float f;
	} u;
	u.f = f;
	return u.i;
}

bool Convert::isEqual(char meta[], char compar[], int length)
{
	for (int i = 0; i < length; i++) {
		if (meta[i] == compar[i])
		{
			continue;
		}
		else
		{
			return false;
		}
	}

	return true;
}

bool Convert::isEmpty(char meta[], int length)
{
	for (int i = 0; i < length; i++) {
		if (meta[i] == '\0')
		{
			continue;
		}
		else
		{
			return false;
		}
	}
	return true;
}

bool Convert::startWith(char meta[], char compar[])
{
	if (sizeof(meta) < sizeof(compar))
	{
		return false;
	}

	for (int i = 0; i < 1; i++)
	{
		if (meta[i] != compar[i]) {
			return false;
		}
	}

	return true;
}


bool Convert::endWith(char meta[], char compar[], int index)
{
	if (sizeof(meta) < sizeof(compar))
	{
		return false;
	}

	for (int i = 0; i < 1; i++)
	{
		if (meta[(i + index)] != compar[i])
		{
			return false;
		}
	}

	return true;
}

int Convert::float2char(float value, char array[], int start) 
{
	float xval;
	int i;
	int j = 0;
	unsigned char *S;
	xval = value;
	S = (unsigned char*)(&xval);
	for (i = 3; i >= 0; i--)
	{
		array[j + start] = *(S + i);
		j++;
	}

	return 4;
} 

int Convert::double2char(double value, char array[], int start)
{

	int j = 0;
	char* p = (char*)&value;
	for (int i = 0; i < 8; i++)
	{
		array[7-i] = *p++;
	}
	return 8;
}




uint16_t Convert::user_crc16_le(uint16_t crc_in, void* mem, uint32_t size)
{
	volatile uint8_t* in = (uint8_t*)mem;
	uint_fast16_t crc = crc_in;

	int i;
	for (i = 0; i < size; i++)
	{
		uint8_t tmp = in[i];

		crc ^= (uint_fast16_t)tmp;
		crc = TABLE_CRC16_LE[crc & 0xF] ^ (crc >> 4);
		crc = TABLE_CRC16_LE[crc & 0xF] ^ (crc >> 4);
	}

	return crc;
}

uint32_t Convert::user_crc32_le(uint32_t crc_in, void* mem, uint32_t size)
{
	volatile uint8_t* in = (uint8_t*)mem;
	uint_fast32_t crc = crc_in;

	int i;
	for (i = 0; i < size; i++)
	{
		uint8_t tmp = in[i];

		crc ^= (uint_fast32_t)tmp;
		crc = TABLE_CRC32_LE[crc & 0xF] ^ (crc >> 4);
		crc = TABLE_CRC32_LE[crc & 0xF] ^ (crc >> 4);
	}

	return crc;
}
