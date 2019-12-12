#include "myprint.h"

UART_HandleTypeDef huart1;

void myPrint(char* fmt, ...)
{
	double vargflt = 0;
	int  vargint = 0;
	char* vargpch = NULL;
	char vargch = 0;
	char* pfmt = NULL;
	va_list vp;

	va_start(vp, fmt);
	pfmt = fmt;

	while (*pfmt)
	{
		if (*pfmt == '%')
		{
			switch (*(++pfmt))
			{

			case 'c':
				vargch = va_arg(vp, int);
				/*    va_arg(ap, type), if type is narrow type (char, short, float) an error is given in strict ANSI
				mode, or a warning otherwise.In non-strict ANSI mode, 'type' is allowed to be any expression. */
				printCh(vargch);
				break;
			case 'd':
			case 'i':
				vargint = va_arg(vp, int);
				printDec(vargint);
				break;
			case 'f':
				vargflt = va_arg(vp, double);
				/*    va_arg(ap, type), if type is narrow type (char, short, float) an error is given in strict ANSI
				mode, or a warning otherwise.In non-strict ANSI mode, 'type' is allowed to be any expression. */
				printFlt(vargflt);
				break;
			case 's':
				vargpch = va_arg(vp, char*);
				printStr(vargpch);
				break;
			case 'b':
			case 'B':
				vargint = va_arg(vp, int);
				printBin(vargint);
				break;
			case 'x':
			case 'X':
				vargint = va_arg(vp, int);
				printHex(vargint);
				break;
			case '%':
				printCh('%');
				break;
			default:
				break;
			}
			pfmt++;
		}
		else
		{
			printCh(*pfmt++);
		}
	}
	va_end(vp);
}

void printCh(char ch)
{
	//	return;
	//USART_ClearFlag(USART1, USART_FLAG_TC);
    HAL_UART_Transmit(&huart1,(__uint8_t*)&ch,1,0x0ff);
}

void printDec(int dec)
{
	if (dec == 0)
	{
		printCh('0');
		return;
	}
	if (dec < 0)
	{
		dec = -dec;
		printCh('-');
	}

	int num = 0;
	int tmp = dec;
	while (tmp >= 10)
	{
		num += 1;
		tmp /= 10;
	}

	for (int i = num; i >= 0; i--)
	{
		int tmp = dec;
		for (int j = 0; j < i; j++)
			tmp /= 10;
		tmp = tmp % 10;
		printCh((char)(tmp + '0'));
	}
	
}

void printFlt(double flt)
{
//	int icnt = 0;
	int tmpint = 0;

	tmpint = (int)flt;
	printDec(tmpint);
	printCh('.');
	flt = flt - tmpint;
	tmpint = (int)(flt * 100000);
	printDec(tmpint);
}

void printStr(char* str)
{
	while (*str)
	{
		printCh(*str++);
	}
}

void printBin(int bin)
{
	if (bin == 0)
	{
		printStr("0b");
		return;
	}
	printBin(bin / 2);
	printCh((char)(bin % 2 + '0'));
}

void printHex(int hex)
{
	if (hex == 0)
	{
		printStr("0x");
		return;
	}
	printHex(hex / 16);
	if (hex < 10)
	{
		printCh((char)(hex % 16 + '0'));
	}
	else
	{
		printCh((char)(hex % 16 - 10 + 'a'));
	}
}
