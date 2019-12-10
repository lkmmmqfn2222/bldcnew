#ifndef _MYPRINT_H
#define _MYPRINT_H

#include "usart.h"
#include "stdio.h"
#include "stdarg.h"
#include "main.h"


void    myPrint(char* fmt, ...);
void    printCh(char ch);
void    printDec(int dec);
void    printFlt(double flt);
void    printBin(int bin);
void    printHex(int hex);
void    printStr(char* str);


#endif