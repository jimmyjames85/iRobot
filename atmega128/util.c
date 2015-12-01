/*
 * util.c
 *
 *  Created on: Oct 13, 2015
 *      Author: jim
 */
#include <stdlib.h> /* malloc */
#include <stdio.h>
#include <string.h>
#include "util.h"

void ftoa(char * str, float val)
{
	int8_t max_decimals = 6;
	int16_t intValue = val;
	int8_t totalChars = sprintf(str, "%d.", intValue);
	int8_t totalDecimalChars = 0;
	val = val - intValue;


	if (val < 0)
		val *= -1;
	intValue = val * 10;
	if (intValue < 0)
		intValue *= -1;
	val = val * 10;

	while (val != 0 && max_decimals - totalDecimalChars > 0)
	{
		totalDecimalChars += sprintf(str + totalChars + totalDecimalChars, "%d", intValue);
		val = val - intValue;
		intValue = val * 10;
		val = val * 10;
	}
}

char isDigit(char c)
{
	return(c>='0' && c<='9');
}


char * newChar(char c)
{
	char * ret = (char *) malloc(sizeof(char));
	*ret = c;
	return ret;
}

int * newInt(int i)
{
	int * ret = (int *) malloc(sizeof(int));
	*ret = i;
	return ret;
}

/**
 * returns a clone of cstr
 */
char * newCStr(const char * cstr)
{
    int cstr_len = (strlen(cstr) + 1);//plus 1 to include '\0'
    char *newCStr = malloc(sizeof(char) * cstr_len);
    memcpy((void *) newCStr, (const void *) cstr, cstr_len);
    return newCStr;

}
/*
char *trimCStr(char const *cstr)
{
    //trim leading spaces
    while (*cstr != '\0' && *cstr == ' ')
        cstr++;

    return (char *)cstr;
}
*/