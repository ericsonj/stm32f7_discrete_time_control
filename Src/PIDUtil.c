/*
 * PIDUtil.c
 *
 *  Created on: Oct 2, 2019
 *      Author: ericson
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pidUtil.h"

uint32_t PIDUTIL_getValue(ADC_HandleTypeDef *adc) {
	uint32_t value = 0;
	HAL_ADC_Start(adc);
	if (HAL_ADC_PollForConversion(adc, 1000000) == HAL_OK) {
		value = HAL_ADC_GetValue(adc);
	}
	return value;
}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

// Converts a given integer x to string str[]. d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void PIDUTIL_ftoa(float n, char *res, int afterpoint)
{
	char *aux = res;
	bool isNegative =  false;
	if ( n < 0) {
		isNegative = true;
		n = n*(-1);
		res[0] = '-';
		aux++;
	}

	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = intToStr(ipart, aux, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		aux[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int)fpart, aux + i + 1, afterpoint);
	}

}

