#include <stdio.h>
/*#include "platform.h"
#include "xil_printf.h"
#include "MotorFeedback.h"
#include "PmodDHB1.h"
#include "PmodACL.h"
#include "PmodGYRO.h"
#include "PWM.h"
#include "sleep.h"
#include "xil_cache.h"
#include "xparameters.h"*/
#include "math.h"
#include <stdint.h>
#include <stdlib.h> 

#include <time.h>

#define SIZE 10

float invSqrt( float number );
float *klfilt(float *array);
float stdev(float *data);

void main()
{
float sensor[SIZE];
int i = 0;
for(i = 0 ; i<SIZE; i++){
    sensor[i] = rand() % 70;
}
float *fvalue = klfilt(sensor);
   for ( i = 0; i < 10; i++ ) {
      printf( "*(origvalue[%d]) : %f\n", i, sensor[i]);
      printf( "*(filtvalue[%d]) : %f\n", i, fvalue[i]);
   }
}

float invSqrt( float number )
{
	const float x2 = number * 0.5F;
	const float threehalfs = 1.5F;

	union {
		float f;
		uint32_t i;
	} conv  = { .f = number };
	conv.i  = 0x5f3759df - ( conv.i >> 1 );
	conv.f  *= threehalfs - ( x2 * conv.f * conv.f );
	return conv.f;
}

float *klfilt(float *sensor){
float orig_val, upd_val, prev_val;
float curr_pk,curr_xk;
float prev_pk,prev_xk;
float init_pk = 1;
float init_xk = 0;
float Kk; 
float std = stdev(sensor);

static float filtval[SIZE];

int i = 0;
for(i= 0; i<SIZE;i++){
    orig_val = sensor[i];
    prev_pk = init_pk;
    prev_xk = init_xk;
    Kk = prev_pk/(prev_pk+std);
    upd_val = prev_xk + (Kk*(orig_val-prev_xk));

    curr_pk = (1-Kk)*prev_pk;
    curr_xk = upd_val;
    filtval[i] = upd_val;

    init_pk = curr_pk;
    init_xk = curr_pk;

}



return filtval;
}

float stdev(float *data){
    int length = sizeof(data) / sizeof(float);
    int i = 0;
    float sum = 0;
    float avg;
    for(i = 0; i < length; i++){
        sum += data[i];
    }
    avg = sum/length;
    float vsum = 0;
    for(i = 0; i < length; i++){
        vsum += (data[i]-avg)*(data[i]-avg);
    }
    float variance =vsum/length;
    float std = 1/invSqrt(variance);

    return std;
}
