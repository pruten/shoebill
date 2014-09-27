#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "softfloat.h"

floatx80 f[8];

int main (void)
{
    float128 a = int64_to_float128(123);
    float128 b = int64_to_float128(123);
    float128 c = float128_mul(a, b);
    
    int32_t c_int = float128_to_int32(c);
    
    
    printf("123 * 123 = %d (expected %d)\n", c_int, 123 * 123);
    
    printf("%f\n", a);
    
    
    return 0;
}