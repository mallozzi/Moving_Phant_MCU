
#include "MathUtil.h"

//int16_t MultiplyByFraction(int16_t input, int16_t numerator, int16_t denominator) {
//    // Multiplies the input by the fraction numerator/denominator. Uses hardware implementations,
//    // which are extremely fast.
//    
//    signed long prod = __builtin_mulss(input, numerator);
//    int16_t result = __builtin_divsd(prod, denominator);
//    return result;
//}

int16_t MultiplyByFraction(int16_t input, int16_t numerator, int16_t denominator) {
    // Multiplies the input by the fraction numerator/denominator. Uses hardware implementations,
    // which are extremely fast.
    
    signed long prod = __builtin_mulss(input, numerator);
    signed long prod1 = prod + __builtin_divsd(denominator, 2);    // Adding half the denominator amounts to rounding the result to nearest integer after final divide
    int16_t result = __builtin_divsd(prod1, denominator);
    return result;
}
