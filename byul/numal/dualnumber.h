/**
 * @file dualnumber.h
 * @brief DualNumber structure for forward-mode automatic differentiation.
 *
 * DualNumber is represented as a + b * eps, where eps is an infinitesimal value.
 * The key property is eps^2 = 0. Therefore, all terms above eps^2 vanish.
 *
 * Core concepts:
 *   - re: real part (function value)
 *   - du: dual part (derivative value)
 *
 * Example:
 *   For f(x) = x^3, with x = 2.0 + 1*eps,
 *   f(x + eps) = 8 + 12*eps  -> Here 12 is f'(2) = 3 * 2^2.
 *
 * Usage example:
 *   dualnumber_t* x = dualnumber_create_full(2.0f, 1.0f); // eps coefficient is 1
 *   dualnumber_t* y = dualnumber_powf(x, 3.0f);           // f(x) = x^3
 *   printf("f(x)=%.1f, f'(x)=%.1f\n", y->re, y->du);      // Result: 8.0, 12.0
 *
 * This structure is useful for optimization, physics simulations,
 * and solving differential equations. It can also be used for numeric
 * analysis before GPU acceleration.
 */
#ifndef DUALNUMBER_H
#define DUALNUMBER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_config.h"
#include <math.h>

// DualNumber: a + b * eps, where eps^2 = 0
typedef struct {
    float re;  // real part
    float du;  // dual part
} dualnumber_t;

BYUL_API void dualnumber_init(dualnumber_t* out);

BYUL_API void dualnumber_init_full(dualnumber_t* out, float re, float du);

BYUL_API void dualnumber_assign(dualnumber_t* out, const dualnumber_t* src);

BYUL_API bool dualnumber_equal(const dualnumber_t* a, const dualnumber_t* b);

BYUL_API unsigned int dualnumber_hash(const dualnumber_t* a);

BYUL_API void dualnumber_neg(dualnumber_t* out, const dualnumber_t* a);

BYUL_API void dualnumber_add(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b);

BYUL_API void dualnumber_sub(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b);

BYUL_API void dualnumber_mul(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b);

BYUL_API void dualnumber_div(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b);

BYUL_API void dualnumber_scale(dualnumber_t* out, 
    const dualnumber_t* a, float s);

BYUL_API void dualnumber_invscale(dualnumber_t* out, 
    const dualnumber_t* a, float s);

BYUL_API void dualnumber_powf(dualnumber_t* out, 
    const dualnumber_t* a, float n);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // DUALNUMBER_H
