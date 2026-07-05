/* byul_compiler.h */
#ifndef BYUL_COMPILER_H
#define BYUL_COMPILER_H

/* std headers */
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* inline */
#if defined(_MSC_VER)
  #define BYUL_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
  #define BYUL_INLINE __attribute__((always_inline)) inline
#else
  #define BYUL_INLINE inline
#endif

/* restrict (AVR-GCC도 지원하지만 혹시 모를 대비) */
#if defined(_MSC_VER)
  #define BYUL_RESTRICT __restrict
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
  #define BYUL_RESTRICT restrict
#else
  #define BYUL_RESTRICT
#endif

/* Likely/unlikely (optional) */
#if defined(__GNUC__) || defined(__clang__)
  #define BYUL_LIKELY(x)   __builtin_expect(!!(x), 1)
  #define BYUL_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
  #define BYUL_LIKELY(x)   (x)
  #define BYUL_UNLIKELY(x) (x)
#endif

#endif
