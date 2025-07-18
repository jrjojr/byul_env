#ifndef BYUL_CONFIG_H
#define BYUL_CONFIG_H

// ─────────────────────────────
// 플랫폼 감지
// ─────────────────────────────
#if defined(_WIN32) || defined(_WIN64)
  #define BYUL_PLATFORM_WINDOWS 1
#else
  #define BYUL_PLATFORM_WINDOWS 0
#endif

#if defined(__linux__)
  #define BYUL_PLATFORM_LINUX 1
#else
  #define BYUL_PLATFORM_LINUX 0
#endif

#if defined(__APPLE__)
  #define BYUL_PLATFORM_MACOS 1
#else
  #define BYUL_PLATFORM_MACOS 0
#endif

// ─────────────────────────────
// DLL Export/Import 매크로
// ─────────────────────────────
// BYUL_STATIC이 정의되어 있으면 모든 export macro 제거됨
#if defined(BYUL_STATIC)
  #define BYUL_API
#else
  #if BYUL_PLATFORM_WINDOWS
    #ifdef BYUL_EXPORTS
      #define BYUL_API __declspec(dllexport)
    #else
      #define BYUL_API __declspec(dllimport)
    #endif
  #else
    #define BYUL_API __attribute__((visibility("default")))
  #endif
#endif

// ─────────────────────────────
// 버전 정보
// ─────────────────────────────
#define BYUL_VERSION_MAJOR 0
#define BYUL_VERSION_MINOR 1
#define BYUL_VERSION_PATCH 0
#define BYUL_VERSION "0.1.0"

// ─────────────────────────────
// 디버그 여부
// ─────────────────────────────
#ifdef DEBUG
  #define BYUL_DEBUG 1
#else
  #define BYUL_DEBUG 0
#endif

// ─────────────────────────────
// 문자열 매크로
// ─────────────────────────────
#define BYUL_STRINGIFY(x) #x
#define BYUL_TOSTRING(x) BYUL_STRINGIFY(x)

// ─────────────────────────────
// 디버깅 출력 매크로
// ─────────────────────────────
#if BYUL_DEBUG
  #include <stdio.h>
  #define DBG_PRINT(...) printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...) ((void)0)
#endif

// ─────────────────────────────
// 16바이트 정렬 매크로 (SIMD / GPU 등)
// ─────────────────────────────
#if defined(_MSC_VER)
  #define BYUL_ALIGN_16 __declspec(align(16))
#elif defined(__GNUC__) || defined(__clang__)
  #define BYUL_ALIGN_16 __attribute__((aligned(16)))
#else
  #define BYUL_ALIGN_16
#endif

#endif // BYUL_CONFIG_H
