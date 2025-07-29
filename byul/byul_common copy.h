#ifndef BYUL_COMMON_H
#define BYUL_COMMON_H

// -----------------------------
// Platform Detection
// -----------------------------
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

// -----------------------------
// DLL Export/Import Macros
// -----------------------------
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
    #define BYUL_API
  #endif
#endif

// -----------------------------
// Version Information
// -----------------------------
#define BYUL_VERSION_MAJOR 0
#define BYUL_VERSION_MINOR 1
#define BYUL_VERSION_PATCH 0
#define BYUL_VERSION "0.1.0"

// -----------------------------
// Debug Mode
// -----------------------------
#ifdef DEBUG
  #define BYUL_DEBUG 1
#else
  #define BYUL_DEBUG 0
#endif

// -----------------------------
// String Macros
// -----------------------------
#define BYUL_STRINGIFY(x) #x
#define BYUL_TOSTRING(x) BYUL_STRINGIFY(x)

// -----------------------------
// Debug Print Macro
// -----------------------------
#if BYUL_DEBUG
  #include <stdio.h>
  #define DBG_PRINT(...) printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...) ((void)0)
#endif

#endif // BYUL_COMMON_H
