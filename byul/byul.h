// byul.h
//
// Copyright (c) 2025 ByulPapa (byuldev@outlook.kr)
// This file is part of the Byul World project.
// Licensed under the Byul World Public License v1.0
// See the LICENSE file for details.

#ifndef BYUL_H
#define BYUL_H

#include "byul_common.h"
#include "navsys.h"
#include "balix.h"
#include "entity.h"
#include "projectile.h"

#ifdef __cplusplus
extern "C" {
#endif

BYUL_API const char* byul_version_string();
BYUL_API void byul_print_version();

#ifdef __cplusplus
}
#endif

#endif // BYUL_H
