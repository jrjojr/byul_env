/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file navsys_status.h
 * @brief Navsys public C ABI의 공통 상태 값을 선언한다.
 *
 * 경로 탐색, collection, callback과 장기 실행 operation이 공유하는 성공, 예상 결과와
 * 오류 상태의 안정된 숫자 계약을 제공한다.
 */

#ifndef BYUL_NAVSYS_STATUS_H
#define BYUL_NAVSYS_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Navsys public operation의 공통 상태 값이다.
 *
 * 0은 성공이고 양수는 count 또는 일반 result용으로 예약한다. 0이 아닌 상태는 모두
 * 음수이며, 함수별 문서가 partial output을 허용하지 않으면 모든 출력은 보존된다.
 */
typedef enum e_navsys_status {
    NAVSYS_STATUS_OK = 0,
    NAVSYS_STATUS_INVALID_ARGUMENT = -1,
    NAVSYS_STATUS_OUT_OF_MEMORY = -2,
    NAVSYS_STATUS_UNSUPPORTED = -3,
    NAVSYS_STATUS_CALLBACK_FAILED = -4,
    NAVSYS_STATUS_CORRUPT_STATE = -5,
    NAVSYS_STATUS_NOT_FOUND = -6,
    NAVSYS_STATUS_INVALIDATED = -7,
    NAVSYS_STATUS_NO_PATH = -8,
    NAVSYS_STATUS_CANCELLED = -9,
    NAVSYS_STATUS_LIMIT_REACHED = -10,
    NAVSYS_STATUS_INCOMPLETE = -11,
    NAVSYS_STATUS_IN_PROGRESS = -12
} navsys_status_t;

#ifdef __cplusplus
}
#endif

#endif /* BYUL_NAVSYS_STATUS_H */
