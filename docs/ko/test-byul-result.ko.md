# test_byul 실행 결과와 논리 점검 기록

이 문서는 `build_debug/bin/test_byul` 실행 결과를 확인하고, 테스트는 통과했지만
논리 검증이 약하거나 출력 해석에 주의가 필요한 부분을 남긴 기록입니다.

## 확인 일시와 실행 대상

```text
확인일: 2026-07-05
작업 위치: /home/sajang/docs/byul_env
실행 파일: build_debug/bin/test_byul
```

실행 결과 요약:

```text
test cases: 395 | 395 passed | 0 failed
assertions: 1317 | 1317 passed | 0 failed
Status: SUCCESS
exit code: 0
```

따라서 현재 `test_byul` 전체 실행은 실패 없이 끝납니다.

## 주의해야 할 논리 검증 항목

### projectile static target hit

파일:

```text
byul/projectile/tests/test_module_projectile_predict.cpp
```

`projectile_predict - static target hit` 단일 테스트는 성공하지만, 출력과 검증
조건이 서로 맞지 않아 보입니다.

단일 실행 출력:

```text
impact time : 10.058223, impact pos : (0.000, 101.000, 0.000)
test cases: 1 | 1 passed | 0 failed | 394 skipped
assertions: 4 | 4 passed | 0 failed
```

테스트의 target 위치는 `{0, 100, 0}`인데, 검증은 `impact_pos.y`가 `0.0` 근처인지
확인합니다. 또한 `doctest::Approx(0.0f).epsilon(1.0f)`는 허용 범위가 너무 넓어
의도와 다른 값도 통과시킬 수 있습니다.

해석:

- ground collision 테스트라면 `impact_pos.y`가 `0.0` 근처인지 보는 것이 자연스럽다.
- static target hit 테스트라면 `impact_pos`가 target 위치 또는 target collision
  반경 안인지 확인하는 것이 자연스럽다.
- 현재 출력의 `impact pos : (0.000, 101.000, 0.000)`은 target hit 결과로는
  납득 가능하지만, `y ~= 0` 검증 조건과는 맞지 않는다.

권장 정리:

- static target hit 테스트는 `impact_pos.y`를 target 높이 근처로 검증한다.
- target collision 반경을 쓰는 경우, target 중심과 impact 위치의 거리를 검사한다.
- `epsilon(1.0f)` 대신 절대 오차 기반의 더 좁은 허용치를 사용한다.

### D* Lite find_loop 테스트 출력

파일:

```text
byul/navsys/dstar_lite/tests/test_module_dstar_lite.cpp
```

`test_dstar_lite_find_loop`는 성공하지만, 동적 경로 검증 출력이 실제 `real_route`를
보여 주지 못하는 구간이 있습니다.

문제 지점:

```text
coord_list_t* changed_coords = NULL;
...
coord_list_push_back(changed_coords, coord_i);
...
CHECK(dsl->real_route);
route_print(p);
dsl_print_ascii_update_count(dsl, p, 5);
```

해석:

- `changed_coords`가 `NULL`인 상태에서 `coord_list_push_back`을 호출하므로 변경
  좌표가 실제 목록에 들어가지 않는다.
- `dsl->real_route`를 검사하면서도 출력은 `p`를 사용한다.
- 이 시점의 `p`는 앞에서 `route_destroy(p); p = NULL;` 처리된 값이라 동적 경로
  출력이 의미 있게 남지 않는다.

단일 실행 결과는 통과하지만, 출력은 다음처럼 동적 장애물 추가 메시지만 보이고
실제 재계산 경로 확인은 부족합니다.

```text
interval sec : 0.100, dstar_lite_find_loop() cretes dynamic routes.
blocked (4, 5)
...
test cases: 1 | 1 passed | 0 failed | 394 skipped
assertions: 10 | 10 passed | 0 failed
```

권장 정리:

- `changed_coords = coord_list_create();`로 목록을 먼저 만든다.
- 동적 경로 출력과 검증에는 `dsl->real_route`를 사용한다.
- `route_get_success(dsl->real_route)` 또는 route 길이/마지막 좌표가 goal인지까지
  검사한다.
- 출력 문자열의 `cretes` 오탈자는 `creates`로 고친다.

### D* Lite find_dynamic 변경 좌표 목록

같은 파일의 `test_dstar_lite_find_dynamic`에도 유사한 패턴이 있습니다.

```text
coord_list_t* changed_coords = NULL;
...
coord_list_push_back(changed_coords, &coord_i);
dsl->changed_coords_fn_userdata = changed_coords;
```

`changed_coords`를 생성하지 않은 상태에서 push를 호출하므로, 변경 좌표 전달이
실제로 이뤄지지 않을 수 있습니다. 동적 변경을 테스트하려면 목록 생성 후 좌표를
넣고, callback이 그 목록을 복사해서 반환하는지 확인해야 합니다.

## 결론

현재 `test_byul`은 전체 성공으로 종료됩니다. 다만 일부 테스트는 실제 동작이
정상인지 충분히 강하게 검증하지 못합니다. 특히 projectile target hit 검증 조건과
D* Lite 동적 경로 테스트는 테스트가 통과하더라도 논리 오류를 가릴 수 있으므로
후속 수정 대상으로 분리하는 것이 좋습니다.
