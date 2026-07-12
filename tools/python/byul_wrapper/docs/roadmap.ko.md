# BYUL DLL 전체 저수준 Wrapper 로드맵

## 최종 목표

BYUL shared library의 지원 대상 `BYUL_API` 전체를 Python의 `c_*` 객체와 함수로
호출할 수 있게 합니다.

```python
from byul_wrapper.coord import c_coord
from byul_wrapper.navgrid import c_navgrid
from byul_wrapper.route import c_route
```

완료 범위는 C 객체와 직접 대응하는 저수준 wrapper까지입니다. 별도의 `Coord`,
`NavGrid` 같은 고수준 타입이나 단일 facade class는 목표에 포함하지 않습니다.

## 현재 범위

- c_coord, c_coord_list, c_coord_hash
- c_navgrid
- c_route
- c_route_finder와 일부 정적 탐색 알고리즘
- c_dstar_lite와 관련 priority queue
- c_tick
- 일부 console debug API

BYUL에는 `BYUL_API`를 포함하는 public header가 약 85개 있으며 다음 영역이 남아
있습니다.

- navsys 미지원 모듈
- numal
- number_theory
- entity
- balix
- ground
- aerial
- projectile
- rng
- core utility

## 단계

### 1단계: 기존 wrapper 안정화

- 공통 native object 수명 패턴 정리
- owned/borrowed/transferred pointer 검증
- callback handle 수명 공통 처리
- close 이후 pointer 접근 차단
- ABI header와 DLL symbol 검사
- 알려진 navgrid와 D* Lite 수명 오류 수정

### 2단계: navsys 완성

- c_navcell
- maze 모듈
- route finder 전체
- D* Lite tick
- 관련 container와 priority queue

### 3단계: 기반 모듈

- numal
- number_theory
- rng
- core container와 string API

### 4단계: entity와 물리 모듈

- entity
- balix
- ground
- aerial
- projectile

### 5단계: 배포 안정화

- Windows/Linux/macOS ABI test
- wheel과 shared library 배포 자동화
- public symbol coverage 자동 검사
- `c_*` API reference 생성

## 모듈 완료 기준

- 지원 public symbol 목록이 기록됨
- C header와 DLL signature가 검증됨
- 대응하는 `c_*` wrapper가 존재함
- create/copy/get/fetch 소유권이 검증됨
- NULL과 native 오류 처리가 정의됨
- callback 수명과 실행 thread가 검증됨
- 단위 테스트와 통합 테스트가 통과함
- 반복 생성·해제에서 누수가 검증됨
- 최소 사용 예제가 문서화됨

## 범위 밖

- 고수준 Python 값 객체
- `Byul` 단일 facade
- BYUL Grid 전용 World/NPC/GUI 객체
- C ABI 의미를 숨기는 도메인 서비스
