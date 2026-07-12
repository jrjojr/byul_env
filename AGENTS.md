# AGENTS.md instructions

이 저장소는 BYUL 프로젝트입니다.

작업을 시작할 때는 먼저 다음 BYUL 전용 규칙 문서를 읽고, 그 기준을 따른다.

```text
docs/ko/byul-project-rules.ko.md
```

## 작업별 필수 규칙 문서

Public C header를 생성하거나 수정할 때는 작업 전에 다음 문서를 모두 읽고 따른다.

```text
docs/ko/byul-sdk/header-declaration-rules.org
docs/ko/byul-sdk/header-comment-rules.org
```

표준 라이선스 블록은 다음 원본을 그대로 사용한다.

```text
docs/ko/byul-sdk/header-license-block.txt
```

Python wrapper를 생성하거나 수정할 때는 작업 전에 다음 문서를 읽고 따른다.

```text
tools/python/byul_wrapper/docs/wrapper-rules.org
```

Wrapper generator, CFFI 선언, SDK header parser 또는 ABI 자동 생성 작업은 header
선언과 comment를 함께 다루므로 위의 header 규칙 문서도 모두 확인한다.

## 크로스 플랫폼 환경

BYUL은 Windows, Linux, macOS에서 사용하는 크로스 플랫폼 프로젝트다. 현재 개발자의
주요 로컬 환경은 Windows 11이지만, 경로·명령·구현을 Windows 전용으로 고정하지 않는다.

작업 전 현재 운영체제와 shell을 확인하고 해당 플랫폼의 경로 형식과 도구를 사용한다.
공통 코드에서는 플랫폼별 경로 구분자, shared library 이름과 실행 파일 확장자를
직접 가정하지 않는다.

## Python 환경

Python 실행 파일은 다음 순서로 확인한다.

1. `BYUL_PYTHON` 환경 변수
2. `tools/python/.venv`의 해당 플랫폼 interpreter
3. 플랫폼 기본 Python 3 명령

플랫폼별 가상환경 interpreter는 다음과 같다.

```text
Windows: tools/python/.venv/Scripts/python.exe
Linux/macOS: tools/python/.venv/bin/python
```

### Windows 11 로컬 환경

Windows 11에서는 다음 Python 3.13 설치 경로도 우선 확인한다.

```text
C:\Users\critl\AppData\Local\Programs\Python\Python313\python.exe
```

Windows에서 `python`이 PATH에 없다는 이유만으로 Python이 설치되지 않았다고 판단하지
않는다. 다음 후보를 확인한다.

1. `BYUL_PYTHON` 환경 변수
2. 공용 가상환경 interpreter
3. 위의 Python 3.13 절대 경로
4. `py -3`
5. `python`

BYUL Python 도구의 공용 가상환경은 다음 위치를 사용한다.

```text
tools/python/.venv
```
