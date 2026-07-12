# BYUL Python tools

BYUL의 Python 프로젝트는 이 디렉터리에 모읍니다.

```text
tools/python/
├─ .venv/          공용 가상환경(생성 파일, Git 제외)
├─ byul_wrapper/   BYUL shared library 독립 Python 바인딩
├─ byul_grid/      PySide6 기반 시각화 애플리케이션
├─ requirements.txt
├─ requirements-dev.txt
└─ requirements-package.txt
```

Windows 환경 설정:

```bat
tools\python\setup_env.bat
```

Linux 또는 macOS:

```sh
tools/python/setup_env.sh
```

두 프로젝트는 `tools/python/.venv` 하나를 공유합니다. 프로젝트별 실행 스크립트도
이 환경을 우선 사용합니다.

의존성 파일도 이 디렉터리에서 공용으로 관리합니다. 개별 애플리케이션 폴더에는
별도의 requirements 파일을 두지 않습니다.
