import locale
import time
from datetime import datetime
from cx_Freeze import setup, Executable

import sys

sys.path.append("cmake_starter_1039/py_modules")


# 로케일과 시간 설정
locale.setlocale(locale.LC_ALL, 'ko_KR.UTF-8')  # 한국 로케일 설정
current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')  # 현재 시간 포맷

# 포함할 모듈과 제외할 모듈
# 모듈 이름만 적는다 
# 디렉토리는 sys.path에 추가되어야 한다.
build_exe_options = {
    "includes": [
      "cmake_lists_maker",
      'encoding_converter',
      'ext_project_adder'
      ],  # 필요한 모듈 추가
    "include_files": [
        "LICENSE",
        ("cmake_starter_1039/configure_file_in/", "configure_file_in"),
        ("cmake_starter_1039/starter.cmake", "starter.cmake"),
        ('cmake_starter_1039/resources', 'resources'),
        ],      

    # "excludes": ["module_name_to_exclude"],  # 제외할 모듈
    "optimize": 2,  # 최적화 수준 설정,    
    "build_exe": "build/cmake_starter"
    
}

print(f"현재 로케일: {locale.getlocale()}")
print(f"빌드 시작 시간: {current_time}")

setup(
    name="cmake_starter",
    version="1.0",
    license="Proprietary",  # 예: MIT, GPLv3, Apache-2.0 등
    description="creates CMakeLists.txt",
    options={"build_exe": build_exe_options},
    executables=[Executable("cmake_starter_1039/py_modules/cmake_starter.py")]
)