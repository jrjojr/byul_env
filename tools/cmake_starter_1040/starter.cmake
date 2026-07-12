#############################################################################
# Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
# All Rights Reserved.
#
# cmake_starter starter.cmake
#
#############################################################################

# 프로젝트 이름을 대문자로 변환한다.
string(TOUPPER ${PROJECT_NAME} UPPER_CASE_PROJECT_NAME)

# 헤더의 접미어를 설정한다.
set(HEADER_SUFFIX "h")

# 2024-08-03 12:54:48 home 폴더를 알아낸다.
# ${HOME}으로 경로를 알아낸다.
file(REAL_PATH "~" HOME EXPAND_TILDE)

macro(addSub sub)
  if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${sub}/CMakeLists.txt)  
    message ("${sub}/CMakeLists.txt is not exists.")
    set(SUB_PROJECT_NAME ${sub})
    string(TOUPPER ${SUB_PROJECT_NAME} UPPER_CASE_SUB_PROJECT_NAME)

    # sub의 CMakeLists.txt를 생성한다.
    set(args maker -bt --rel_pwd ${sub} ${sub})
    execute_process(COMMAND cmake_starter.bat ${args}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()

  add_subdirectory(${sub})
  string(JOIN "/" aDir ${CMAKE_CURRENT_SOURCE_DIR} ${sub})
  get_name_from_dir(${aDir} aSubName)

  unset(SUB_PROJECT_NAME)
  unset(UPPER_CASE_SUB_PROJECT_NAME)

  is_target_available(${PROJECT_NAME} ret)
  if(ret)
    # if()
    # sub를 자식으로 추가한다.
    message(STATUS "${PROJECT_NAME} add ${aSubName} as child")

    # 자식은 셀프에 링크시키지 않는다.
    # self의 자식들은 self가 빌드할 때에는 필요 없지만
    # 상위 폴더와 형제들 한테는 필요하다.
    # 이 말은 다른 프로젝트에서 링크하니까 정보를 글로발로 저장해야 한다.
    # set() 함수와 CACHE를 사용해서 저장한다.
    # list()는 지역변수라서 글로발로는 저장이 안된다.
    # children은 글로발로 저장해야 겠다.
    # 그래야 다른 프로젝트에서 셀프의 자식들을 확인 할 수 있다.
    # 캐시 저장을 사용 한다는 것이다.
    # 캐시 저장은 리스트를 사용하지 못한다.
    # set으로만 리스트의 효과를 얻어야 한다.
    # list(APPEND ${PROJECT_NAME}_CHILDREN ${sub})
    set(aTempChildren $CACHE{${PROJECT_NAME}_CHILDREN})
    if(aSubName IN_LIST aTempChildren)
      # aSubName가 self의 자식으로 이미 등록되어 있으니까 아무 것도 하지 않는다.
      message("${aSubName} is already registered to ${PROJECT_NAME}")
    else()
      # aSubName를 self의 자식으로 등록한다.
      message("${aSubName} is not registered to ${PROJECT_NAME}")

      set(${PROJECT_NAME}_CHILDREN ${${PROJECT_NAME}_CHILDREN};${aSubName} 
        CACHE STRING "children of ${PROJECT_NAME}" FORCE)

      # 자식이 빌드할 때에는 부모를 링크해야 한다.        
      # self가 자식이면 부모이름을 찾아야 하는데
      # cmake는 부모 소스가 진행중에 자식이 추가되어서,
      # 부모 소스가 완료되지 않았다.
      # 소스가 완료되지 않으면
      # 프로젝트 이름을 확인하지 못한다.
      # 부모이름을 찾지 못한다.
      # 수동으로 입력해야 한다.
      # 자식들의 캐시에 부모이름을 저장해야 한다.
      set(${aSubName}_PARENT ${PROJECT_NAME} 
        CACHE STRING "parent of ${aSubName}" FORCE)      
    endif()

  else()
    # 서브들만 셀프에 링크한다.
    message(STATUS "${PROJECT_NAME} add ${aSubName} as sub")

    list(APPEND ${PROJECT_NAME}_SUBS ${aSubName})

  endif()

  message(STATUS "${PROJECT_NAME} add서브 완료")
endmacro() #addSub

macro(add_tester)
  create_tester()
  add_subdirectory(tester)
  string(JOIN "/" aDir ${CMAKE_CURRENT_SOURCE_DIR} tester)
  get_tester_name_from_dir(${aDir} aSubName)

  set(aTempChildren $CACHE{${PROJECT_NAME}_CHILDREN})
  if(aSubName IN_LIST aTempChildren)
    # ${aSubName}가 self의 자식으로 이미 등록되어 있으니까 아무 것도 하지 않는다.
    message("${aSubName} is already registered to ${PROJECT_NAME}")
  else()
    # ${aSubName}를 self의 자식으로 등록한다.
    message("${aSubName} is not registered to ${PROJECT_NAME}")

    set(${PROJECT_NAME}_CHILDREN ${${PROJECT_NAME}_CHILDREN};${aSubName}
      CACHE STRING "children of ${PROJECT_NAME}" FORCE)

    # 자식이 빌드할 때에는 부모를 링크해야 한다.        
    # self가 자식이면 부모이름을 찾아야 하는데
    # cmake는 부모 소스가 진행중에 자식이 추가되어서,
    # 부모 소스가 완료되지 않았다.
    # 소스가 완료되지 않으면
    # 프로젝트 이름을 확인하지 못한다.
    # 부모이름을 찾지 못한다.
    # 수동으로 입력해야 한다.
    # 자식들의 캐시에 부모이름을 저장해야 한다.
    set(${aSubName}_PARENT ${PROJECT_NAME} 
      CACHE STRING "parent of ${aSubName}" FORCE)
  endif()

endmacro() #add_tester

macro(addExt srcDir)
  # srcDir 폴더를 링크시킨다.
  # if (srcDir/CMakeLists.txt.is_exist()):
    # 심링크를 생성한다.
    # ext 폴더를 addSub한다.
    addSub(ext)
  # else:
      # srcDir이 cmake 프로젝트 폴더가 아니다.
      # 종료한다.
  # endif()
endmacro() #addExt

macro(addSrc filePath)
  list(APPEND ${PROJECT_NAME}_SRCS ${filePath})
endmacro() # macro(addSrc filePath)

macro(addHeader filePath)
  list(APPEND ${PROJECT_NAME}_HEADERS ${filePath})
endmacro() # macro(addHeader filePath)

macro(addIncDir dirPath)
  list(APPEND ${PROJECT_NAME}_INC_DIRS ${dirPath})
endmacro() # addIncDir

macro(addLibDir dirPath)
  list(APPEND ${PROJECT_NAME}_LIB_DIRS ${dirPath})
endmacro() # addLibDir

macro(addLib libName)
  list(APPEND ${PROJECT_NAME}_LIBS ${libName})
endmacro() # addLibDir

macro(addCommandWhenBuildExcute tCommand tArgv)
  list(APPEND 
    ${PROJECT_NAME}_WHEN_${PROJECT_NAME}_BUILD_EXECUTED_COMMANDS COMMAND)

  list(APPEND 
    ${PROJECT_NAME}_WHEN_${PROJECT_NAME}_BUILD_EXECUTED_COMMANDS ${tCommand})

  list(APPEND 
    ${PROJECT_NAME}_WHEN_${PROJECT_NAME}_BUILD_EXECUTED_COMMANDS ${tArgv})
endmacro() # addCommandWhenBuildExcute

macro(create_base)
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  # 루트에만 적용되는 코드를 작성한다.
  # 대부분이 파일을 생성하는 작업이다.

  # 설치를 위해 CPACK을 사용하고, 옵션을 설정한다.
  if (NOT EXISTS "${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_cpack_config.cmake")
    configure_file (
    "${ROOT}/configure_file_in/cpack_config.cmake.in"
    "${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_cpack_config.cmake"
    @ONLY)
    message("${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_cpack_config.cmake creates.")
  endif()

  set (CPACK_PROJECT_CONFIG_FILE
  "${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_cpack_config.cmake"
  )

  # CPACK에서 사용하는 resources 파일들을 복사한다.
  if (NOT EXISTS "${CMAKE_BINARY_DIR}/resources/ico/${PROJECT_NAME}.ico")
    file(COPY 
    "${ROOT}/resources/ico/nsis_mui.ico"    
      DESTINATION "${CMAKE_BINARY_DIR}/resources/ico"
      FOLLOW_SYMLINK_CHAIN)
    file(RENAME "${CMAKE_BINARY_DIR}/resources/ico/nsis_mui.ico" 
      "${CMAKE_BINARY_DIR}/resources/ico/${PROJECT_NAME}.ico" )
    message("${CMAKE_BINARY_DIR}/resources/ico/${PROJECT_NAME}.ico creates.")
  endif()

  if (NOT EXISTS "${CMAKE_BINARY_DIR}/resources/bmp/${PROJECT_NAME}.bmp")
    file(COPY 
  "${ROOT}/resources/bmp/nsis_header_150x57.bmp"
      DESTINATION "${CMAKE_BINARY_DIR}/resources/bmp"
      FOLLOW_SYMLINK_CHAIN)      
    file(RENAME "${CMAKE_BINARY_DIR}/resources/bmp/nsis_header_150x57.bmp" 
      "${CMAKE_BINARY_DIR}/resources/bmp/${PROJECT_NAME}.bmp" )
    message("${CMAKE_BINARY_DIR}/resources/bmp/${PROJECT_NAME}.bmp creates.")
  endif()

  # license.txt를 생성한다.
  if (NOT EXISTS "${CMAKE_BINARY_DIR}/resources/txt/license.txt")
    configure_file(
      ${ROOT}/configure_file_in/license.txt.in 
      "${CMAKE_BINARY_DIR}/resources/txt/license.txt" )
    message("${CMAKE_BINARY_DIR}/resources/txt/license.txt creates.")
  else()
    # message("${CMAKE_BINARY_DIR}/resources/txt/license.txt is exists.")
  endif()        

  # release_notes.org가 없으면 추가한다.
  if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/release_notes.org)
  message("${CMAKE_CURRENT_SOURCE_DIR}/release_notes.org creates.")
    configure_file(
      ${ROOT}/configure_file_in/release_notes.org.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/release_notes.org )
  else()
    # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h) is exists.")
  endif()        
    # ${PROJECT_NAME}.cmake 파일 생성

endif() # endif (${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})

  # 프로젝트에 기본적으로 필요한 ${PROJECT_NAME}_config.h 파일을 생성한다.
  if (NOT EXISTS "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_config.h")
    configure_file(
      ${ROOT}/configure_file_in/config.h.in 
      ${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_config.h)
    message("${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_config.h creates.")
  endif()

  if (${PROJECT_NAME}_PROJECT_TYPE STREQUAL "cpp_main")
  create_main_cpp()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "c_main")
  create_main_c()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "lib")
    create_lib()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "dll")
    create_dll()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex")
    create_flex()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison")
    create_bison()
    create_flex()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison_main")
  create_bison()
  create_flex()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex_main")
    create_flex()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "libpp")
    create_libpp()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "dllpp")
    create_dllpp()

  else()
  message("${PROJECT_NAME} TYPE : invalid")
  message("create default project type base.")

  endif()
endmacro() # macro(create_main)

macro(create_main_cpp)
    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
    message("${CMAKE_CURRENT_SOURCE_DIR}/main.cpp) is creates.")
    configure_file(
      ${ROOT}/configure_file_in/main.cpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
    else()
      # message("${CMAKE_CURRENT_SOURCE_DIR}/main.cpp) is exists.")
    endif()        

    addSrc(main.cpp)
endmacro() # endmacro(create_main_cpp)

macro(create_main_c)
    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/main.c)
    message("${CMAKE_CURRENT_SOURCE_DIR}/main.c) is creates.")
    configure_file(
      ${ROOT}/configure_file_in/main.c.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/main.c)
    else()
      # message("${CMAKE_CURRENT_SOURCE_DIR}/main.c) is exists.")
    endif()        

    addSrc(main.c)
endmacro() # endmacro(create_main_c)

macro(create_lib)
    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h) is creates.")
      configure_file(
        ${ROOT}/configure_file_in/lib.h.in 
        ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h)
    else()
      # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h) is exists.")
    endif()    

    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is creates.")
    configure_file(${ROOT}/configure_file_in/lib.c.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c)
    else()
    # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is exists.")
    endif()        
    addSrc(${PROJECT_NAME}.c)
    addHeader(${PROJECT_NAME}.h)    
endmacro(create_lib)

macro(create_dll)
    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h) is creates.")
      configure_file(
        ${ROOT}/configure_file_in/lib.h.in 
        ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h)
    else()
      # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.h) is exists.")
    endif()    

    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is creates.")
    configure_file(
      ${ROOT}/configure_file_in/lib.c.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.c)
    else()
    # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is exists.")
    endif()        
    addSrc(${PROJECT_NAME}.c)
    addHeader(${PROJECT_NAME}.h)    
endmacro(create_dll)

macro(create_flex)
  # if not exitst file
  if (NOT EXISTS ${gFlexFilePath})
    message("${gFlexFilePath} is creates.")
    configure_file(
      # ${ROOT}/configure_file_in/flex.l.in
      ${ROOT}/configure_file_in/${gFlexIn}
      ${gFlexFilePath}
      )
  else()
    # message("${gFlexFilePath} is exists.")
  endif()    

  if ( (NOT EXISTS ${gFlexCFilePath}) OR 
       (NOT EXISTS ${gFlexHFilePath}) )

    cmake_path(GET gFlexCFilePath PARENT_PATH aParent)       
    if (NOT EXISTS ${aParent})
      message("${aParent} 폴더가 없어서 생성한다.")
      file(MAKE_DIRECTORY ${aParent})
    endif()

    cmake_path(GET gFlexHFilePath PARENT_PATH aParent)
    if (NOT EXISTS ${aParent})
      message("${aParent} 폴더가 없어서 생성한다.")
      file(MAKE_DIRECTORY ${aParent})  
    endif()
  
    # flex를 실행해서 헤더와 소스를 생산한다.
    message("${gFlexPath} ${FLEX_ARGS} 를 실행해서 헤더와 소스를 생산한다.")
    execute_process(COMMAND ${gFlexPath} ${gFlexArgs}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )  
  endif()

  addSrcs(    ${gFlexCFilePath}    )
  addHeaders(    ${gFlexHFilename}    )  

endmacro() # create_flex

macro(create_bison)
  # if not exitst file
  if (NOT EXISTS ${gBisonFilePath})
  message("${gBisonFilePath} is creates.")
    configure_file(
      # ${ROOT}/configure_file_in/bison.y.in 
      ${ROOT}/configure_file_in/${gBisonIn}
      ${gBisonFilePath})
  else()
    # message("${gBisonFilePath} is exists.")
  endif()      

  if ( (NOT EXISTS ${gBisonCFilePath}) OR 
       (NOT EXISTS ${gBisonHFilePath}) )

    cmake_path(GET gBisonCFilePath PARENT_PATH aParent)
    if (NOT EXISTS ${aParent})
      message("${aParent} 폴더가 없어서 생성한다.")
      file(MAKE_DIRECTORY ${aParent})
    endif()
     
    cmake_path(GET gBisonHFilePath PARENT_PATH aParent)
    if (NOT EXISTS ${aParent})
      message("${aParent}가 없어서 생성한다.")
      file(MAKE_DIRECTORY ${aParent})
    endif()
       
    # bison을 실행해서 헤더와 소스를 생산한다.
    message("${gBisonPath} ${BISON_ARGS} 를 실행해서 헤더와 소스를 생산한다.")
    execute_process(COMMAND ${gBisonPath} ${gBisonArgs}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )         

  endif()

  addSrcs(    ${gBisonCFilePath}    )
  addHeaders(    ${gBisonHFilename}  )
endmacro() # create_bison

macro(create_libpp)
    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp) is creates.")
      configure_file(
        ${ROOT}/configure_file_in/lib.hpp.in 
        ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp)
    else()
      # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp) is exists.")
    endif()    

    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is creates.")
    configure_file(
      ${ROOT}/configure_file_in/lib.cpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp)
    else()
    # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is exists.")
    endif()        
    addSrc(${PROJECT_NAME}.cpp)
    addHeader(${PROJECT_NAME}.hpp)    
endmacro(create_libpp)

macro(create_dllpp)
    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp) is creates.")
      configure_file(
        ${ROOT}/configure_file_in/lib.hpp.in 
        ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp)
    else()
      # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.hpp) is exists.")
    endif()    

    # if not exitst file
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp)
    message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is creates.")
    configure_file(
      ${ROOT}/configure_file_in/lib.cpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp)
    else()
    # message("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.cpp) is exists.")
    endif()        
    addSrc(${PROJECT_NAME}.cpp)
    addHeader(${PROJECT_NAME}.hpp)    
endmacro(create_dllpp)

macro(create_tester)
    # tester/CMakeLists.txt를 생성한다.
    if (NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tester/CMakeLists.txt)
message(" ${CMAKE_CURRENT_SOURCE_DIR}/tester/CMakeLists.txt is creates.")

      set(args maker --create_tester ${PROJECT_NAME})
      execute_process(COMMAND cmake_starter.bat ${args}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      )    
    endif()    
    
    # if not exitst tester/${PROJECT_NAME}_tester.hpp
    if (NOT EXISTS 
      ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.hpp)
    message(
" ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.hpp is creates.")    
      configure_file(
        ${ROOT}/configure_file_in/tester.hpp.in 
        ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.hpp)      
    else()
#     message(
# " ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.hpp is exists.")
    endif()

    # if not exitst tester/${PROJECT_NAME}_tester.cpp
    if (NOT EXISTS 
      ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp)
    
    message(
  " ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp is creates.")

      if(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex")
        configure_file(
        ${ROOT}/configure_file_in/flex_tester.cpp.in
          ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp)

      elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison")
        configure_file(
        ${ROOT}/configure_file_in/bison_tester.cpp.in
          ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp)

      elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "libpp")
        configure_file(
        ${ROOT}/configure_file_in/libpp_tester.cpp.in
          ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp)

      elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "dllpp")
        configure_file(
        ${ROOT}/configure_file_in/libpp_tester.cpp.in
          ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp)          

      else()
        configure_file(
          ${ROOT}/configure_file_in/tester.cpp.in
          ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp)
      endif()
    else()
  #     message(
# " ${CMAKE_CURRENT_SOURCE_DIR}/tester/${PROJECT_NAME}_tester.cpp is exists.")
    endif()
endmacro(create_tester)

macro(start_project type)
  # ${ARGV1}
  # ${ARGC}
  # message(STATUS "")
  message("${PROJECT_NAME} project type : ${type} start config.")
  # message("")
  
  set(${PROJECT_NAME}_PROJECT_TYPE ${type})

  if (${PROJECT_NAME}_PROJECT_TYPE STREQUAL "cpp_main")
    message("${PROJECT_NAME} TYPE : cpp_main")
    prepare_main_cpp()
    option(${PROJECT_NAME}_HAS_MAIN "${PROJECT_NAME} has main()" ON)

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "c_main")
    message("${PROJECT_NAME} TYPE : c_main")
    prepare_main_c()
    option(${PROJECT_NAME}_HAS_MAIN "${PROJECT_NAME} has main()" ON)

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "lib")
    message("${PROJECT_NAME} TYPE : lib")
    prepare_lib()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "dll")
    message("${PROJECT_NAME} TYPE : dll")
    prepare_dll()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex")
    message("${PROJECT_NAME} TYPE : flex")
    prepare_flex(${PROJECT_NAME} flex.l.in)

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison")
    message("${PROJECT_NAME} TYPE : bison")
    prepare_bison(${PROJECT_NAME} bison.y.in)
    prepare_flex(${PROJECT_NAME}_lexer bison.l.in)

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison_main")
    message("${PROJECT_NAME} TYPE : bison_main")
    prepare_bison(${PROJECT_NAME} bison_main.y.in)
    prepare_flex(${PROJECT_NAME}_lexer bison.l.in)
    option(${PROJECT_NAME}_HAS_MAIN "${PROJECT_NAME} has main()" ON)

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex_main")
    message("${PROJECT_NAME} TYPE : flex_main")
    prepare_flex(${PROJECT_NAME} flex_main.l.in)
    option(${PROJECT_NAME}_HAS_MAIN "${PROJECT_NAME} has main()" ON)

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "libpp")
    message("${PROJECT_NAME} TYPE : libpp")
    prepare_libpp()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "dllpp")
    message("${PROJECT_NAME} TYPE : dllpp")
    prepare_dllpp()

  else()
    message("${PROJECT_NAME} TYPE : invalid")
    message("use default project type.")

  endif()
  
  # ${CMAKE_INSTALL_BINDIR}, ${CMAKE_INSTALL_LIBDIR} 등을 사용한다.
  include(GNUInstallDirs)

  # cmake에 필요한 config파일들을 자동으로 작성하는 함수들을 호출한다.
  include(CMakePackageConfigHelpers)

  get_cur_inc_dirs(aRetIncDirs)
  list(APPEND ${PROJECT_NAME}_INC_DIRS ${aRetIncDirs})

  get_cur_lib_dirs(aRetLibDirs)
  list(APPEND ${PROJECT_NAME}_LIB_DIRS ${aRetLibDirs})
  
endmacro()

macro(do_project)
  if (${PROJECT_NAME}_PROJECT_TYPE STREQUAL "cpp_main")
  # message("${PROJECT_NAME} : end project cpp_main")
  do_main_cpp()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "c_main")
  # message("${PROJECT_NAME} : end project c_main")
  do_main_c()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "lib")
  # message("${PROJECT_NAME} : end project lib")
  do_lib()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "dll")
  # message("${PROJECT_NAME} : end project dll")
  do_dll()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex")
  # message("${PROJECT_NAME} : end project flex")
  do_flex()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison")
  # message("${PROJECT_NAME} : end project bison")
  do_bison()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex_main")
  # message("${PROJECT_NAME} : end project flex_main")
  do_flex_main()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison_main")
  # message("${PROJECT_NAME} : end project bison_main")
  do_bison_main()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "libpp")
  # message("${PROJECT_NAME} : end project libpp")
  do_libpp()

  elseif(${PROJECT_NAME}_PROJECT_TYPE STREQUAL "dllpp")
  # message("${PROJECT_NAME} : end project dllpp")
  do_dllpp()

  else()
  message("${PROJECT_NAME} : end project invalid")
  message("use default end project type.")

  endif()  

  set_target_properties(${PROJECT_NAME} PROPERTIES
  DEBUG_POSTFIX "_d"
  )

  get_target_property(target_type ${PROJECT_NAME} TYPE)

  # 라이브러리의 출력 디렉토리 설정
  set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY 
      "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}"
    RUNTIME_OUTPUT_DIRECTORY 
      "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}"
    ARCHIVE_OUTPUT_DIRECTORY 
      "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}"
  )

  set(aBuildType $<$<CONFIG:Debug>:DEBUG>)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ${aBuildType})
  # target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<CONFIG:Debug>:DEBUG>)  

  # tester를 추가한다.
  if(${PROJECT_NAME}_USE_TESTER)
    # 만약 프로젝트가 main을 갖고 있으면,
    # tester를 생성하지 않는다.
    if(NOT ${PROJECT_NAME}_HAS_MAIN)
      add_tester()
    endif()
  endif()

endmacro() # macro(do_project)

macro(end_project)
  # 빌드 시작할 때 외부 프로그램 실행
  add_custom_target(excute_before_build_${PROJECT_NAME} 
  COMMAND echo "${PROJECT_NAME} build starts."
  ${${PROJECT_NAME}_WHEN_${PROJECT_NAME}_BUILD_EXECUTED_COMMANDS} 

  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

  # 빌드 타겟에 의존성 추가 : 빌드 시 excute_before_build_${PROJECT_NAME}을 실행.
  add_dependencies(${PROJECT_NAME} excute_before_build_${PROJECT_NAME})  

  if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    # 현재 프로젝트가 루트이다.
    # 루트에서만 동작하는 코드를 작성한다.

    # install the generated configuration files
    # install(FILES
    # ${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_config.cmake
    # ${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_config_version.cmake
    # DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME}
    # )

    install(FILES 
      ${CMAKE_BINARY_DIR}/resources/txt/license.txt
      ${CMAKE_CURRENT_SOURCE_DIR}/release_notes.org

      TYPE DOC
    )    

  else()
    # 셀프가 루트 프로젝트가 아니다.
  
  endif()

  # 설치 규칙 설정

  install(FILES 
  "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_config.h"

  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  )

  install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}
  )

  # generate the export targets for the build tree
  # needs to be after the install(TARGETS) command
  export(EXPORT ${PROJECT_NAME}_targets
  FILE "${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_targets.cmake"
  )

  # install the configuration targets
  install(EXPORT ${PROJECT_NAME}_targets
  FILE ${PROJECT_NAME}_targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME}
  )
  
  if( ${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex")
    foreach( HD ${${PROJECT_NAME}_HEADERS} )
    list(APPEND  ${PROJECT_NAME}_HEADERS_PATHS "${PROJECT_BINARY_DIR}/include/${HD}")
    endforeach()

  elseif( ${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison")
    foreach( HD ${${PROJECT_NAME}_HEADERS} )
    list(APPEND  ${PROJECT_NAME}_HEADERS_PATHS "${PROJECT_BINARY_DIR}/include/${HD}")
    endforeach()

  elseif( ${PROJECT_NAME}_PROJECT_TYPE STREQUAL "flex_main")
    foreach( HD ${${PROJECT_NAME}_HEADERS} )
    list(APPEND  ${PROJECT_NAME}_HEADERS_PATHS "${PROJECT_BINARY_DIR}/include/${HD}")
    endforeach()    

  elseif( ${PROJECT_NAME}_PROJECT_TYPE STREQUAL "bison_main")
    foreach( HD ${${PROJECT_NAME}_HEADERS} )
    list(APPEND  ${PROJECT_NAME}_HEADERS_PATHS "${PROJECT_BINARY_DIR}/include/${HD}")
    endforeach()  

  else()
    foreach( HD ${${PROJECT_NAME}_HEADERS} )
    list(APPEND  ${PROJECT_NAME}_HEADERS_PATHS "${PROJECT_SOURCE_DIR}/${HD}")
    endforeach()
  endif()

  install(FILES ${${PROJECT_NAME}_HEADERS_PATHS}

  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${CMAKE_PROJECT_NAME}
  )

  get_all_inc_dirs(retIncDirs)
  list(APPEND ${PROJECT_NAME}_INC_DIRS ${retIncDirs})
  list(REMOVE_DUPLICATES ${PROJECT_NAME}_INC_DIRS)

  target_include_directories(${PROJECT_NAME} 
    PRIVATE ${${PROJECT_NAME}_INC_DIRS}
    )

  get_all_lib_dirs(retLibDirs)
  list(APPEND ${PROJECT_NAME}_LIB_DIRS ${retLibDirs})    
  list(REMOVE_DUPLICATES ${PROJECT_NAME}_LIB_DIRS)

  target_link_directories(${PROJECT_NAME} 
    PRIVATE ${${PROJECT_NAME}_LIB_DIRS}
    )

  # names_to_libs(${PROJECT_NAME}_SUBS aRetLibs)
  # get_all_libs_from(${CMAKE_PROJECT_NAME} aRetLibs)
  get_all_libs(aRetLibs)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${aRetLibs})

  # 현재 구성을 출판한다.
  # export(TARGETS ${PROJECT_NAME} FILE ${PROJECT_NAME}_config.cmake)
  # export(EXPORT ${PROJECT_NAME} FILE ${PROJECT_NAME}_config.cmake)

  include(CPack)

  message("${PROJECT_NAME} project config done.")
endmacro()

macro(prepare_lib)
  message("${PROJECT_NAME} prepare_lib()")
  # 라이브러리 cpp h를 생산한다.
  # do_lib()
  # 라이브러리 테스터 cpp 를 생산한다.
endmacro(prepare_lib)

macro(do_lib)
  add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
endmacro() #endmacro(do_lib)

macro(prepare_dll)
  message("${PROJECT_NAME} prepare_dll()")
  # do_lib()
endmacro() # prepare_dll

macro(do_dll)
  add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})
  target_compile_definitions(${PROJECT_NAME} PUBLIC IS_DLL)
  target_compile_definitions(${PROJECT_NAME} PUBLIC DLL_EXPORT)
endmacro() # do_dll

macro(prepare_main_cpp)
  message("${PROJECT_NAME} prepare_main_cpp()")
  # do_main_cpp()
endmacro() # macro(prepare_main_cpp)

macro(do_main_cpp)
  add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
endmacro() # macro(do_main_cpp)

macro(prepare_main_c)
  message("${PROJECT_NAME} prepare_main_c()")
  # do_main_c()
endmacro() # macro(prepare_main_c)

macro(do_main_c)
  add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
endmacro() # macro(do_main_c)

macro(prepare_flex tFilename tFlexIn)
  message("${PROJECT_NAME} prepare_flex(${tFilename} ${tFlexIn})")
  set(gFlexIn ${tFlexIn})

  if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(gFlexPath "${HOME}/apps/local/bin/win_flex.exe" )
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(gFlexPath "flex" )
  else()
    # 해당되는 system을 모르겠다.
    message("CMAKE_SYSTEM_NAME 이 ${CMAKE_SYSTEM_NAME} 이다.")
  endif()  

  set(gFlexFilePath "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.l" )
  
  # set(gFlexCFilename ${PROJECT_NAME}.c )
  set(gFlexCFilename ${tFilename}.c )

  set(gFlexCFilePath "${CMAKE_BINARY_DIR}/src/${gFlexCFilename}" )

  # set(gFlexHFilename ${PROJECT_NAME}.h )
  set(gFlexHFilename ${tFilename}.h )
  
  set(gFlexHFilePath "${CMAKE_BINARY_DIR}/include/${gFlexHFilename}" )    
  
  list(APPEND gFlexArgs --outfile=src/${gFlexCFilename} 
    --header=include/${gFlexHFilename} 
    # --tables-file=../bin/${gFlexTblFilename} 
    ${gFlexFilePath}
    )

  string(JOIN " " FLEX_ARGS ${gFlexArgs})

  add_custom_target(${PROJECT_NAME}_excute_flex
    ${gFlexPath} ${gFlexArgs}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )    
      
endmacro()

macro(do_flex)
  add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_excute_flex)
endmacro()

macro(prepare_bison tFilename tBisonIn)
  message("${PROJECT_NAME} prepare_bison(${tFilename} ${tBisonIn})")
  set(gBisonIn ${tBisonIn})

  if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(gBisonPath "${HOME}/apps/local/bin/win_bison.exe")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(gBisonPath "bison")
  else()
    # 해당되는 system을 모르겠다.
    message("CMAKE_SYSTEM_NAME 이 ${CMAKE_SYSTEM_NAME} 이다.")
  endif()

  set(gBisonFilePath "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.y")
  set(gBisonCFilename ${tFilename}.c)
  set(gBisonCFilePath "${CMAKE_BINARY_DIR}/src/${gBisonCFilename}")
  set(gBisonHFilename ${tFilename}.h)
  set(gBisonHFilePath "${CMAKE_BINARY_DIR}/include/${gBisonHFilename}")

    # win_bison -d --output=test_flex.c .\test_lexer.y
    # win_bison --output=test_flex.c --header=test_flex.h .\test_lexer.y
  # set(gBisonArgs "--output=${gBisonCFilename} ${gBisonFilePath}"
  list(APPEND gBisonArgs 
    --output=src/${gBisonCFilename} 
    --header=include/${gBisonHFilename} ${gBisonFilePath})

  string(JOIN " " BISON_ARGS ${gBisonArgs})

  add_custom_target(${PROJECT_NAME}_excute_bison
    ${gBisonPath} ${gBisonArgs}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )  
endmacro()

macro(do_bison)
  add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_excute_bison)
endmacro()

macro(do_flex_main)
  add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
endmacro()

macro(do_bison_main)
  add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
endmacro()

macro(prepare_libpp)
  message("${PROJECT_NAME} prepare_libpp()")
  # 라이브러리 cpp h를 생산한다.
  # do_libpp()
  # 라이브러리 테스터 cpp 를 생산한다.
  # tester.cpp에서 헤더 파일을 인클루드 한다.
  set(HEADER_SUFFIX "hpp")
endmacro()

macro(do_libpp)
  add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
endmacro() #endmacro(do_libpp)

macro(prepare_dllpp)
  message("${PROJECT_NAME} prepare_dllpp()")
  # 라이브러리 cpp h를 생산한다.
  # do_dllpp()
  # 라이브러리 테스터 cpp 를 생산한다.
  # tester.cpp에서 헤더 파일을 인클루드 한다.
  set(HEADER_SUFFIX "hpp")  
endmacro()

macro(do_dllpp)
  add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})
  target_compile_definitions(${PROJECT_NAME} PUBLIC IS_DLL)
  target_compile_definitions(${PROJECT_NAME} PUBLIC DLL_EXPORT)
endmacro() #endmacro(do_dllpp)

function(addSrcs list)
  set(aList)
  # list의 값들을 projectSrcs에 하나씩 붙인다.
  # message("ARGC = ${ARGC}")
  # message("ARGV = ${ARGV}")
  # message("ARGN = ${ARGN}")
  # message("myArg = ${myArg}")
  # message("ARGV0 = ${ARGV0}")
  # message("ARGV1 = ${ARGV1}")
  # message("ARGV50 = ${ARGV50}")  
  foreach(value ${ARGV})
    # message("value = ${value}")
    # list(APPEND ${PROJECT_NAME}_SRCS ${value})
    list(APPEND aList ${value})
    # message("aList = ${$aList}")
  endforeach() # foreach(value list)

  list(APPEND aList ${${PROJECT_NAME}_SRCS})
  # projectSrcs의 값과 list를 리턴한다.
  # unset(${PROJECT_NAME}_SRCS)
  set(${PROJECT_NAME}_SRCS ${aList} PARENT_SCOPE)
  # message("${PROJECT_NAME}_SRCS = ${${PROJECT_NAME}_SRCS}")

endfunction() # function(addSrcs list)

function(printSubs)
  message("")
  message("${PROJECT_NAME}_SUBS is : ")
  foreach(sub IN LISTS ${PROJECT_NAME}_SUBS)
  # foreach(sub IN LISTS ${PROJECT_NAME}_SUBS)
    message(STATUS "  ${PROJECT_NAME} sub : ${sub}")
  endforeach() # foreach(src ${PROJECT_NAME}_SUBS)
  message("")  
endfunction()

function(printLibs)
  message("")
  message(STATUS "${PROJECT_NAME}_LIBS is : ")
  get_target_property(aRetLibs ${PROJECT_NAME} LINK_LIBRARIES)
  foreach(aValue IN LISTS aRetLibs)
    message(STATUS "  ${PROJECT_NAME} lib : ${aValue}")
  endforeach() # foreach(src ${PROJECT_NAME}_SRCS)
  message("")
endfunction()

function(printSrcs)
  message("")
  message(STATUS "${PROJECT_NAME}_SRCS is : ")
  get_target_property(aRetSrcs ${PROJECT_NAME} SOURCES)
  foreach(src IN LISTS aRetSrcs)
    message(STATUS "  ${PROJECT_NAME} src : ${src}")
  endforeach() # foreach(src ${PROJECT_NAME}_SRCS)
  message("")
endfunction()

function(printHeaders)
  message("")
  message("${PROJECT_NAME}_HEADERS : ")
  foreach(header IN LISTS ${PROJECT_NAME}_HEADERS)
    message(STATUS "  ${PROJECT_NAME} header : ${header}")
  endforeach()
  message("")  
endfunction()

function(printIncDirs)
  message("")
  message(STATUS "${PROJECT_NAME}_INC_DIRS : ")
  get_target_property(aRetIncDirs ${PROJECT_NAME} INCLUDE_DIRECTORIES)
  foreach(value IN LISTS aRetIncDirs)
  # foreach(value ${CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE})
    message(STATUS "  ${PROJECT_NAME} inc dir : ${value}")
  endforeach()

  foreach(value IN LISTS CMAKE_INCLUDE_DIRECTORIES_BEFORE)
    message(STATUS "  CMAKE_INCLUDE_DIRECTORIES_BEFORE : ${value}")
  endforeach()

  message("")
endfunction()

function(printLibDirs)
  message("")
  message(STATUS "${PROJECT_NAME}_LIB_DIRS : ")
  get_target_property(aRetLibDirs ${PROJECT_NAME} LINK_DIRECTORIES)
  foreach(value IN LISTS aRetLibDirs)
    message(STATUS "  ${PROJECT_NAME} lib dir : ${value}")
  endforeach() # foreach(value ${${PROJECT_NAME}_LIB_DIRS})
  message("")
endfunction() # function(printLibDirs)

function(print_children)
  message("")
  message(STATUS "${PROJECT_NAME}_CHILDREN : ")

  foreach(aName IN LISTS ${PROJECT_NAME}_CHILDREN)
    message(STATUS "  ${PROJECT_NAME} Child : ${aName}")
  endforeach() 

  message("")
endfunction() #function(print_children)

function(print_children_libs)
  message("")
  message(STATUS "${PROJECT_NAME}_CHILDREN_LIBS is : ")
  foreach(aValue IN LISTS ${PROJECT_NAME}_CHILDREN_LIBS)
    message(STATUS "  ${PROJECT_NAME} child lib : ${aValue}")
  endforeach() # foreach(src ${PROJECT_NAME}_SRCS)
  message("")
endfunction() 

function(print_children_srcs)
  message("")
  message(STATUS "${PROJECT_NAME}_CHILDREN_SRCS is : ")
  foreach(src IN LISTS ${PROJECT_NAME}_CHILDREN_SRCS)
    message(STATUS "  ${PROJECT_NAME} child src : ${src}")
  endforeach() 
  message("")
endfunction() 

function(print_children_headers)
  message("")
  message("${PROJECT_NAME}_CHILDREN_HEADERS : ")
  foreach(header IN LISTS ${PROJECT_NAME}_CHILDREN_HEADERS)
    message(STATUS "  ${PROJECT_NAME} child header : ${header}")
  endforeach() 
  message("")  
endfunction() 

function(print_children_inc_dirs)
  message("")
  message(STATUS "${PROJECT_NAME}_CHILDREN_INC_DIRS : ")
  foreach(value IN LISTS ${PROJECT_NAME}_CHILDREN_INC_DIRS)
    message(STATUS "  ${PROJECT_NAME} child inc dir : ${value}")
  endforeach() 
  message("")
endfunction() 

function(print_children_lib_dirs)
  message("")
  message(STATUS "${PROJECT_NAME}_CHILDREN_LIB_DIRS : ")
  foreach(value IN LISTS ${PROJECT_NAME}_CHILDREN_LIB_DIRS)
    message(STATUS "  ${PROJECT_NAME} child lib dir : ${value}")
  endforeach() 
  message("")
endfunction() 

macro(addHeaders list)
  foreach(value ${ARGV})
    list(APPEND ${PROJECT_NAME}_HEADERS ${value})
  endforeach() # foreach(value list)
endmacro() # function(addHeaders list)

macro(appendThisProjectNameToChildProjectNamesOfParent)
  message(STATUS "CMAKE_PROJECT_NAME is ${CMAKE_PROJECT_NAME}")
  message(STATUS "PROJECT_NAME is ${PROJECT_NAME}")
  if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message("CMAKE_PROJECT_NAME IS This Project Name")
  else()
    set(aList ChildProjectNames)
    set(ChildProjectNames ${PROJECT_NAME} PARENT_SCOPE)
  endif() # if(CMAKE_PROJECT_NAME EQUAL PROJECT_NAME)
endmacro() #macro(appendThisProjectNameToChildProjectNamesOfParent)

macro(startTester)
  # message(STATUS "")
  message(STATUS "${PROJECT_NAME} config starts.")
  # message(STATUS "")

  # get_cur_inc_dirs(aRetIncDirs)
  # list(APPEND ${PROJECT_NAME}_INC_DIRS ${aRetIncDirs})

  # get_cur_lib_dirs(aRetLibDirs)
  # list(APPEND ${PROJECT_NAME}_LIB_DIRS ${aRetLibDirs})
  
  # ${CMAKE_INSTALL_BINDIR}, ${CMAKE_INSTALL_LIBDIR} 등을 사용한다.
  include(GNUInstallDirs)

  # cmake에 필요한 config파일들을 자동으로 작성하는 함수들을 호출한다.
  include(CMakePackageConfigHelpers)

  # tester는 boost를 사용한다.
  # boost의 폴더를 추가한다.
  if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    addIncDir("${HOME}/apps/local/include")
    addLibDir("${HOME}/apps/local/lib")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    addIncDir("${HOME}/apps/include")
    addLibDir("${HOME}/apps/lib")
  else()
    # 해당되는 system을 모르겠다.
    message("CMAKE_SYSTEM_NAME 이 ${CMAKE_SYSTEM_NAME} 이다.")
  endif()

endmacro() # startTester

macro(endTester)
  # message("${PROJECT_NAME} executable creates.")
  addSrc(${PROJECT_NAME}.cpp)
  add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

  set_target_properties(${PROJECT_NAME} PROPERTIES
  DEBUG_POSTFIX "_d"
  )

  get_target_property(target_type ${PROJECT_NAME} TYPE)
  
  # 라이브러리의 출력 디렉토리 설정
  set_target_properties(${PROJECT_NAME} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY 
        "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}"
      RUNTIME_OUTPUT_DIRECTORY 
        "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}"
      ARCHIVE_OUTPUT_DIRECTORY 
        "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}"
  )

  # 헤더파일을 추가하는 곳을 지정. 
  # #include "sub/directory/header.h"가 아니라,
  # #include "header.h" 라고 적을 수 있다.
  get_all_inc_dirs(aRetIncDirs)
  list(APPEND ${PROJECT_NAME}_INC_DIRS ${aRetIncDirs})
  list(REMOVE_DUPLICATES ${PROJECT_NAME}_INC_DIRS)

  target_include_directories(${PROJECT_NAME} 
    PRIVATE ${${PROJECT_NAME}_INC_DIRS}
    )
  
  get_all_lib_dirs(aRetLibDirs)    
  list(APPEND ${PROJECT_NAME}_LIB_DIRS ${aRetLibDirs})
  list(REMOVE_DUPLICATES ${PROJECT_NAME}_LIB_DIRS)

  target_link_directories(${PROJECT_NAME} 
    PRIVATE ${${PROJECT_NAME}_LIB_DIRS}
    )

  # get_all_libs_from(${CMAKE_PROJECT_NAME} aRetLibs)
  # list(APPEND aRetSubs ${PROJECT_NAME})
  # get_all_libs_from(${CMAKE_PROJECT_NAME} aRetLibs)
  get_all_libs(aRetLibs)
  # name_to_lib(${aRetParentName} aRetLib)
  # list(APPEND aRetLibDirs ${aRetLib})  
  target_link_libraries(${PROJECT_NAME} PRIVATE ${aRetLibs})
      
  set(aBuildType $<$<CONFIG:Debug>:DEBUG>)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ${aBuildType})
  # target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<CONFIG:Debug>:DEBUG>)

  # 빌드 시작할 때 외부 프로그램 실행
  add_custom_target(excute_before_build_${PROJECT_NAME} 
    COMMAND echo "${PROJECT_NAME} build starts."
    ${${PROJECT_NAME}_WHEN_${PROJECT_NAME}_BUILD_EXECUTED_COMMANDS} 
    
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )

  # 프로젝트 빌드 시작 전에 외부 프로그램을 실행한다.
  add_dependencies(${PROJECT_NAME} excute_before_build_${PROJECT_NAME} )

  # 셀프는 부모가 먼저 빌드된 후에 빌드한다.
  get_parent(aRetParentName)
  message("${PROJECT_NAME}`ParentName is : ${aRetParentName}")
  add_dependencies(excute_before_build_${PROJECT_NAME} ${aRetParentName})  
  # add_dependencies(${aRetParentName} ${PROJECT_NAME})  

  # 설치 규칙 설정
  install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${CMAKE_PROJECT_NAME}
  )

  # generate the export targets for the build tree
  # needs to be after the install(TARGETS) command
  export(EXPORT ${PROJECT_NAME}_targets
  FILE "${CMAKE_BINARY_DIR}/lib/cmake/${CMAKE_PROJECT_NAME}/${PROJECT_NAME}_targets.cmake"
  )

  # # install the configuration targets
  install(EXPORT ${PROJECT_NAME}_targets
  FILE ${PROJECT_NAME}_targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${CMAKE_PROJECT_NAME}
  )

  foreach( HD ${${PROJECT_NAME}_HEADERS} )
    list(APPEND  ${PROJECT_NAME}_HEADERS_PATHS "${PROJECT_SOURCE_DIR}/${HD}")
  endforeach()

  install(FILES ${${PROJECT_NAME}_HEADERS_PATHS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  )

  include(CPack)

  message("${PROJECT_NAME} tester config done.")
endmacro() # endTester

function(names_to_libs tNames retLibs)
  foreach (aName IN LISTS ${tNames})
    list(APPEND tempLibs $<IF:$<CONFIG:Debug>,${aName}_d,${aName}> )
  endforeach()
  set(${retLibs} ${tempLibs} PARENT_SCOPE)
  # set(retLibs ${tempLibs})
endfunction()

function(name_to_lib tName retLib)
  set(tempLib $<IF:$<CONFIG:Debug>,${tName}_d,${tName}> )
  set(${retLib} ${tempLib} PARENT_SCOPE)
endfunction()

function(get_cur_inc_dirs retIncDirs)
  # 수동으로 현재 프로젝트의 헤더 폴더들을 얻는다.
  list(APPEND aRetIncDirs "${CMAKE_BINARY_DIR}/include")
  list(APPEND aRetIncDirs "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}")
  list(APPEND aRetIncDirs "${CMAKE_CURRENT_SOURCE_DIR}")

  set(${retIncDirs} ${aRetIncDirs} PARENT_SCOPE)
endfunction()

function(get_cur_lib_dirs retLibDirs)
  list(APPEND aRetLibDirs "${CMAKE_BINARY_DIR}/lib")
  list(APPEND aRetLibDirs "${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}")
  list(APPEND aRetLibDirs "${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}/Debug")
  list(APPEND aRetLibDirs "${CMAKE_BINARY_DIR}/lib/${PROJECT_NAME}/Release")

  set(${retLibDirs} ${aRetLibDirs} PARENT_SCOPE)
endfunction()

function(get_name_from_dir tDir retName)
  # tDir의 타켓을 알아낸다.
  # 여기서 타겟은 PROJECT_NAME과 같다.

  if(tDir STREQUAL CMAKE_SOURCE_DIR)
    # 루트인 경우
    set(aRetNames ${CMAKE_PROJECT_NAME})
    get_directory_property(aRetNames 
      DIRECTORY ${CMAKE_SOURCE_DIR}
      BUILDSYSTEM_TARGETS)
    set(${retName} ${aRetNames} PARENT_SCOPE)

  elseif(tDir STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # tDir이 셀프이다.
    get_directory_property(aRetNames BUILDSYSTEM_TARGETS)
    set(${retName} ${aRetNames} PARENT_SCOPE)

  else()
    # tDir이 셀프의 서브이거나 상위 폴더이다.
    get_directory_property(aRetNames DIRECTORY ${tDir} BUILDSYSTEM_TARGETS)
    set(${retName} ${aRetNames} PARENT_SCOPE)
  endif()

  if(aRetNames)
    # tDir에 타겟의 이름들이 있다.
    list(FILTER aRetNames EXCLUDE REGEX excute_)
    list(FILTER aRetNames EXCLUDE REGEX tester)
  else()
    # tDir에 타겟의 이름들이 없다.
    # tDir이 자식, 상위 폴더, 존재하지 않는다 이다.

  endif()  

  set(${retName} ${aRetNames} PARENT_SCOPE)
endfunction()

function(get_tester_name_from_dir tDir retName)
  # tDir의 타켓을 알아낸다.
  # 여기서 타겟은 PROJECT_NAME과 같다.

  if(tDir STREQUAL CMAKE_SOURCE_DIR)
    # 루트인 경우
    set(aRetNames ${CMAKE_PROJECT_NAME})
    get_directory_property(aRetNames 
      DIRECTORY ${CMAKE_SOURCE_DIR}
      BUILDSYSTEM_TARGETS)
    set(${retName} ${aRetNames} PARENT_SCOPE)

  elseif(tDir STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # tDir이 셀프이다.
    get_directory_property(aRetNames BUILDSYSTEM_TARGETS)
    set(${retName} ${aRetNames} PARENT_SCOPE)

  else()
    # tDir이 셀프의 서브이거나 상위 폴더이다.
    get_directory_property(aRetNames DIRECTORY ${tDir} BUILDSYSTEM_TARGETS)
    set(${retName} ${aRetNames} PARENT_SCOPE)
  endif()

  if(aRetNames)
    # tDir에 타겟의 이름들이 있다.
    list(FILTER aRetNames EXCLUDE REGEX excute_)
  else()
    # tDir에 타겟의 이름들이 없다.
    # tDir이 자식, 상위 폴더, 존재하지 않는다 이다.

  endif()  

  set(${retName} ${aRetNames} PARENT_SCOPE)
endfunction()

# tDir의 모든 서브 폴더들을 얻는다.
# 현재 프로젝트의 do_project 전에 추가된 서브들은 그냥 서브들이다.
# 현재 프로젝트의 do_project 후에 추가된 서브들은 그냥 자식들이다.
# 모든 서브 폴더들에는 서브들과 자식들과 기타드등이 포함되어 있다.
# 프로젝트 이름과 헷갈리면 안된다.
function(get_all_subs_from_dir tDir retSubs)
  get_directory_property(aRetSubDirs DIRECTORY ${tDir} SUBDIRECTORIES)  
  list(LENGTH aRetSubDirs retLen)
  if(retLen)
    # 서브 폴더들이 있다.
    list(APPEND aRetSubs ${aRetSubDirs})
    foreach(sub IN LISTS aRetSubDirs)
      get_all_subs_from_dir(${sub} aRetSubSubs)
    list(APPEND aRetSubs ${aRetSubSubs})
    endforeach()
  else()
    # 서브 폴더들이 없다.
    # 종료한다.
  endif()

  list(REMOVE_DUPLICATES aRetSubs)
  set(${retSubs} ${aRetSubs} PARENT_SCOPE)
endfunction()

# tProjectName의 부모 폴더를 알아낸다.
function(get_parent_dir_from tProjectName retName)
  get_target_property(aSrcDir ${tProjectName} SOURCE_DIR )
  get_directory_property(aRetDir DIRECTORY ${aSrcDir} PARENT_DIRECTORY)
  set(${retName} ${aRetDir} PARENT_SCOPE)
endfunction() 

# 셀프의 부모 폴더를 알아낸다.
function(get_parent_dir retName)
  get_directory_property(aRetDir PARENT_DIRECTORY)
  set(${retName} ${aRetDir} PARENT_SCOPE)
endfunction() 

# 셀프가 자식인 경우 부모의 이름을 알아낸다.
function(get_parent retName)
  get_parent_from(${PROJECT_NAME} aRetName)
  set(${retName} ${aRetName} PARENT_SCOPE)
endfunction() 

# tProjectName의 부모의 이름을 알아낸다.
# 없으면 ""이다.
function(get_parent_from tProjectName retName)
  set(aParent $CACHE{${tProjectName}_PARENT})
  if(aParent)
    set(${retName} ${aParent} PARENT_SCOPE)
  else()
    # 부모 이름이 없다
    # 셀프가 부모의 자식인데도 이름이 없다는 것은 
    # CACHE 파일에 아직 쓰기 작업이 안되어 있다는 것이다.
    # 소스파일이 완료되어야 쓰기작업이 된다.
    # 소스파일이 아직 완료되기 전에 함수가 호출되었다.
    # 수동으로 이름을 지정해야 한다.
    # 셀프가 테스터이면 수동으로 지정할 수 있다.
    if(tProjectName MATCHES tester)
      string(REPLACE "_tester" "" aParent ${PROJECT_NAME})
      set(${retName} ${aParent} PARENT_SCOPE)
    else()
      get_parent_dir_from(${tProjectName} aRetDir)
      if(aRetDir)
        get_name_from_dir(${aRetDir} aRetName)
        set(${retName} ${aRetName} PARENT_SCOPE)
      else()
        set(${retName} "" PARENT_SCOPE)
      endif()
    endif()
  endif()
endfunction() 

# 셀프가 자식인 경우 부모와 조부모들의 이름을 알아낸다.
function(get_all_parent tProjectName retNames)
  get_parent_from(${tProjectName} aParent)

  while(aParent)
    list(APPEND aRetNames ${aParent})
    get_parent_from(${aParent} aParent)
  endwhile()

  set(${retNames} ${aRetNames} PARENT_SCOPE)    
endfunction() 

# 현재 디렉토리에서 타겟이 적용되었는지 확인한다.
#  add_executable이나 add_library를 호출해야 적용된 것이다.
function(is_target_available tTarget ret)
  # get_all_targets_from_dir(${CMAKE_SOURCE_DIR} aTargets)
  get_directory_property(aTargets BUILDSYSTEM_TARGETS)
  list(LENGTH aTargets aLenTargets)

  if (aLenTargets EQUAL 0)
    # 현재 디렉토리에 타겟이 없다.
    set(${ret} 0 PARENT_SCOPE)

  else()
    if(${tTarget} IN_LIST aTargets)
      set(${ret} 1 PARENT_SCOPE)
    else()
      set(${ret} 0 PARENT_SCOPE)
    endif()

  endif()
endfunction()

# is_target_available과 같다.
# 차이점은 모든 디렉토리에서 타겟을 찾는다.
function(is_target_available_from_all_dirs tTarget ret)
  get_all_targets_from_dir(${CMAKE_SOURCE_DIR} aTargets)

  list(LENGTH aTargets aLenTargets)

  if (aLenTargets EQUAL 0)
    # 현재 디렉토리에 타겟이 없다.
    set(${ret} 0 PARENT_SCOPE)

  else()
    if(tTarget IN_LIST aTargets)
      set(${ret} 1 PARENT_SCOPE)
    else()
      set(${ret} 0 PARENT_SCOPE)
    endif()

  endif()
endfunction()

function(get_all_targets_from_dir tDir retTargets)
  get_all_subs_from_dir(${tDir} aRetSubs)
  list(APPEND aRetSubs ${tDir})

  foreach(sub IN LISTS aRetSubs)
    get_name_from_dir(${sub} aSubName)
    list(APPEND aRetTargets ${aSubName})
  endforeach()

  set(${retTargets} ${aRetTargets} PARENT_SCOPE)
endfunction()

# 셀프를 기준으로 이전에 추가된 모든 프로젝트의 이름을 알아낸다.
function(get_all_names retNames)
  get_all_subs_from_dir(${CMAKE_SOURCE_DIR} aRetSubDirs)

  # 모든 tester 폴더를 제외한다.
  list(FILTER aRetSubDirs EXCLUDE REGEX tester)

  foreach(aSubDir IN LISTS aRetSubDirs)
    get_name_from_dir(${aSubDir} aSubName)
    if(aSubName)
      list(APPEND aSubNames ${aSubName})
      # 서브의 부모와 조부모들은 따로 추가해야한다.
      get_all_parent(${aSubName} aRetParentNames)
    endif()

    list(APPEND aSubNames ${aRetParentNames})    
  endforeach()

  # 부모와 조부모들은 따로 추가해야한다.
  get_all_parent(${PROJECT_NAME} aCurParentNames)

  list(APPEND aSubNames ${aCurParentNames})    

  list(REMOVE_DUPLICATES aSubNames)
  set(${retNames} ${aSubNames} PARENT_SCOPE)
endfunction()

# 셀프를 기준으로 이전에 추가된 모든 프로젝트의 헤더 폴더를 알아낸다.
function(get_all_inc_dirs retIncDirs)
  get_all_names(aRetNames)

  foreach(aName IN LISTS aRetNames)
    get_target_property(aRetSubsIncDirs ${aName} INCLUDE_DIRECTORIES)
    if(aRetSubsIncDirs STREQUAL "aRetSubsIncDirs-NOTFOUND")
      # aName의 헤더 폴더가 없다.
    else()
      list(APPEND aRetIncDirs ${aRetSubsIncDirs})
    endif()  
  endforeach()

  list(REMOVE_DUPLICATES aRetIncDirs)
  set(${retIncDirs} ${aRetIncDirs} PARENT_SCOPE)
endfunction()

# 셀프를 기준으로 이전에 추가된 모든 프로젝트의 링크 폴더를 알아낸다.
function(get_all_lib_dirs retLibDirs)
  get_all_names(aRetNames)

  foreach(aName IN LISTS aRetNames)
    get_target_property(aRetSubsLibDirs ${aName} LINK_DIRECTORIES)
    if(aRetSubsLibDirs STREQUAL "aRetSubsLibDirs-NOTFOUND")
      # aName의 헤더 폴더가 없다.
    else()
      list(APPEND aRetLibDirs ${aRetSubsLibDirs})
    endif()  
  endforeach()

  list(REMOVE_DUPLICATES aRetLibDirs)
  set(${retLibDirs} ${aRetLibDirs} PARENT_SCOPE)
endfunction()

# 셀프를 기준으로 이전에 추가된 모든 프로젝트의 링크를 알아낸다.
# 셀프의 링크는 제외한다. 셀프가 빌드하는데 셀프를 링크하면 모순이다.
function(get_all_libs retLibs)
  get_all_names(aRetNames)
  # 셀프의 이름은 제거한다.
  list(FILTER aRetNames EXCLUDE REGEX ${PROJECT_NAME})
  names_to_libs(aRetNames aRetLibs)
  set(${retLibs} ${aRetLibs} PARENT_SCOPE)
endfunction()

# tProjectName을 기준으로 이전에 추가된 모든 프로젝트의 링크를 알아낸다.
# tProjectName의 링크는 제외한다. 
# tProjectName가 빌드하는데 tProjectName를 링크하면 모순이다.
function(get_all_libs_from tProjectName retLibs)
  get_all_names(aRetNames)
  # 셀프의 이름은 제거한다.
  list(FILTER aRetNames EXCLUDE REGEX ${tProjectName})
  names_to_libs(aRetNames aRetLibs)
  set(${retLibs} ${aRetLibs} PARENT_SCOPE)
endfunction()
