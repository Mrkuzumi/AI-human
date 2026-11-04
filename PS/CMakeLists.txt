cmake_minimum_required(VERSION 3.16)
# set sln name
project(PLin+TriNet+HDMI)
message(STATUS "Project name: ${PROJECT_NAME}")
message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")
message(STATUS "Project binary dir: ${PROJECT_BINARY_DIR}")

# --- 芯片后端选择 ---
# 添加一个缓存变量 TARGET_CHIP，默认值为 BY
# 用户可以在配置时通过 -DTARGET_CHIP=ZG 来修改
set(TARGET_CHIP "ZG" CACHE STRING "Select the target chip backend: ZG (ZG330)")
# 允许的值列表
set_property(CACHE TARGET_CHIP PROPERTY STRINGS "BY" "ZG")
# 将选项转换为大写，以便进行不区分大小写的比较
string(TOUPPER "${TARGET_CHIP}" TARGET_CHIP_UPPER)
# 根据选择设置编译定义和链接库
if(TARGET_CHIP_UPPER STREQUAL "BY")
    # 添加编译定义，以便C++代码可以识别
    add_compile_definitions(USE_BUYI_BACKEND)
    set(BACKEND_LIB_NAME "buyibackend") # 假设后端库名为 buyibackend
    message(STATUS "Configuring for Buyi (BY) Backend.")
elseif(TARGET_CHIP_UPPER STREQUAL "ZG")
    # 添加编译定义
    add_compile_definitions(USE_ZG330_BACKEND)
    set(BACKEND_LIB_NAME "zg330backend") # 假设后端库名为 zg330backend
    message(STATUS "Configuring for ZG330 (ZG) Backend.")
else()
    message(FATAL_ERROR "Invalid TARGET_CHIP value: ${TARGET_CHIP}. Please choose 'BY' or 'ZG'.")
endif()

# Define multiple targets
set(TARGET_NAMES 
	sdicamera+yolov8-pose+yolov8-face+yolov8-finger+hdmi
	# Add more target names here
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
#/Zi 是 MSVC 编译器的一个选项，用于生成调试信息
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/Zi>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/Zi>")
#/permissive- 用于启用更严格的语法检查和符合性检查
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/permissive->")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/permissive->")
#/wd4244 是 MSVC 编译器的一个选项，用于禁止特定的警告
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/wd4244>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/wd4244>")
#/INCREMENTAL:NO 是 MSVC 链接器的一个选项，用于禁用增量链接
add_link_options("$<$<C_COMPILER_ID:MSVC>:/INCREMENTAL:NO>")
add_link_options("$<$<CXX_COMPILER_ID:MSVC>:/INCREMENTAL:NO>")
add_link_options("$<$<C_COMPILER_ID:MSVC>:/DEBUG>")
add_link_options("$<$<CXX_COMPILER_ID:MSVC>:/DEBUG>")
#/OPT:REF 是 MSVC 链接器的一个选项，用于进行引用消除优化
add_link_options("$<$<AND:$<C_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/OPT:REF>")
add_link_options("$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/OPT:REF>")
#/OPT:ICF 是 MSVC 链接器的一个选项，用于进行函数节省优化（Identical Code Folding）
add_link_options("$<$<AND:$<C_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/OPT:ICF>")
add_link_options("$<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/OPT:ICF>")

set(DEPS_DIR ../../../deps CACHE PATH "FPAI SDK dependency root")

set(THIRDPARTY_INCLUDES
	${DEPS_DIR}/thirdparty/include
	${DEPS_DIR}/thirdparty/include/Eigen-3.4.0
	${DEPS_DIR}/thirdparty/include/Eigen-3.4.0/Eigen
	${DEPS_DIR}/thirdparty/include/fmt
	${DEPS_DIR}/thirdparty/include/spdlog
	${DEPS_DIR}/thirdparty/include/opencv2
	${DEPS_DIR}/thirdparty/include/onnxruntime
	${DEPS_DIR}/thirdparty/include/OpenEXR
	${DEPS_DIR}/thirdparty/include/ffmpeg
	)

if (WIN32)
    # Windows 操作系统
    set_property(GLOBAL PROPERTY USE_FOLDERS OFF)
	#add_definitions(-DYAML_CPP_STATIC_DEFINE)
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${TARGET_NAME}) # 设置目标项目作为启动项目
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT /Zi")
	set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
	set(ORT_LIB_DIR x64)

    link_directories(
		${DEPS_DIR}/thirdparty/lib
	)
	find_package(Icraft-BuyiBackend REQUIRED)
	find_package(Icraft-HostBackend REQUIRED)

	link_libraries(
	
		opencv_world455 opencv_img_hash455 ade IlmImf ippicvmt ippiw
		ittnotify libjpeg-turbo libopenjp2 libpng libprotobuf libtiff libwebp quirc
		fmt jpeg libpng16 tiff turbojpeg zlib webp webpdecoder webpdemux lzma 
		yaml-cpp
    )
	
	configure_file(${DEPS_DIR}/thirdparty/dll/yaml-cpp.dll "C:/Icraft/CLI/bin/" COPYONLY)

elseif(UNIX)
    # Unix/Linux 操作系统
    set(CMAKE_SKIP_RPATH TRUE) # ARM SDK libraries are resolved by the board's installed iCraft runtime.
    SET(CMAKE_SYSTEM_NAME Linux)
    SET(CMAKE_SYSTEM_PROCESSOR aarch64)
    SET(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
    SET(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
	# SET(CMAKE_BUILD_TYPE Debug)
	SET(CMAKE_BUILD_TYPE Release)

	link_directories(${DEPS_DIR}/thirdparty/a)
	link_libraries(
		avformat avdevice avfilter avcodec avutil swscale swresample
		opencv_imgcodecs opencv_imgproc opencv_core tegra_hal libjpeg-turbo ittnotify libpng libwebp libtiff libopenjp2
		IlmImf-2_3_s Iex-2_3_s Half-2_3_s IexMath-2_3_s IlmImfUtil-2_3_s IlmThread-2_3_s Imath-2_3_s #openexr deps for opencv4.10.0
		fmt pthread dl zlib
		yaml-cpp
		ssl crypto atomic
	)
	find_package(Icraft-BuyiBackend REQUIRED)
	find_package(Icraft-HostBackend REQUIRED)
	find_package(icraft-zg330backend REQUIRED)
endif()

# Loop through each target and create executables
foreach(TARGET_NAME ${TARGET_NAMES})
	# Check if source file exists
	if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${TARGET_NAME}.cpp)
		# set source files for current target
		set(APP_SOURCES_${TARGET_NAME}
			src/${TARGET_NAME}.cpp
			# Add common source files here if needed
		)
		
		add_executable(${TARGET_NAME} ${APP_SOURCES_${TARGET_NAME}})
		
		target_include_directories(${TARGET_NAME}
									PUBLIC
									${DEPS_DIR}/modelzoo_utils/include
									${CMAKE_CURRENT_SOURCE_DIR}/include
									${CMAKE_CURRENT_SOURCE_DIR}/third_party/bytetrack
									${THIRDPARTY_INCLUDES}
									)
		
		target_link_libraries(${TARGET_NAME}
							  PRIVATE
							  Icraft::HostBackend Icraft::BuyiBackend Icraft::ZG330Backend
							  stdc++fs
							  )
	endif()
endforeach()



