cmake_minimum_required(VERSION 3.20)

# ISM330BX driver — tri-mode build (ESP-IDF, Zephyr, plain CMake).
# Bus-agnostic: the HAL exposes register read/write callbacks; the platform
# code wraps either SPI or I²C transactions to satisfy them.

if(IDF_VERSION_MAJOR)
	# ── ESP-IDF Component ────────────────────────────────────────────
	idf_component_register(
		SRCS "src/ism330bx.c"
		INCLUDE_DIRS "include"
	)
elseif(ZEPHYR_BASE)
	# ── Zephyr Module ────────────────────────────────────────────────
	zephyr_library()
	zephyr_library_sources(src/ism330bx.c)
	zephyr_include_directories(include)
else()
	# ── Standard CMake (STM32, desktop tests, etc.) ──────────────────
	if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
		project(ism330bx
			LANGUAGES C
			VERSION 0.1.0
			DESCRIPTION "Platform-agnostic driver for the ST ISM330BX 6-axis IMU"
		)
	endif()

	add_library(ism330bx STATIC
		src/ism330bx.c
	)
	target_include_directories(ism330bx PUBLIC include)
	target_compile_features(ism330bx PUBLIC c_std_11)
	target_compile_options(ism330bx PRIVATE -Wall -Wextra -Wpedantic)
endif()
