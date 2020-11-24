set(sdk_dir "${AFR_VENDORS_DIR}/ublox/sdk")
set(afr_ports_dir "${CMAKE_CURRENT_LIST_DIR}/ports")
set(board_demos_dir "${CMAKE_CURRENT_LIST_DIR}/aws_demos")
set(board_tests_dir "${CMAKE_CURRENT_LIST_DIR}/aws_tests")
if(AFR_IS_TESTING)
    set(board_dir "${board_tests_dir}")
else()
    set(board_dir "${board_demos_dir}")
endif()

if(BOARD_HAS_CELLULAR)
    set(board_has_cellular_define -DBOARD_HAS_CELLULAR=1)
endif()

# -------------------------------------------------------------------------------------------------
# FreeRTOS Console metadata
# -------------------------------------------------------------------------------------------------
afr_set_board_metadata(ID "c030_r412m")
afr_set_board_metadata(DISPLAY_NAME "c030_r412m")
afr_set_board_metadata(DESCRIPTION "c030_r412m")
afr_set_board_metadata(VENDOR_NAME "ublox")
afr_set_board_metadata(FAMILY_NAME "C030")
afr_set_board_metadata(IS_ACTIVE "TRUE")
afr_set_board_metadata(DATA_RAM_MEMORY "192KB")
afr_set_board_metadata(PROGRAM_MEMORY "1MB")
afr_set_board_metadata(CODE_SIGNER "null")
afr_set_board_metadata(SUPPORTED_IDE "uvision")
afr_set_board_metadata(RECOMMENDED_IDE "uvision")
afr_set_board_metadata(IDE_uvision_NAME "Keil uvision")
afr_set_board_metadata(IDE_uvision_COMPILER "ARMCC")
afr_set_board_metadata(KEY_IMPORT_PROVISIONING "TRUE")

afr_set_board_metadata(IDE_uvision_PROJECT_LOCATION "${AFR_ROOT_DIR}/projects/ublox/c030_r412m/uvision/aws_demos")
afr_set_board_metadata(AWS_DEMOS_CONFIG_FILES_LOCATION "${CMAKE_CURRENT_LIST_DIR}/aws_demos/config_files")

# -------------------------------------------------------------------------------------------------
# Compiler settings
# -------------------------------------------------------------------------------------------------
afr_mcu_port(compiler)
target_compile_definitions(
    AFR::compiler::mcu_port
    INTERFACE $<$<COMPILE_LANGUAGE:C>:${compiler_defined_symbols}>
    -DSTM32F437xx -DMBEDTLS_CONFIG_FILE="<aws_mbedtls_config.h>"
    -DMBEDTLS_USER_CONFIG_FILE="\"mbedtls_user_config.h\""
    -DCONFIG_MEDTLS_USE_AFR_MEMORY -D__LITTLE_ENDIAN__
    -DFREERTOS_ENABLE_UNIT_TESTS
    -D__little_endian__=1
    -DNDEBUG
    -DUNITY_INCLUDE_CONFIG_H
    -DAMAZON_FREERTOS_ENABLE_UNIT_TESTS
    -DUSE_HAL_DRIVER
    ${extra_define}
)
if(AFR_IS_TESTING)
    target_compile_definitions(
        AFR::compiler::mcu_port
        INTERFACE -DMBEDTLS_USER_CONFIG_FILE=\"mbedtls_user_config.h\"
    )
endif()
target_compile_definitions(
    AFR::compiler::mcu_port
    INTERFACE $<$<COMPILE_LANGUAGE:ASM>:${assembler_defined_symbols}>
)

set (compiler_flags --c99 -c --cpu Cortex-M4.fp.sp  -g -O0 --apcs=interwork)
set (assembler_flags --cpu Cortex-M4.fp.sp -g --apcs=interwork --pd "__UVISION_VERSION SETA 532" )

target_compile_options(
    AFR::compiler::mcu_port
    INTERFACE
        $<$<COMPILE_LANGUAGE:C>:${compiler_flags}>
        $<$<COMPILE_LANGUAGE:C>:${board_has_cellular_define}>
        
)
target_compile_options(
    AFR::compiler::mcu_port
    INTERFACE
        $<$<COMPILE_LANGUAGE:ASM>:${assembler_flags}>
)

# Linker flags
set(linker_flags --cpu Cortex-M4.fp.sp --strict --scatter "${CMAKE_CURRENT_LIST_DIR}/c030_r412m.sct" --summary_stderr --info summarysizes,sizes,totals,unused,veneers --map --xref --callgraph --symbols )

target_link_options(
    AFR::compiler::mcu_port
    INTERFACE ${linker_flags}
)

target_link_libraries(
    AFR::compiler::mcu_port
    INTERFACE ${link_dependent_libs}
)

# -------------------------------------------------------------------------------------------------
# FreeRTOS portable layers
# -------------------------------------------------------------------------------------------------
# Normally the portable layer for kernel should be vendor's driver code.
afr_mcu_port(kernel)
afr_glob_src(driver_src DIRECTORY "${sdk_dir}" RECURSE)
target_sources(
    AFR::kernel::mcu_port
    INTERFACE
        ${driver_src}		
        "${AFR_KERNEL_DIR}/portable/MemMang/heap_4.c"
        "${AFR_KERNEL_DIR}/portable/RVDS/ARM_CM4F/port.c"
        "${AFR_KERNEL_DIR}/portable/RVDS/ARM_CM4F/portmacro.h"
)

set(
    kernel_inc_dirs
    "${AFR_KERNEL_DIR}/portable/RVDS/ARM_CM4F"
    "${sdk_dir}/CMSIS/Include"
    "${sdk_dir}/CMSIS/Device/ST/STM32F4xx/Include"
    "${sdk_dir}/STM32F4xx_HAL_Driver/Inc"
    "${board_dir}/application_code"
    "${board_dir}/application_code/st_code"
    "$<IF:${AFR_IS_TESTING},${AFR_TESTS_DIR},${AFR_DEMOS_DIR}>/include"
)

target_include_directories(
    AFR::kernel::mcu_port
    INTERFACE
        $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:${kernel_inc_dirs}>
        "${board_dir}/config_files"
)

if(BOARD_HAS_CELLULAR)
# Cellular
afr_mcu_port(cellular)
target_sources(
    AFR::cellular::mcu_port
    INTERFACE
        "${afr_ports_dir}/comm_if/comm_if_uart.c>"
        "${afr_ports_dir}/comm_if/device_control.c"
        "${afr_ports_dir}/comm_if/iot_fifo.c"
)

target_include_directories(
    AFR::cellular::mcu_port
    INTERFACE
        "${AFR_MODULES_ABSTRACTIONS_DIR}/common_io/include"
)
include("${AFR_VENDORS_DIR}/ublox/modules/sara_r4/CMakeLists.txt")
endif()

# PKCS11
afr_mcu_port(pkcs11_implementation)
target_sources(
    AFR::pkcs11_implementation::mcu_port
    INTERFACE "${afr_ports_dir}/pkcs11/core_pkcs11_pal.c"
)
# Link to AFR::pkcs11_mbedtls if you want to use default implementation based on mbedtls.
target_link_libraries(
    AFR::pkcs11_implementation::mcu_port
    INTERFACE
        AFR::pkcs11_mbedtls
)

# Secure sockets
afr_mcu_port(secure_sockets)
target_link_libraries(
    AFR::secure_sockets::mcu_port
    INTERFACE
        AFR::tls
        AFR::cellular
        AFR::secure_sockets_cellular
)
target_sources(
    AFR::secure_sockets::mcu_port
    INTERFACE "${afr_ports_dir}/secure_sockets/iot_secure_sockets.c"
)

# Common I/O
afr_mcu_port(common_io)
target_sources(
    AFR::common_io::mcu_port
    INTERFACE 
        "${afr_ports_dir}/common_io/iot_uart.c"
        $<${AFR_IS_TESTING}:${afr_ports_dir}/common_io/iot_test_common_io_internal.c>
)
target_include_directories(
    AFR::common_io::mcu_port
    INTERFACE
        "${AFR_MODULES_ABSTRACTIONS_DIR}/common_io/test"
)
# -------------------------------------------------------------------------------------------------
# FreeRTOS demos and tests
# -------------------------------------------------------------------------------------------------
set(CMAKE_EXECUTABLE_SUFFIX ".axf")

if(AFR_IS_TESTING)
    set(exe_target aws_tests)
else()
    set(exe_target aws_demos)
endif()

afr_glob_src(board_code_src DIRECTORY "${board_dir}/application_code/st_code")
afr_glob_src(config_files DIRECTORY "${board_dir}/config_files")

# Do not add demos or tests if they're turned off.
if(AFR_ENABLE_DEMOS OR AFR_ENABLE_TESTS)
    add_executable(
        ${exe_target}
        ${board_code_src}
        ${config_files}
        "${board_dir}/application_code/main.c"
        "$<${AFR_IS_TESTING}:${board_dir}/application_code/cellular_setup.c>"
    )
    target_link_libraries(
        ${exe_target}
        PRIVATE
            AFR::utils
            AFR::common_io
    )

    if(NOT AFR_METADATA_MODE)
        # Convert afx file to bin file
        find_program(ARM_KEIL_FROMELF fromelf.exe PATHS "${AFR_COMPILER_DIR}")
    endif()

    add_custom_command(
        TARGET ${exe_target} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:${exe_target}>" "${CMAKE_BINARY_DIR}"
        COMMAND "${ARM_KEIL_FROMELF}" --bin "${exe_target}.axf" --output "${exe_target}.bin"
        COMMAND "${ARM_KEIL_FROMELF}" --text -c "${exe_target}.axf" --output "${exe_target}.txt"
    )
endif()
