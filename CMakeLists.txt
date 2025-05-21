# Minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Define project name and language (C in this case)
project(MyProject C)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Set the path to arm-none-eabi-gcc and other tools
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")  # Optional: if you're using assembly
set(CMAKE_LINKER "arm-none-eabi-ld")         # Optional: if you need a specific linker
# set(CMAKE_C_FLAGS -I${CMAKE_SOURCE_DIR}/lib/include -I${CMAKE_SOURCE_DIR}/CMSIS/CMSIS/Core/Include -g -Xassembler -Wa -mcpu=cortex-m4 -mthumb -Wall -lm -mfpu=fpv4-sp-d16 -mfloat-abi=hard -lc -std=c99 -ffreestanding -T ${CMAKE_SOURCE_DIR}/samd51j20a_flash.ld -D__SAMD51J20A__)
set(CMAKE_C_FLAGS -x c -mthumb -D__SAMD51J20A__ -DDEBUG -I${CMAKE_SOURCE_DIR}/lib/include -I${CMAKE_SOURCE_DIR}/CMSIS/CMSIS/Core/Include -O3 -fno-strict-aliasing -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m4 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)")
set(CMAKE_L_FLAGS -mthumb -Wl,-Map="build.map" -Wl,--start-group -lm -lc -Wl,--end-group -Wl,--gc-sections -mcpu=cortex-m4 -T ${CMAKE_SOURCE_DIR}/samd51j20a_flash.ld) 

# Set the output directory for the build
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Set compiler flags for arm-none-eabi-gcc

# Define the source files (all .c files in the src/ directory)
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*/*.c ${CMAKE_SOURCE_DIR}/src/*.c)
# Define the object files output directory
set(OBJECT_DIR ${CMAKE_BINARY_DIR}/obj)

# Create object files for each source file
foreach(src_file IN LISTS SOURCES)
    get_filename_component(object_file ${src_file} NAME_WE)
    set(object_file "${OBJECT_DIR}/${object_file}.o")
    list(APPEND OBJECTS ${object_file})
    add_custom_command(
        OUTPUT ${object_file}
        COMMAND ${CMAKE_C_COMPILER} ${CMAKE_C_FLAGS} -c ${src_file} -o ${object_file}
        DEPENDS ${src_file}
        COMMENT "Compiling ${src_file}"
    )
endforeach()

# Link the object files to create the ELF file
add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/build.elf
    COMMAND ${CMAKE_C_COMPILER} ${OBJECTS} -o ${CMAKE_BINARY_DIR}/build.elf ${CMAKE_L_FLAGS}
    DEPENDS ${OBJECTS}
    COMMENT "Linking ${CMAKE_BINARY_DIR}/build.elf"
)

# Add a custom target to build the ELF file
add_custom_target(build ALL
    DEPENDS ${CMAKE_BINARY_DIR}/build.elf
)

add_custom_target(flash
    COMMAND openocd -f openocd.cfg -c \"program ${CMAKE_BINARY_DIR}/build.elf verify reset exit\"
    DEPENDS ${CMAKE_BINARY_DIR}/build.elf
    COMMENT "Flashing to the SAMD51 chip..." 
)


