

   set (IO_SLAVE_VERSION_MAJOR 1)
   set (IO_SLAVE_VERSION_MINOR 1)

   message(STATUS "Version: ${IO_SLAVE_VERSION_MAJOR}.${IO_SLAVE_VERSION_MINOR}")
   message(STATUS "Firmware target:" ${PROJECT_NAME} )

   message(STATUS ">>>DIRECTORY USED")
   message(STATUS "Source= ${PROJECT_SOURCE_DIR}")
   message(STATUS "Binary=${PROJECT_BINARY_DIR}")


   configure_file (
   "${PROJECT_SOURCE_DIR}/firmware/userconfig.h.in"
   "${PROJECT_BINARY_DIR}/firmware/userconfig.h"  ) 
   include_directories("${PROJECT_BINARY_DIR}/firmware") 

   add_subdirectory(i2c_slave)

   add_executable(${PROJECT_NAME} slave.c)

   pico_enable_stdio_uart(${PROJECT_NAME} 0)
   pico_enable_stdio_usb(${PROJECT_NAME} 1)

   pico_add_extra_outputs(${PROJECT_NAME})

   target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

   target_link_libraries(${PROJECT_NAME} i2c_slave pico_stdlib)


   
   # Set the UF2 file and ELF file
   set(UF2_FILE ${CMAKE_BINARY_DIR}/firmware/${PROJECT_NAME}.uf2)
   set(ELF_FILE ${CMAKE_BINARY_DIR}/firmware/${PROJECT_NAME}.elf)
   
   
   # Copy the .uf2 file
   add_custom_command(
       TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy ${UF2_FILE} ${CMAKE_BINARY_DIR}
      COMMENT "Copying .uf2 file to ${CMAKE_BINARY_DIR}"
   )
   
   # Copy the .elf file
   add_custom_command(
       TARGET ${PROJECT_NAME}  POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy ${ELF_FILE} ${CMAKE_BINARY_DIR}
      COMMENT "Copying .elf file to ${CMAKE_BINARY_DIR}"
   )
   
   