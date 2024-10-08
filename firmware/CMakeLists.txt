

   set (IO_SLAVE_VERSION_MAJOR 1)
   set (IO_SLAVE_VERSION_MINOR 0)


   message(STATUS ">>>DIRECTORY USED")
   message(STATUS "Source= ${PROJECT_SOURCE_DIR}")
   message(STATUS "Binary=${PROJECT_BINARY_DIR}")


   configure_file (
   "${PROJECT_SOURCE_DIR}/firmware/userconfig.h.in"
   "${PROJECT_BINARY_DIR}/firmware/userconfig.h"  ) 
   include_directories("${PROJECT_BINARY_DIR}/firmware") 

   add_subdirectory(i2c_slave)

   add_executable(slave slave.c)

   pico_enable_stdio_uart(slave 0)
   pico_enable_stdio_usb(slave 1)

   pico_add_extra_outputs(slave)

   target_compile_options(slave PRIVATE -Wall)

   target_link_libraries(slave i2c_slave pico_stdlib)
   
