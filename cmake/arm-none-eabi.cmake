include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)

cmake_force_c_compiler(arm-none-eabi-gcc GNU)
cmake_force_cxx_compiler(arm-none-eabi-g++ GNU)
