from string import Template
import os
import re

template = '''
cmake_minimum_required(VERSION 3.5)
project(${TARGET})
enable_language(ASM)

add_executable($${PROJECT_NAME} ${C_SOURCES} ${ASM_SOURCES})
target_compile_definitions($${PROJECT_NAME} PUBLIC ${C_DEFS})
target_include_directories($${PROJECT_NAME} PUBLIC ${C_INCLUDES})
target_compile_options($${PROJECT_NAME} PUBLIC ${MCU} -Wall -fdata-sections -ffunction-sections -std=c11)
target_link_libraries($${PROJECT_NAME} -L$${CMAKE_CURRENT_SOURCE_DIR} ${LDFLAGS} -Wl,--no-wchar-size-warning)
'''

#special cases:
#defines: remove -D
defines_list = [re.sub('-D([a-zA-Z_][a-zA-Z_0-9]*)', r'\1', x) for x in os.environ['C_DEFS'].split()];
os.environ['C_DEFS'] = ' '.join(defines_list);

#includes: remove -I
includes_list = [re.sub('-I([a-zA-Z_][a-zA-Z_0-9]*)', r'\1', x) for x in os.environ['C_INCLUDES'].split()];
os.environ['C_INCLUDES'] = ' '.join(includes_list);

print (Template(template).substitute(os.environ));
