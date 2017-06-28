include ../Makefile
BUILD_DIR=.
.PHONY: gencmake

gencmake: ../CMakeLists.txt

export

../CMakeLists.txt: ../Makefile CMakeLists.txt.py
	python3 CMakeLists.txt.py > ../CMakeLists.txt
