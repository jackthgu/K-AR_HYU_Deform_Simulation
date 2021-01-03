# Usage:
# for the first time: make install_dependencies; make init; make
# from next time, just make or make debug 
#
# This file internally uses cmake.
# Makefile is reusable. Simply change the EXE variable.
EXE=OgreFltk
BIN_FOLDER=../build_$(EXE)
BIN_TO_SRC=../../src
SRC_TO_WORK=../work
SRC_TO_EXE=../work/$(EXE)
NUM_CORES=$(shell cat /proc/cpuinfo | grep processor | wc -l)

# all depends on init_finished.txt
all: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory -j $(NUM_CORES)
	cp $(BIN_FOLDER)/build_linux/OgreFltk $(SRC_TO_EXE)

mac: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux;make --no-print-directory 
	cp $(BIN_FOLDER)/build_linux/OgreFltk $(SRC_TO_EXE)

# debug depends on init_finished.txt
debug: init_finished.txt
	if ! test -d $(BIN_FOLDER) ; then rm init_finished.txt; exit 1;fi
	cd $(BIN_FOLDER)/build_linux_debug;make --no-print-directory VERBOSE=1 -j $(NUM_CORES)
	cp $(BIN_FOLDER)/build_linux_debug/OgreFltk $(SRC_TO_EXE)_debug


run: 
	cd ../work;./$(EXE)
runf: 
	cd ../work;./$(EXE) --sep
drun: 
	make debug
	cd ../work;kdbg ./$(EXE)_debug
cgdb: 
	make debug
	cd ../work;cgdb ./$(EXE)_debug
kdbg: 
	make debug
	cd ../work;kdbg ./$(EXE)_debug
memcheck:
	cd ../work;valgrind --tool=memcheck ./$(EXE)_debug

# init_finished.txt depends on CMakeLists.txt
init_finished.txt: CMakeLists.txt
	mkdir -p $(BIN_FOLDER)
	mkdir -p $(BIN_FOLDER)/build_linux;cd $(BIN_FOLDER)/build_linux;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Release $(BIN_TO_SRC)
	mkdir -p $(BIN_FOLDER)/build_linux_debug;cd $(BIN_FOLDER)/build_linux_debug;export CC=gcc;export CXX=g++;cmake -G "Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug $(BIN_TO_SRC)
	touch init_finished.txt

clean:
	rm -rf $(BIN_FOLDER)
	rm init_finished.txt
memcheck:
	cd ../work;valgrind --leak-check=yes ./$(EXE)_debug
showcapture:
	avidemux ../dump/dump/00000.jpg
	
# this would probably install more libraries than necessary, but I guess it doens't matter for most.
install_dependencies:
	sudo apt-get install libsuitesparse-dev meshlab libtinyxml-dev libopencv-dev libglfw3-dev libglew-dev
	mkdir -p ../work
	mkdir -p ../dump
	cp ../../taesooLib/work/readline.so ../work/
	cd ..;ln -sf ../taesooLib/Resource Resource
	cd ..;ln -sf ../taesooLib/media media
	cd ..;ln -sf ../taesooLib/media12 media12
	cd ..;cp ../taesooLib/work/*.cfg work/
	cd ..;cp ../taesooLib/work/*.py work/

install_dependencies2:
	sudo apt install libvtk7-dev libopencv-dev
