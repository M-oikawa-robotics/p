# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tsuji-lab/デスクトップ/peginpos

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tsuji-lab/デスクトップ/peginpos

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/tsuji-lab/デスクトップ/peginpos/CMakeFiles /home/tsuji-lab/デスクトップ/peginpos/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/tsuji-lab/デスクトップ/peginpos/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named mymod

# Build rule for target.
mymod: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 mymod
.PHONY : mymod

# fast build rule for target.
mymod/fast:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/build
.PHONY : mymod/fast

control_joint.o: control_joint.cpp.o

.PHONY : control_joint.o

# target to build an object file
control_joint.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_joint.cpp.o
.PHONY : control_joint.cpp.o

control_joint.i: control_joint.cpp.i

.PHONY : control_joint.i

# target to preprocess a source file
control_joint.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_joint.cpp.i
.PHONY : control_joint.cpp.i

control_joint.s: control_joint.cpp.s

.PHONY : control_joint.s

# target to generate assembly for a file
control_joint.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_joint.cpp.s
.PHONY : control_joint.cpp.s

control_pegin.o: control_pegin.cpp.o

.PHONY : control_pegin.o

# target to build an object file
control_pegin.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_pegin.cpp.o
.PHONY : control_pegin.cpp.o

control_pegin.i: control_pegin.cpp.i

.PHONY : control_pegin.i

# target to preprocess a source file
control_pegin.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_pegin.cpp.i
.PHONY : control_pegin.cpp.i

control_pegin.s: control_pegin.cpp.s

.PHONY : control_pegin.s

# target to generate assembly for a file
control_pegin.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_pegin.cpp.s
.PHONY : control_pegin.cpp.s

control_sliding.o: control_sliding.cpp.o

.PHONY : control_sliding.o

# target to build an object file
control_sliding.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_sliding.cpp.o
.PHONY : control_sliding.cpp.o

control_sliding.i: control_sliding.cpp.i

.PHONY : control_sliding.i

# target to preprocess a source file
control_sliding.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_sliding.cpp.i
.PHONY : control_sliding.cpp.i

control_sliding.s: control_sliding.cpp.s

.PHONY : control_sliding.s

# target to generate assembly for a file
control_sliding.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_sliding.cpp.s
.PHONY : control_sliding.cpp.s

control_turnover.o: control_turnover.cpp.o

.PHONY : control_turnover.o

# target to build an object file
control_turnover.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_turnover.cpp.o
.PHONY : control_turnover.cpp.o

control_turnover.i: control_turnover.cpp.i

.PHONY : control_turnover.i

# target to preprocess a source file
control_turnover.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_turnover.cpp.i
.PHONY : control_turnover.cpp.i

control_turnover.s: control_turnover.cpp.s

.PHONY : control_turnover.s

# target to generate assembly for a file
control_turnover.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/control_turnover.cpp.s
.PHONY : control_turnover.cpp.s

kyomath.o: kyomath.cpp.o

.PHONY : kyomath.o

# target to build an object file
kyomath.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/kyomath.cpp.o
.PHONY : kyomath.cpp.o

kyomath.i: kyomath.cpp.i

.PHONY : kyomath.i

# target to preprocess a source file
kyomath.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/kyomath.cpp.i
.PHONY : kyomath.cpp.i

kyomath.s: kyomath.cpp.s

.PHONY : kyomath.s

# target to generate assembly for a file
kyomath.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/kyomath.cpp.s
.PHONY : kyomath.cpp.s

mymod.o: mymod.cpp.o

.PHONY : mymod.o

# target to build an object file
mymod.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/mymod.cpp.o
.PHONY : mymod.cpp.o

mymod.i: mymod.cpp.i

.PHONY : mymod.i

# target to preprocess a source file
mymod.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/mymod.cpp.i
.PHONY : mymod.cpp.i

mymod.s: mymod.cpp.s

.PHONY : mymod.s

# target to generate assembly for a file
mymod.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/mymod.cpp.s
.PHONY : mymod.cpp.s

nn_interface.o: nn_interface.cpp.o

.PHONY : nn_interface.o

# target to build an object file
nn_interface.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/nn_interface.cpp.o
.PHONY : nn_interface.cpp.o

nn_interface.i: nn_interface.cpp.i

.PHONY : nn_interface.i

# target to preprocess a source file
nn_interface.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/nn_interface.cpp.i
.PHONY : nn_interface.cpp.i

nn_interface.s: nn_interface.cpp.s

.PHONY : nn_interface.s

# target to generate assembly for a file
nn_interface.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/nn_interface.cpp.s
.PHONY : nn_interface.cpp.s

pancake.o: pancake.cpp.o

.PHONY : pancake.o

# target to build an object file
pancake.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/pancake.cpp.o
.PHONY : pancake.cpp.o

pancake.i: pancake.cpp.i

.PHONY : pancake.i

# target to preprocess a source file
pancake.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/pancake.cpp.i
.PHONY : pancake.cpp.i

pancake.s: pancake.cpp.s

.PHONY : pancake.s

# target to generate assembly for a file
pancake.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/pancake.cpp.s
.PHONY : pancake.cpp.s

robot.o: robot.cpp.o

.PHONY : robot.o

# target to build an object file
robot.cpp.o:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/robot.cpp.o
.PHONY : robot.cpp.o

robot.i: robot.cpp.i

.PHONY : robot.i

# target to preprocess a source file
robot.cpp.i:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/robot.cpp.i
.PHONY : robot.cpp.i

robot.s: robot.cpp.s

.PHONY : robot.s

# target to generate assembly for a file
robot.cpp.s:
	$(MAKE) -f CMakeFiles/mymod.dir/build.make CMakeFiles/mymod.dir/robot.cpp.s
.PHONY : robot.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... mymod"
	@echo "... control_joint.o"
	@echo "... control_joint.i"
	@echo "... control_joint.s"
	@echo "... control_pegin.o"
	@echo "... control_pegin.i"
	@echo "... control_pegin.s"
	@echo "... control_sliding.o"
	@echo "... control_sliding.i"
	@echo "... control_sliding.s"
	@echo "... control_turnover.o"
	@echo "... control_turnover.i"
	@echo "... control_turnover.s"
	@echo "... kyomath.o"
	@echo "... kyomath.i"
	@echo "... kyomath.s"
	@echo "... mymod.o"
	@echo "... mymod.i"
	@echo "... mymod.s"
	@echo "... nn_interface.o"
	@echo "... nn_interface.i"
	@echo "... nn_interface.s"
	@echo "... pancake.o"
	@echo "... pancake.i"
	@echo "... pancake.s"
	@echo "... robot.o"
	@echo "... robot.i"
	@echo "... robot.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

