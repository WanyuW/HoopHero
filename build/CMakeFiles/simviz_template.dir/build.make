# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.26.3/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.26.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/wynneweng/Desktop/sai2/apps/HoopHero

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/wynneweng/Desktop/sai2/apps/HoopHero/build

# Include any dependencies generated for this target.
include CMakeFiles/simviz_template.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/simviz_template.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/simviz_template.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simviz_template.dir/flags.make

CMakeFiles/simviz_template.dir/simviz.cpp.o: CMakeFiles/simviz_template.dir/flags.make
CMakeFiles/simviz_template.dir/simviz.cpp.o: /Users/wynneweng/Desktop/sai2/apps/HoopHero/simviz.cpp
CMakeFiles/simviz_template.dir/simviz.cpp.o: CMakeFiles/simviz_template.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/wynneweng/Desktop/sai2/apps/HoopHero/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/simviz_template.dir/simviz.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/simviz_template.dir/simviz.cpp.o -MF CMakeFiles/simviz_template.dir/simviz.cpp.o.d -o CMakeFiles/simviz_template.dir/simviz.cpp.o -c /Users/wynneweng/Desktop/sai2/apps/HoopHero/simviz.cpp

CMakeFiles/simviz_template.dir/simviz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/simviz_template.dir/simviz.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/wynneweng/Desktop/sai2/apps/HoopHero/simviz.cpp > CMakeFiles/simviz_template.dir/simviz.cpp.i

CMakeFiles/simviz_template.dir/simviz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/simviz_template.dir/simviz.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/wynneweng/Desktop/sai2/apps/HoopHero/simviz.cpp -o CMakeFiles/simviz_template.dir/simviz.cpp.s

# Object files for target simviz_template
simviz_template_OBJECTS = \
"CMakeFiles/simviz_template.dir/simviz.cpp.o"

# External object files for target simviz_template
simviz_template_EXTERNAL_OBJECTS =

/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: CMakeFiles/simviz_template.dir/simviz.cpp.o
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: CMakeFiles/simviz_template.dir/build.make
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-common/build/libsai2-common.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/chai3d/build/libchai3d.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libtinyxml2.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-simulation/build/libsai2-simulation.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-model/build/libsai2-model.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libtinyxml2.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-model/rbdl/build/librbdl.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-graphics/build/libsai2-graphics.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libtinyxml2.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/chai3d/build/libchai3d.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libjsoncpp.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libhiredis.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libglfw.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-primitives/build/libsai2-primitives.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-primitives/../external/ReflexxesTypeII/MacOS/x64/release/lib/libReflexxesTypeII.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-common/build/libsai2-common.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/chai3d/build/libchai3d.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libtinyxml2.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-simulation/build/libsai2-simulation.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-model/build/libsai2-model.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libtinyxml2.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-model/rbdl/build/librbdl.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-graphics/build/libsai2-graphics.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-urdfreader/build/libsai2-urdf.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libtinyxml2.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/chai3d/build/libchai3d.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libjsoncpp.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libhiredis.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /usr/local/lib/libglfw.dylib
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: /Users/wynneweng/Desktop/sai2/core/sai2-primitives/../external/ReflexxesTypeII/MacOS/x64/release/lib/libReflexxesTypeII.a
/Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template: CMakeFiles/simviz_template.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/wynneweng/Desktop/sai2/apps/HoopHero/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simviz_template.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simviz_template.dir/build: /Users/wynneweng/Desktop/sai2/apps/HoopHero/bin/template_project/simviz_template
.PHONY : CMakeFiles/simviz_template.dir/build

CMakeFiles/simviz_template.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simviz_template.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simviz_template.dir/clean

CMakeFiles/simviz_template.dir/depend:
	cd /Users/wynneweng/Desktop/sai2/apps/HoopHero/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/wynneweng/Desktop/sai2/apps/HoopHero /Users/wynneweng/Desktop/sai2/apps/HoopHero /Users/wynneweng/Desktop/sai2/apps/HoopHero/build /Users/wynneweng/Desktop/sai2/apps/HoopHero/build /Users/wynneweng/Desktop/sai2/apps/HoopHero/build/CMakeFiles/simviz_template.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simviz_template.dir/depend

