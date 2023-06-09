# Add sai2-primitives
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${HOOPHERO_BINARY_DIR}/project)
ADD_EXECUTABLE (simviz simviz.cpp ${HOOPHERO_COMMON_SOURCE})
ADD_EXECUTABLE (controller controller.cpp ${HOOPHERO_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (simviz ${HOOPHERO_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})
TARGET_LINK_LIBRARIES (controller ${HOOPHERO_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# copy interface to output as well as any additional images
FILE(COPY basketball_court.jpg DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY wind.png DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY angle.png DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY 1.png DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY 2.png DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY 11.png DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY button-3.wav DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY button-2.wav DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY button-09a.wav DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY button-7.wav DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY bgm.mp3 DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY bgm2.mp3 DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY power.mp3 DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY swoosh.wav DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY swish.mp3 DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY mmp_panda.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY kuka_iiwa.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY toro.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY ball.urdf DESTINATION ${APP_RESOURCE_DIR})

# copy in sai2-interfaces 
FILE(COPY ${SAI2-INTERFACES_UI_DIR} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy launch script
FILE(COPY launch.sh DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# copy interface.py
FILE(COPY interface.py DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY new-interface.py DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY keys.py DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY customized_theme.json DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
FILE(COPY joystick_controller.py DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})


