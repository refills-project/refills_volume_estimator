CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT (Volume_Counter)

set (CMAKE_CXX_STANDARD 11)
set(DEPENDENCIES ${OPENGL_LIBRARIES} )

#################################################################
# OpenCV
find_package(OpenCV REQUIRED)  # Assuming OpenCV_DIR is an environment variable
if (NOT ${OPENCV_FOUND})
	MESSAGE(STATUS "Could not locate OpenCV. Please define an environment variable named 'OpenCV_DIR'.")
	return()
endif (NOT ${OPENCV_FOUND})

#################################################################
# OpenGL
find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)

#Find glfw header
find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
	PATHS /usr/X11R6/include
	/usr/include/X11
	/opt/graphics/OpenGL/include
	/opt/graphics/OpenGL/contrib/libglfw
	/usr/local/include
	/usr/include/GL
	/usr/include
	)
#Find glfw library
find_library(GLFW_LIBRARIES NAMES glfw glfw3 GL glut GLU
	PATHS /usr/lib64
	/usr/lib
	/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
	/usr/local/lib64
	/usr/local/lib
	/usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
	/usr/X11R6/lib
	)

list(APPEND DEPENDENCIES m ${GLFW_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBS} realsense2 ${LIBUSB1_LIBRARIES} pthread)

#################################################################

FILE(GLOB src 
	*.cpp  	
	)
SOURCE_GROUP("Src" FILES ${src})

FILE(GLOB hdrs 
	*.h
	)
SOURCE_GROUP("Include" FILES ${hdrs})

ADD_EXECUTABLE( ${PROJECT_NAME} ${src} ${hdrs})

TARGET_INCLUDE_DIRECTORIES (${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${GLFW_INCLUDE_DIR} ${GLEW_INCLUDE} .)
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} PUBLIC ${DEPENDENCIES})


