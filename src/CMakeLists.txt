cmake_minimum_required (VERSION 3.5)
project (hw3)

##########################################################################
# EXECUTABLE NAME
set(my_executable render)



if (${APPLE})

  ##########################################################################
  # MAC OSX / METAL SPECIFIC INSTRUCTIONS

  # todo:  remove this from code & then this goes away
  add_definitions(-DDEBUG=1 -DTARGET_MACOS)

  # location of bundle base directory
  set(bundle_output_dir ${PROJECT_BINARY_DIR}/${my_executable}.app)

  get_filename_component(clang_dir ${CMAKE_C_COMPILER} DIRECTORY)
  
  # build storyboard variables
  set(my_storyboard_name Metal.storyboard)
  set(my_storyboard ${PROJECT_SOURCE_DIR}/${my_storyboard_name})
  set(storyboard_output_dir ${bundle_output_dir}/Contents/Resources/Base.lproj)
  set(storyboard_info ${storyboard_output_dir}/${my_storyboard_name}c/MetalInfo.plist)
  set_source_files_properties(${storyboard_info} PROPERTIES GENERATED true)
  
  # build storyboard
  find_program(ibtool ibtool)
  if (NOT ibtool)
    message(FATAL_ERROR "Missing ibtool")
  endif ()
  add_custom_target(storyboard_target DEPENDS ${storyboard_info})
  add_custom_command(OUTPUT ${storyboard_info} DEPENDS ${my_storyboard}
    COMMAND ${ibtool} ARGS --compilation-directory ${storyboard_output_dir} ${my_storyboard} VERBATIM)
  
  # variables for metal & air files
  set(my_shader ${PROJECT_SOURCE_DIR}/MetalShaders.metal)
  set(air_file ${PROJECT_BINARY_DIR}/MetalShaders.air)
  set(metal_file ${bundle_output_dir}/Contents/Resources/default.metallib)
  set_source_files_properties(${metal_file} PROPERTIES GENERATED true)
  
  # building shader/airfile
  find_program(metal metal HINTS ${clang_dir})
  if (NOT metal)
    message(FATAL_ERROR "Missing metal")
  endif ()
  add_custom_target(air_target DEPENDS ${air_file})
  # The ibtool will create the Resources directory; ${metal} will not, so we generate
  # storyboard_target first.
  add_custom_command(OUTPUT ${air_file} DEPENDS ${my_shader} ${storyboard_info}
    COMMAND ${metal} ARGS -arch air64 -emit-llvm -c -o ${air_file} ${my_shader} VERBATIM)
  
  # building metal library
  find_program(metallib metallib HINTS ${clang_dir})
  if (NOT metallib)
    message(FATAL_ERROR "Missing metallib")
  endif ()
  add_custom_target(metal_target DEPENDS ${metal_file})
  add_custom_command(OUTPUT ${metal_file} DEPENDS ${air_file}
    COMMAND ${metallib} ARGS -o ${metal_file} ${air_file} VERBATIM)

else()

  ##########################################################################
  # LINUX / WINDOWS / GLM SPECIFIC INSTRUCTIONS

  set(OpenGL_GL_PREFERENCE GLVND)

  # http://glm.g-truc.net/0.9.5/updates.html
  add_definitions(-DGLM_FORCE_RADIANS)

  # We've placed a few FindXXX.cmake files in the the source directory
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")
  
  # Following the instructions for Windows here:
  # http://www.cs.rpi.edu/~cutler/classes/advancedgraphics/S19/opengl_install_notes.php
  # the graphics librarys files are placed in this directory
  set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:\\GraphicsLibraries")

endif ()


##########################################################################
# SOURCE FILES

if (${APPLE})
  message ("MAC OS X")
  set (OS_SPECIFIC_FILES
    ${PROJECT_SOURCE_DIR}/MetalViewController.m
    ${PROJECT_SOURCE_DIR}/MetalRenderer.m
    ${PROJECT_SOURCE_DIR}/MetalMTKView.m
    ${PROJECT_SOURCE_DIR}/MetalCamera.cpp
    ${storyboard_info}
    ${metal_file}
    )
else()
  message ("NOT MAC OS X")
  set (OS_SPECIFIC_FILES
    ${PROJECT_SOURCE_DIR}/OpenGLCanvas.cpp
    ${PROJECT_SOURCE_DIR}/OpenGLRenderer.cpp
    )
endif ()

set (SRCS
  ${PROJECT_SOURCE_DIR}/argparser.h
  ${PROJECT_SOURCE_DIR}/argparser.cpp
  ${PROJECT_SOURCE_DIR}/boundingbox.h
  ${PROJECT_SOURCE_DIR}/camera.h
  ${PROJECT_SOURCE_DIR}/camera.cpp
  ${PROJECT_SOURCE_DIR}/cylinder_ring.h
  ${PROJECT_SOURCE_DIR}/cylinder_ring.cpp
  ${PROJECT_SOURCE_DIR}/edge.h
  ${PROJECT_SOURCE_DIR}/edge.cpp
  ${PROJECT_SOURCE_DIR}/face.h
  ${PROJECT_SOURCE_DIR}/face.cpp
  ${PROJECT_SOURCE_DIR}/hash.h
  ${PROJECT_SOURCE_DIR}/hit.h
  ${PROJECT_SOURCE_DIR}/image.h
  ${PROJECT_SOURCE_DIR}/image.cpp
  ${PROJECT_SOURCE_DIR}/kdtree.h
  ${PROJECT_SOURCE_DIR}/kdtree.cpp
  ${PROJECT_SOURCE_DIR}/main.cpp
  ${PROJECT_SOURCE_DIR}/material.h
  ${PROJECT_SOURCE_DIR}/material.cpp
  ${PROJECT_SOURCE_DIR}/matrix.h
  ${PROJECT_SOURCE_DIR}/matrix.cpp
  ${PROJECT_SOURCE_DIR}/mesh.h
  ${PROJECT_SOURCE_DIR}/mesh.cpp
  ${PROJECT_SOURCE_DIR}/meshdata.h
  ${PROJECT_SOURCE_DIR}/meshdata.cpp
  ${PROJECT_SOURCE_DIR}/photon.h
  ${PROJECT_SOURCE_DIR}/photon_mapping.h  
  ${PROJECT_SOURCE_DIR}/photon_mapping.cpp
  ${PROJECT_SOURCE_DIR}/primitive.h
  ${PROJECT_SOURCE_DIR}/radiosity.h
  ${PROJECT_SOURCE_DIR}/radiosity.cpp
  ${PROJECT_SOURCE_DIR}/ray.h
  ${PROJECT_SOURCE_DIR}/raytracer.h
  ${PROJECT_SOURCE_DIR}/raytracer.cpp
  ${PROJECT_SOURCE_DIR}/raytree.h
  ${PROJECT_SOURCE_DIR}/raytree.cpp
  ${PROJECT_SOURCE_DIR}/sphere.h
  ${PROJECT_SOURCE_DIR}/sphere.cpp
  ${PROJECT_SOURCE_DIR}/utils.h
  ${PROJECT_SOURCE_DIR}/utils.cpp
  ${PROJECT_SOURCE_DIR}/vectors.h
  ${PROJECT_SOURCE_DIR}/vertex.h 
  ${OS_SPECIFIC_FILES}
  )

# all the .cpp files that make up this project
add_executable(${my_executable}
  MACOSX_BUNDLE
  ${SRCS}
  )


##########################################################################
# LIBRARIES AND HEADERS

if (${APPLE})

  # nothing here

else()

  # make sure all of the necessary graphics libraries are available
  find_package(OpenGL REQUIRED)
  if(OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIRS})
    target_link_libraries(${my_executable} ${OPENGL_LIBRARIES} )
  endif(OPENGL_FOUND)
  find_package(GLEW REQUIRED)
  if(GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    target_link_libraries(${my_executable} ${GLEW_LIBRARIES})
  endif(GLEW_FOUND)
  find_package(GLM REQUIRED)
  if(GLM_FOUND)
    include_directories(${GLM_INCLUDE_DIRS})
  endif()
  # find all the dependencies of GLFW
  set(ENV{PKG_CONFIG_PATH} /usr/local/lib/pkgconfig:/usr/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
  find_package(PkgConfig)
  if(PKG_CONFIG_FOUND)
    pkg_search_module(GLFW REQUIRED glfw3)
    include_directories(${GLFW_INCLUDE_DIRS})
  else(PKG_CONFIG_FOUND)
    message("Did not find pkg-config, trying FindGLFW.cmake")
    find_package(GLFW REQUIRED)
    if(GLFW_FOUND)
      include_directories(${GLFW_INCLUDE_DIR})
    else(GLFW_FOUND)
    endif(GLFW_FOUND)
  endif(PKG_CONFIG_FOUND)
  message(STATUS "OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")
  message(STATUS "GLEW_LIBRARIES: ${GLEW_LIBRARIES}")
  message(STATUS "GLFW_LIBRARIES: ${GLFW_LIBRARIES}")
  message(STATUS "GLFW_STATIC_LIBRARIES: ${GLFW_STATIC_LIBRARIES}")
  message(STATUS "GLFW_LDFLAGS: ${GLFW_LDFLAGS}")
  message(STATUS "GLFW_STATIC_LDFLAGS: ${GLFW_STATIC_LDFLAGS}")

  # some linux compilations require this hack to get the libraries in the right order
  if (UNIX)
    set(MISSING_FLAGS "-lX11 -lXxf86vm -lXrandr -lpthread -lXi -lXinerama -lXcursor -lrt -ldl")
    target_link_libraries(${my_executable} "${OPENGL_gl_LIBRARY}" "${GLEW_LIBRARIES}"  "${GLFW_LIBRARIES}" "${MISSING_FLAGS}")
  endif()

  target_link_libraries(${my_executable} "${OPENGL_gl_LIBRARY}" "${GLEW_LIBRARIES}" "${GLFW_LIBRARIES}")
  ## this will hopefully work whether you have the static or the dynamic GLFW libraries
  # string replace hack to fix a bug in the pkg_config information
  string(REPLACE ";" " " flags_static "${GLFW_STATIC_LDFLAGS}")
  string(REPLACE ";" " " flags_dynamic "${GLFW_LDFLAGS}")
  set_property(TARGET ${my_executable} APPEND_STRING PROPERTY LINK_FLAGS "${flags_static} ${flags_dynamic}")

endif()
  
##########################################################################
# COMPILATION FLAGS
if (APPLE)

  set(MY_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wextra")

  # src specific compilation flags (to mix c++ and objective c)
  # http://ysflight.in.coocan.jp/programming/cmake/e.html
  foreach(SRC ${SRCS})
    if(${SRC} MATCHES .cpp$)
      # cpp files
      set_source_files_properties(
        ${SRC} 
        PROPERTIES 
        COMPILE_FLAGS 
        "${MY_CMAKE_CXX_FLAGS}")
    elseif(${SRC} MATCHES m$)
      # objective c files
      set_source_files_properties(
        ${SRC} 
        PROPERTIES 
        COMPILE_FLAGS 
        "-x objective-c -fmodules")
    endif()
  endforeach(SRC)
  
  # more mac specific things
  set_target_properties(${my_executable} PROPERTIES LINK_FLAGS "-framework MetalKit")
  set_target_properties(${my_executable} PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${PROJECT_SOURCE_DIR}/MetalInfo.plist)

else()
  if (UNIX)
    # LINUX
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    set_target_properties (${my_executable} PROPERTIES COMPILE_FLAGS "-g -Wall -Wextra -pedantic ${BUILD_32}")
  else()
    # WINDOWS
    set_target_properties (${my_executable} PROPERTIES COMPILE_FLAGS "/W4")
  endif()
endif()

##########################################################################
