# Install script for directory: /home/dieu_detruit/3A/simulation/ode/ode-0.16

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xruntimex" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so.0.16.0" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so.0.16.0")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so.0.16.0"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/libode.so.0.16.0")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so.0.16.0" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so.0.16.0")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so.0.16.0")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/libode.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libode.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ode" TYPE FILE FILES
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/collision.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/collision_space.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/collision_trimesh.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/common.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/compatibility.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/contact.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/cooperative.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/error.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/export-dif.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/mass.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/matrix.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/matrix_coop.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/memory.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/misc.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/objects.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/ode.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/odeconfig.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/odecpp.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/odecpp_collision.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/odeinit.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/odemath.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/odemath_legacy.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/rotation.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/threading.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/threading_impl.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/include/ode/timer.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/include/ode/precision.h"
    "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/include/ode/version.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/ode.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE PROGRAM FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/ode-config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0" TYPE FILE FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/ode-config.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0" TYPE FILE FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/ode-config-version.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0/ode-export.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0/ode-export.cmake"
         "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles/Export/lib/cmake/ode-0.16.0/ode-export.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0/ode-export-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0/ode-export.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0" TYPE FILE FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles/Export/lib/cmake/ode-0.16.0/ode-export.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/ode-0.16.0" TYPE FILE FILES "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CMakeFiles/Export/lib/cmake/ode-0.16.0/ode-export-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
