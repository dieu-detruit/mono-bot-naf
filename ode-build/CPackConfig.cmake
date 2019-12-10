# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


SET(CPACK_BINARY_7Z "")
SET(CPACK_BINARY_BUNDLE "")
SET(CPACK_BINARY_CYGWIN "")
SET(CPACK_BINARY_DEB "OFF")
SET(CPACK_BINARY_DRAGNDROP "")
SET(CPACK_BINARY_FREEBSD "OFF")
SET(CPACK_BINARY_IFW "OFF")
SET(CPACK_BINARY_NSIS "OFF")
SET(CPACK_BINARY_OSXX11 "")
SET(CPACK_BINARY_PACKAGEMAKER "")
SET(CPACK_BINARY_PRODUCTBUILD "")
SET(CPACK_BINARY_RPM "OFF")
SET(CPACK_BINARY_STGZ "ON")
SET(CPACK_BINARY_TBZ2 "OFF")
SET(CPACK_BINARY_TGZ "ON")
SET(CPACK_BINARY_TXZ "OFF")
SET(CPACK_BINARY_TZ "ON")
SET(CPACK_BINARY_WIX "")
SET(CPACK_BINARY_ZIP "")
SET(CPACK_BUILD_SOURCE_DIRS "/home/dieu_detruit/3A/simulation/ode/ode-0.16;/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build")
SET(CPACK_CMAKE_GENERATOR "Unix Makefiles")
SET(CPACK_COMPONENTS_ALL "development;runtime")
SET(CPACK_COMPONENT_DEVELOPMENT_DEPENDS "runtime")
SET(CPACK_COMPONENT_DEVELOPMENT_DESCRIPTION "Open Dynamics Engine - development files
ODE is a free, industrial quality library for simulating articulated rigid body dynamics - for example ground vehicles, legged creatures, and moving objects in VR environments. It is fast, flexible, robust and platform independent, with advanced joints, contact with friction, and built-in collision detection.")
SET(CPACK_COMPONENT_RUNTIME_DESCRIPTION "Open Dynamics Engine - runtime library
ODE is a free, industrial quality library for simulating articulated rigid body dynamics - for example ground vehicles, legged creatures, and moving objects in VR environments. It is fast, flexible, robust and platform independent, with advanced joints, contact with friction, and built-in collision detection.")
SET(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
SET(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
SET(CPACK_DEBIAN_DEVELOPMENT_FILE_NAME "DEB-DEFAULT")
SET(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_CONFLICTS "libode-sp-dev")
SET(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_DEPENDS "libode6")
SET(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_NAME "libode-dev")
SET(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_SECTION "libdevel")
SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "ODE is a free, industrial quality library for simulating articulated rigid body dynamics - for example ground vehicles, legged creatures, and moving objects in VR environments. It is fast, flexible, robust and platform independent, with advanced joints, contact with friction, and built-in collision detection.")
SET(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://www.ode.org/")
SET(CPACK_DEBIAN_PACKAGE_NAME "libode")
SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
SET(CPACK_DEBIAN_RUNTIME_FILE_NAME "DEB-DEFAULT")
SET(CPACK_DEBIAN_RUNTIME_PACKAGE_CONFLICTS "libode6sp")
SET(CPACK_DEBIAN_RUNTIME_PACKAGE_NAME "libode6")
SET(CPACK_DEBIAN_RUNTIME_PACKAGE_SECTION "libs")
SET(CPACK_DEBIAN_RUNTIME_PACKAGE_SHLIBDEPS "ON")
SET(CPACK_DEB_COMPONENT_INSTALL "ON")
SET(CPACK_GENERATOR "STGZ;TGZ;TZ")
SET(CPACK_INSTALL_CMAKE_PROJECTS "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build;ODE;ALL;/")
SET(CPACK_INSTALL_PREFIX "/usr/local")
SET(CPACK_MODULE_PATH "")
SET(CPACK_NSIS_DISPLAY_NAME "ode-0.16.0")
SET(CPACK_NSIS_INSTALLER_ICON_CODE "")
SET(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
SET(CPACK_NSIS_PACKAGE_NAME "ODE 0.16.0")
SET(CPACK_NSIS_URL_INFO_ABOUT "http://www.ode.org/")
SET(CPACK_OUTPUT_CONFIG_FILE "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CPackConfig.cmake")
SET(CPACK_PACKAGE_CONTACT "ode@ode.org")
SET(CPACK_PACKAGE_DEFAULT_LOCATION "/")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-3.10/Templates/CPack.GenericDescription.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "High performance library for simulating rigid body dynamics")
SET(CPACK_PACKAGE_DISPLAY_NAME "ODE 0.16.0")
SET(CPACK_PACKAGE_FILE_NAME "ode-0.16.0-Linux")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "ode-0.16.0")
SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "ode-0.16.0")
SET(CPACK_PACKAGE_NAME "ode")
SET(CPACK_PACKAGE_RELOCATABLE "true")
SET(CPACK_PACKAGE_VENDOR "")
SET(CPACK_PACKAGE_VERSION "0.16.0")
SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "16")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_RESOURCE_FILE_LICENSE "/home/dieu_detruit/3A/simulation/ode/ode-0.16/COPYING")
SET(CPACK_RESOURCE_FILE_README "/usr/share/cmake-3.10/Templates/CPack.GenericDescription.txt")
SET(CPACK_RESOURCE_FILE_WELCOME "/usr/share/cmake-3.10/Templates/CPack.GenericWelcome.txt")
SET(CPACK_RPM_COMPONENT_INSTALL "ON")
SET(CPACK_RPM_DEVELOPMENT_FILE_NAME "RPM-DEFAULT")
SET(CPACK_RPM_PACKAGE_DESCRIPTION "ODE is an open source, high performance library for simulating rigid body dynamics. It is fully featured, stable, mature and platform independent with an easy to use C/C++ API. It has advanced joint types and integrated collision detection with friction. ODE is useful for simulating vehicles, objects in virtual reality environments and virtual creatures. It is currently used in many computer games, 3D authoring tools and simulation tools.")
SET(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
SET(CPACK_RPM_PACKAGE_LICENSE "BSD or LGPLv2+")
SET(CPACK_RPM_PACKAGE_NAME "ode")
SET(CPACK_RPM_PACKAGE_SUMMARY "High performance library for simulating rigid body dynamics")
SET(CPACK_RPM_PACKAGE_URL "http://www.ode.org/")
SET(CPACK_RPM_RUNTIME_FILE_NAME "RPM-DEFAULT")
SET(CPACK_RPM_development_PACKAGE_DESCRIPTION "The ode-devel package contains libraries and header files for developing applications that use ode or ode-double.")
SET(CPACK_RPM_development_PACKAGE_NAME "ode-devel")
SET(CPACK_RPM_development_PACKAGE_REQUIRES "ode-double")
SET(CPACK_RPM_development_PACKAGE_SUMMARY "Development files for ODE")
SET(CPACK_RPM_runtime_PACKAGE_CONFLICTS "ode")
SET(CPACK_RPM_runtime_PACKAGE_DESCRIPTION "The ode-double package contains a version of the ODE library for simulating rigid body dynamics compiled with double precision.")
SET(CPACK_RPM_runtime_PACKAGE_NAME "ode-double")
SET(CPACK_RPM_runtime_PACKAGE_SUMMARY "ODE physics library compiled with double precision")
SET(CPACK_SET_DESTDIR "OFF")
SET(CPACK_SOURCE_7Z "")
SET(CPACK_SOURCE_CYGWIN "")
SET(CPACK_SOURCE_GENERATOR "TBZ2;TGZ;TXZ;TZ")
SET(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CPackSourceConfig.cmake")
SET(CPACK_SOURCE_RPM "OFF")
SET(CPACK_SOURCE_TBZ2 "ON")
SET(CPACK_SOURCE_TGZ "ON")
SET(CPACK_SOURCE_TXZ "ON")
SET(CPACK_SOURCE_TZ "ON")
SET(CPACK_SOURCE_ZIP "OFF")
SET(CPACK_SYSTEM_NAME "Linux")
SET(CPACK_TOPLEVEL_TAG "Linux")
SET(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/home/dieu_detruit/3A/simulation/ode/ode-0.16/ode-build/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
