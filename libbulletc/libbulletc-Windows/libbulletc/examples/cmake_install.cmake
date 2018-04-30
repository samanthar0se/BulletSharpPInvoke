# Install script for directory: D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/src/examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/BULLET_PHYSICS")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/HelloWorld/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/BasicDemo/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/ExampleBrowser/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/RobotSimulator/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/SharedMemory/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/ThirdPartyLibs/Gwen/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/ThirdPartyLibs/BussIK/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/ThirdPartyLibs/clsocket/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/OpenGLWindow/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/TwoJoint/cmake_install.cmake")
  include("D:/DropBoxRoot/Dropbox/Unity/Unity/Bullet/newBulletBuild/ian/BulletSharpPInvoke/libbulletc/libbulletc-Windows/libbulletc/examples/pybullet/cmake_install.cmake")

endif()

