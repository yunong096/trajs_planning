cmake_minimum_required(VERSION 3.16)

# Abseil requires C++14
set(CMAKE_CXX_STANDARD 17) # 设置C++语言标准为C++11
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 确保编译器遵循所设置的语言标准

# Process Abseil's CMake build system
add_subdirectory(abseil-cpp-master)
project(testPkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# ## --- C++11 build flags ---
# add_definitions(-D_GLIBCXX_USE_C99=1)
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#set(absl_DIR "/home/lynnn/abseil-cpp-master/build/abslConfig.cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  # cmake_modules\
)

# set(AltroCpp_DIR ~/.local)  # or wherever the user installed the library via CMAKE_INSTALL_PREFIX
# find_library(AltroCpp 0.3  REQUIRED EXACT)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Development NumPy)

set(fmt_ROOT "/home/lynnn/dependpkgs/fmt-8.1.1")
set(fmt_REQUIRED_VERSION 6.1.2)
set(fmt_REQUIRED_DEBIAN_VERSION ${fmt_REQUIRED_VERSION})
find_package(fmt ${fmt_REQUIRED_VERSION} REQUIRED)

set(AltroCpp_ROOT "/home/lynnn/dependpkgs/ALTRO-iLQR-main")
find_package(AltroCpp 0.3.4 REQUIRED)

file(GLOB_RECURSE PYTHON2.7_LIB "/usr/lib/python2.7/config-x86_64-linux-gnu/*.so")
set(PYTHON2.7_INLCUDE_DIRS "/usr/include/python2.7")

# find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES testPkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
  # ${Eigen_INCLUDE_DIRS}
  /home/lynnn/test_ws/src/testPkg/include/testPkg
  /home/lynnn/test_ws/src/testPkg/include
  /home/lynnn/test_ws/src/testPkg/include/testPkg/object_msgs
  /usr/local/include/osqp
  /usr/local/include/Eigen
  /usr/local/include/osqp-eigen
  #/usr/local/include/absl
#  /home/lynnn/dependpkgs/osqp-eigen-master/include
  /usr/include/python2.7
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/testPkg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/testPkg_node.cpp)
add_executable(cfstest  src/main_test.cpp 
                                  src/SCCFS.cpp 
                                  src/CoarsePathGenerator.cpp
                                  # src/CoarsePathGenerator.cpp
                                  include/testPkg/modules/planning/planning_base/reference_line/spiral_problem_interface.cc
                                  include/testPkg/modules/common/math/math_utils.cc
                                  include/testPkg/modules/common/math/integral.cc
                                  include/testPkg/modules/planning/planning_base/math/curve1d/quintic_polynomial_curve1d.cc
                                  include/testPkg/modules/common/math/vec2d.cc
                                  include/testPkg/modules/planning/planning_base/math/curve1d/quintic_spiral_path.cc
                                  ) 

add_executable(dynamic_routing  src/dynamic_routing_node.cpp 
                                  src/MovingObs.cpp
                                  src/dynamic_routing.cpp
                                  src/SCCFS.cpp 
                                  src/CoarsePathGenerator.cpp
                                  src/CoarsePathGenerator.cpp
                                  src/dp_planner.cpp
                                  src/nmpc2.cpp
                                  src/traj_optimizer.cpp
                                  src/traj_optimizer_global.cpp
                                  src/traj_manager.cpp
                                  src/altro_optimizer.cpp
                                  include/testPkg/examples/vehicle.cpp
                                  include/testPkg/examples/quadratic_cost.cpp
                                  include/testPkg/modules/planning/planning_base/reference_line/spiral_problem_interface.cc
                                  include/testPkg/modules/common/math/math_utils.cc
                                  include/testPkg/modules/common/math/integral.cc
                                  include/testPkg/modules/planning/planning_base/math/curve1d/quintic_polynomial_curve1d.cc
                                  include/testPkg/modules/common/math/vec2d.cc
                                  include/testPkg/modules/planning/planning_base/math/curve1d/quintic_spiral_path.cc
                                  ) 

add_executable(global_routing src/global_routing_node.cpp
                                  src/global_routing.cpp
                                  src/lqr_controller.cpp
                                  src/MovingObs.cpp
                                  )

add_executable(car_simulation 
                                  src/car_state.cpp
                                  )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cfstest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(global_routing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(car_simulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

link_directories(
  # ${Eigen_LIB_DIRS}
  /usr/local
  /usr/local/lib
  /snap/gnome-3-34-1804/93/usr/lib
  /usr/local/osqp-eigen/lib
  /home/lynnn//test_ws/src/testPkg/abseil-cpp-master/build/lib
  #/home/lynnn/dependpkgs/osqp-eigen-master/build/lib
  )

# set(AltroCpp_DIR /usr/local)  # or wherever the user installed the library via CMAKE_INSTALL_PREFIX
# find_library(AltroCpp 0.3 REQUIRED EXACT)

target_link_libraries(cfstest 
  ${catkin_LIBRARIES}
  libosqp.so  
  libOsqpEigen.so 
  libqdldl.so
  
  absl::strings

  libipopt.so

  fmt::fmt
  altro::altro
  # libgtest.a
  # libfmt.a
  
  # libcommon.a
  # libthreadpool.a
  # libilqr.a
  # libproblem.a
  # libutils.a
  # libconstraints.a
  # libaugmented_lagrangian.a
  
)

target_link_libraries(dynamic_routing
  ${catkin_LIBRARIES}
  libosqp.so  
  libOsqpEigen.so 
  libqdldl.so
  absl::strings
  libipopt.so
  /usr/local/lib/libcasadi.so.3.7

  fmt::fmt
  altro::altro
)

target_link_libraries(global_routing
  ${catkin_LIBRARIES}
  libosqp.so  
  libOsqpEigen.so 
  libqdldl.so
  
  absl::strings

  libipopt.so
  # altro::altro
)

target_link_libraries(car_simulation
  ${catkin_LIBRARIES}
  libosqp.so  
  libOsqpEigen.so 
  libqdldl.so
  # altro::altro
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_testPkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
