PROJECT ( ReferenceIKTest )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

FIND_PACKAGE(Simox REQUIRED)
FIND_PACKAGE(Coin3D REQUIRED)
FIND_PACKAGE(Bipedal REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
  INCLUDE_DIRECTORIES(${BIPEDAL_INCLUDE_DIRS})

  # the variable "demo_SRCS" contains all .cpp files of this project
  FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/ReferenceIKTest.cpp ${PROJECT_SOURCE_DIR}/Window.cpp)
  FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/Window.h)


  ################################## moc'ing ##############################
  set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/Window.h)
  set(GUI_UIS ${PROJECT_SOURCE_DIR}/Window.ui)

  # create the executable
  VirtualRobotQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")

  MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
  MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " ${INSTALL_BIN_DIR})

ENDIF()
