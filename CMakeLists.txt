PROJECT ( PatternGenerator )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

FIND_PACKAGE(Simox REQUIRED)
FIND_PACKAGE(Coin3D REQUIRED)
IF(Simox_USE_COIN_VISUALIZATION)
  FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/PatternGeneratorDemo.cpp ${PROJECT_SOURCE_DIR}/PatternGeneratorWindow.cpp ${PROJECT_SOURCE_DIR}/FootstepPlaner.cpp ${PROJECT_SOURCE_DIR}/PolynomialFootstepPlaner.cpp ${PROJECT_SOURCE_DIR}/ZMPPlaner.cpp ${PROJECT_SOURCE_DIR}/ZMPPreviewControl.cpp ${PROJECT_SOURCE_DIR}/CoMCalculator.cpp)
  FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/PatternGeneratorWindow.h ${PROJECT_SOURCE_DIR}/FootstepPlaner.h ${PROJECT_SOURCE_DIR}/PolynomialFootstepPlaner.h ${PROJECT_SOURCE_DIR}/ZMPPlaner.h ${PROJECT_SOURCE_DIR}/ZMPPreviewControl.h ${PROJECT_SOURCE_DIR}/CoMCalculator.h)
  set(GUI_MOC_HDRS ${PROJECT_SOURCE_DIR}/PatternGeneratorWindow.h) 
  set(GUI_UIS ${PROJECT_SOURCE_DIR}/PatternGeneratorWindow.ui)
  SimoxQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
ENDIF() 