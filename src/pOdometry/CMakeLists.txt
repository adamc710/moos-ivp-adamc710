#--------------------------------------------------------
# The CMakeLists.txt for:                 pOdometry
# Author(s):                              Adam Cohen
#--------------------------------------------------------

find_package(MOOS REQUIRED)
find_package(MOOSGeodesy REQUIRED)

include_directories(
  ${MOOS_INCLUDE_DIRS}
  ${MOOSGeodesy_INCLUDE_DIRS}
)

SET(SRC
  Odometry.cpp
  Odometry_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pOdometry ${SRC})

TARGET_LINK_LIBRARIES(pOdometry
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

