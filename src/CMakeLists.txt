
SET(SOURCES
  box.cpp
  main.cpp
  p1task.cpp
  p2task.cpp
  p3task.cpp
  p3creator.cpp
  hash.cpp
  problem.cpp
  result.cpp
  solutions.cpp)


FILE(WRITE "${CMAKE_CURRENT_BINARY_DIR}/mkhash.sh"
  "GIT_HASH=$(cd ${CMAKE_CURRENT_SOURCE_DIR} && git rev-parse HEAD)\necho \"#include <string>\nstd::string HASH = \\\"\${GIT_HASH}\\\";\" > \"\${1}/hash.cpp\""
  )

ADD_CUSTOM_TARGET(
  ALWAYS_RUN ALL
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/always_missing.h"
  )

ADD_CUSTOM_COMMAND(
  OUTPUT
  "${CMAKE_CURRENT_BINARY_DIR}/always_missing.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/hash.cpp"
  COMMAND "chmod" ARGS "u+x" "${CMAKE_CURRENT_BINARY_DIR}/mkhash.sh"
  COMMAND "${CMAKE_CURRENT_BINARY_DIR}/mkhash.sh" ARGS "${CMAKE_CURRENT_SOURCE_DIR}"
  )


ADD_EXECUTABLE(kppm ${SOURCES})
TARGET_LINK_LIBRARIES(kppm ${Boost_PROGRAM_OPTIONS_LIBRARY} ${CPLEX_LIBRARY})
