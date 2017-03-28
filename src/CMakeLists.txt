
SET(SOURCES 
  box.cpp
  main.cpp
  p1task.cpp
  p2task.cpp
  p3task.cpp
  p3creator.cpp
  problem.cpp
  result.cpp
  solutions.cpp)

ADD_EXECUTABLE(kppm ${SOURCES})
TARGET_LINK_LIBRARIES(kppm ${Boost_PROGRAM_OPTIONS_LIBRARY} ${CPLEX_LIBRARY})
