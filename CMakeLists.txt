cmake_minimum_required(VERSION 3.3)
project(magisterka)

set(SYSTEM x86-64_sles10_4.1)
set(LIBFORMAT static_pic)

set(CMTFLAGS "-DILOUSEMT -D_REENTRANT")
set(DEBUG "-O3 -fomit-frame-pointer -DNDEBUG")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DIL_STD ${DEBUG} ${CMTFLAGS}")
set(SCHEDDIR /opt/ibm/ILOG/CP17/scheduler)
set(SOLVERDIR ${SCHEDDIR}/../solver)
set(CONCERTDIR ${SCHEDDIR}/../concert)

include_directories(${SCHEDDIR}/include ${SOLVERDIR}/include ${CONCERTDIR}/include)
link_directories(${SCHEDDIR}/lib/${SYSTEM}/${LIBFORMAT} ${SOLVERDIR}/lib/${SYSTEM}/${LIBFORMAT} ${CONCERTDIR}/lib/${SYSTEM}/${LIBFORMAT})

set(SOURCE_FILES basic_scheduler.cpp)
add_executable(basic_scheduler ${SOURCE_FILES})

target_link_libraries(basic_scheduler schedule solveriim solver concert pthread)