cmake_minimum_required (VERSION 2.6)
project (BRVBI2File)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
add_library(brvbi-api api/BRVBIHelper.cpp api/BRVBIControl.cpp)

add_executable(BRVBI2File Example.cpp)
target_link_libraries(BRVBI2File PRIVATE brvbi-api Threads::Threads)

