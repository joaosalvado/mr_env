add_executable(mrenv_exe main.cpp)
target_link_libraries(mrenv_exe mrenv)


add_executable(tesselate_exe tesselate.cpp)
target_link_libraries(tesselate_exe mrenv)