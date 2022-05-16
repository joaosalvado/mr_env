add_library(mrenv STATIC Tesselation.h Tesselation.cpp)
target_include_directories(mrenv PUBLIC
        PUBLIC ${OpenCV_INCLUDE_DIRS}
        PUBLIC /usr/local/include/yaml-cpp/)
target_link_libraries( mrenv
    PUBLIC ${OpenCV_LIBS}
    PUBLIC    /usr/local/lib/libyaml-cpp.so
)