add_library(mrenv STATIC Tesselation.h Tesselation.cpp)
target_include_directories(mrenv PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        /usr/local/include/yaml-cpp/)
target_link_libraries( mrenv PUBLIC
        ${OpenCV_LIBS}
        /usr/local/lib/libyaml-cpp.so
)