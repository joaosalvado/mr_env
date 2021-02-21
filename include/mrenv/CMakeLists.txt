add_library(mrenv STATIC Tesselation.h Tesselation.cpp)
target_include_directories(mrenv PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries( mrenv
    ${OpenCV_LIBS}
    cdd cddgmp iomp5
    iris_cvxgen_ldp_cpp iris_cvxgen_ldp 
    iris_geometry iris_mosek mosek64
    iris
)