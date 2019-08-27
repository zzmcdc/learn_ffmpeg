file(GLOB MOD_SRCS
    *.cpp
    )
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main  ${DEPENCES_LIBS})
