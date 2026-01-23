cmake_minimum_required(VERSION 3.16)

project(rhex_motor_testing LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optional external Pi3Hat SDK paths.
set(PI3HAT_INCLUDE_DIR "" CACHE PATH "Path to pi3hat headers (pi3hat.h)")
set(PI3HAT_LIBRARY "" CACHE FILEPATH "Path to pi3hat library")

# add_executable(rhex_motor_testing
#         src/motor_test.cpp
#         src/cubemars_pi3hat.cpp
#         src/pi3hat.cpp
# )

# add_executable(rhex_stand_test
#         # src/stand_test.cpp
#         src/cubemars_pi3hat.cpp
#         src/pi3hat.cpp
#         # src/stand_test_v1.cpp
#         src/stand_test_v2.cpp
# )

# add_executable(rhex_zero_test
#         src/zero_motors_test.cpp
#         src/cubemars_pi3hat.cpp
#         # src/zero_motors_main.cpp
#         # src/zero_motors_main_exp.cpp
#         src/pi3hat.cpp
# )

add_executable(rhex_tripod_test
        # src/tripod_test.cpp #original tripod test to move legs in tripod gait
        src/stand_sit_tripod_main.cpp #modified to add stand/sit and return home functionality
        src/cubemars_pi3hat.cpp
        src/pi3hat.cpp
        #src/tripod_test_zero.cpp #for zeroing test only with tripod and return

)


#### MAIN CODE #### 

# add_executable(rhex_tripod_main
#         src/tripod_main.cpp
#         src/cubemars_pi3hat.cpp
#         src/pi3hat.cpp
# )

# add_executable(rhex_stand_main
#         src/stand_main.cpp
#         src/cubemars_pi3hat.cpp
#         src/pi3hat.cpp
# )

add_executable(rhex_stand_sit_test
        src/stand_sit_main.cpp
        src/cubemars_pi3hat.cpp
        src/pi3hat.cpp
)

# target_include_directories(rhex_motor_testing
#     PRIVATE
#         include
# )

# target_include_directories(rhex_stand_test
#     PRIVATE
#         include
# )

# target_include_directories(rhex_zero_test
#     PRIVATE
#         include
# )

target_include_directories(rhex_tripod_test
    PRIVATE
        include
)

# target_include_directories(rhex_tripod_main
#     PRIVATE
#         include
# )

# target_include_directories(rhex_stand_main
#     PRIVATE
#         include
# )
target_include_directories(rhex_stand_sit_test
    PRIVATE
        include
)


if(PI3HAT_INCLUDE_DIR)
    # target_include_directories(rhex_stand_test PRIVATE "${PI3HAT_INCLUDE_DIR}")
    target_include_directories(rhex_stand_sit_test PRIVATE "${PI3HAT_INCLUDE_DIR}")
    # target_include_directories(rhex_motor_testing PRIVATE "${PI3HAT_INCLUDE_DIR}")
    # target_include_directories(rhex_zero_test PRIVATE "${PI3HAT_INCLUDE_DIR}")
    target_include_directories(rhex_tripod_test PRIVATE "${PI3HAT_INCLUDE_DIR}")
    # target_include_directories(rhex_tripod_main PRIVATE "${PI3HAT_INCLUDE_DIR}")
    # target_include_directories(rhex_stand_main PRIVATE "${PI3HAT_INCLUDE_DIR}")
endif()

if(PI3HAT_LIBRARY)
    # target_link_libraries(rhex_stand_test PRIVATE "${PI3HAT_LIBRARY}")
    target_link_libraries(rhex_stand_sit_test PRIVATE "${PI3HAT_LIBRARY}")
    # target_link_libraries(rhex_motor_testing PRIVATE "${PI3HAT_LIBRARY}")
    # target_link_libraries(rhex_zero_test PRIVATE "${PI3HAT_LIBRARY}")
    target_link_libraries(rhex_tripod_test PRIVATE "${PI3HAT_LIBRARY}")
    # target_link_libraries(rhex_tripod_main PRIVATE "${PI3HAT_LIBRARY}")
    # target_link_libraries(rhex_stand_main PRIVATE "${PI3HAT_LIBRARY}")
endif()

find_library(BCM_HOST_LIB bcm_host)
if(BCM_HOST_LIB)
    # target_link_libraries(rhex_stand_test PRIVATE "${BCM_HOST_LIB}")
    target_link_libraries(rhex_stand_sit_test PRIVATE "${BCM_HOST_LIB}")
    # target_link_libraries(rhex_motor_testing PRIVATE "${BCM_HOST_LIB}")
    # target_link_libraries(rhex_zero_test PRIVATE "${BCM_HOST_LIB}")
    target_link_libraries(rhex_tripod_test PRIVATE "${BCM_HOST_LIB}")
    # target_link_libraries(rhex_tripod_main PRIVATE "${BCM_HOST_LIB}")
    # target_link_libraries(rhex_stand_main PRIVATE "${BCM_HOST_LIB}")
endif()
