add_library( mpu6050_lib
    mpu6050.h
    mpu6050.c
    )

target_include_directories(mpu6050_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(mpu6050_lib pico_stdlib hardware_i2c)