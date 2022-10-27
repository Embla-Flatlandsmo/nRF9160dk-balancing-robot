cmake_minimum_required(VERSION 3.20.0)

if(CONFIG_TB6612FNG)
target_sources(app PRIVATE
    tb6612fng_motor.c
)
endif()