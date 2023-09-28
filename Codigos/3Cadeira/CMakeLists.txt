cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(3Cadeira)
#[[idf_component_register(SRCS "app_main.c"
INCLUDE_DIRS ""
REQUIRES soc nvs_flash ulp driver)
set(ulp_app_name ulp_main)
set(ulp_s_sources "../ulp/ulp.S")
set(ulp_exp_dep_srcs "main.c")
ulp_embed_binary(${ulp_app_name} ${ulp_s_sources} ${ulp_
exp_dep_srcs})]]