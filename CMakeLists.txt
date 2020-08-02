cmake_minimum_required(VERSION 3.5)
#cmake_minimum_required(VERSION 3.16.0)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(UVC_Cabinet_Ctrl)

# To be activate with ssl certificates
#target_add_binary_data(mqtt_ssl.elf "src/mqtt_eclipse_org.pem" TEXT)
#target_add_binary_data(mqtt_ssl.elf "main/mqtt_eclipse_org.pem" TEXT)printen