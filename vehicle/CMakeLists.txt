# Gazebo plugin.
add_library(gazeboVehiclePlugin SHARED ${CMAKE_CURRENT_SOURCE_DIR}/plugins/vehicle_plugin.cpp)
target_link_libraries(gazeboVehiclePlugin ${GAZEBO_LIBRARIES} ${TORCH_LIBRARIES} keyboard)

