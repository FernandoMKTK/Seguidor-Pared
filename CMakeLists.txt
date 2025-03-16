cmake_minimum_required(VERSION 3.0.2)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  nav_msgs
  tf
  actionlib
  actionlib_msgs
  message_generation
)

## 1️⃣ Agregar archivos de servicio y acción antes de generate_messages()
add_service_files(
  FILES
  FindWall.srv
)

add_action_files(
  FILES
  OdomRecord.action  # No uses DIRECTORY action, solo la referencia al archivo
)

## 2️⃣ Ahora sí, generar mensajes después de definir servicios y acciones
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

## 3️⃣ Definir el paquete Catkin correctamente
catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 4️⃣ Agregar los ejecutables y definir sus dependencias
add_executable(wall_follower src/wall_follower.cpp)
add_dependencies(wall_follower ${wall_follower_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wall_follower ${catkin_LIBRARIES})

add_executable(wall_follower_client src/wall_follower_client.cpp)
add_dependencies(wall_follower_client ${wall_follower_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wall_follower_client ${catkin_LIBRARIES})

add_executable(find_wall_server src/find_wall_server.cpp)
add_dependencies(find_wall_server ${find_wall_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_wall_server ${catkin_LIBRARIES})

add_executable(record_odom_server src/record_odom_server.cpp)
add_dependencies(record_odom_server ${record_odom_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_odom_server ${catkin_LIBRARIES})
