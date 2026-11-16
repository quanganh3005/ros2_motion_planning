cmake_minimum_required(VERSION 3.8)
project(system_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(glog REQUIRED)
find_package(common REQUIRED)
find_package(Protobuf REQUIRED)

set(PROTO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto_gen)
file(MAKE_DIRECTORY ${PROTO_OUTPUT_DIR})

file(GLOB_RECURSE PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/system_config/*.proto")

set(PROTO_SRCS "")
set(PROTO_HDRS "")
foreach(proto_file ${PROTO_FILES})
  file(RELATIVE_PATH rel_proto "${CMAKE_CURRENT_SOURCE_DIR}" "${proto_file}")
  get_filename_component(rel_dir "${rel_proto}" DIRECTORY)
  get_filename_component(proto_name "${rel_proto}" NAME_WE)
  set(src_out "${PROTO_OUTPUT_DIR}/${rel_dir}/${proto_name}.pb.cc")
  set(hdr_out "${PROTO_OUTPUT_DIR}/${rel_dir}/${proto_name}.pb.h")
  list(APPEND PROTO_SRCS "${src_out}")
  list(APPEND PROTO_HDRS "${hdr_out}")
  add_custom_command(
    OUTPUT "${src_out}" "${hdr_out}"
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${PROTO_OUTPUT_DIR} -I${CMAKE_CURRENT_SOURCE_DIR} ${proto_file}
    DEPENDS ${proto_file}
    COMMENT "Running C++ protocol buffer compiler on ${rel_proto}"
    VERBATIM
  )
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/proto_util.cpp
  src/system_config.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROTO_OUTPUT_DIR}>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  common
)

target_link_libraries(${PROJECT_NAME}
  ${Protobuf_LIBRARIES}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY ${PROTO_OUTPUT_DIR}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY system_config/
  DESTINATION share/${PROJECT_NAME}/system_config
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  common
  Protobuf
)

ament_package()
