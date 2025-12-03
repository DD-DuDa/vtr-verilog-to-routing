cmake_minimum_required(VERSION 3.16)
cmake_policy(VERSION 3.10)

project("rr_graph_transform")

add_executable(rr_graph_transform src/main.cpp)
target_link_libraries(rr_graph_transform
    libpugixml
)

install(TARGETS rr_graph_transform DESTINATION bin)

