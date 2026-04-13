ament_add_gtest(road_network_loader_test road_network_loader_test.cc)
ament_add_gtest(road_rulebook_loader_test road_rulebook_loader_test.cc)

macro(add_dependencies_to_test target)
if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::api
          maliput::base
          maliput::common
          maliput_sparse::loader
          maliput_sparse::geometry
          maliput_sparse::parser
      )

    endif()
endmacro()

add_dependencies_to_test(road_network_loader_test)
add_dependencies_to_test(road_rulebook_loader_test)
