cmake_minimum_required(VERSION 3.13)
project(mavlink_hub_heartbeat_generator VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

find_package(PendarlabMavlinkHubSdk 1.0.0 REQUIRED)

add_library(MavlinkHubHeartbeatGenerator SHARED
  src/HeartbeatAgent.cpp
  src/HeartbeatMessageBuilder.cpp
  src/HeartbeatSender.cpp
  src/PeriodicSender.cpp
  src/HeartbeatAgentDefinition.cpp
)
target_include_directories(MavlinkHubHeartbeatGenerator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(MavlinkHubHeartbeatGenerator
  PRIVATE pendarlab::MavlinkHubSdk
)
set_target_properties(MavlinkHubHeartbeatGenerator PROPERTIES OUTPUT_NAME mavlink_hub_heartbeat_generator)

find_package(GTest CONFIG REQUIRED)

enable_testing()

add_executable(PeriodicSenderTest
  test/periodic_sender_test.cpp
)
target_include_directories(PeriodicSenderTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(PeriodicSenderTest
  PRIVATE MavlinkHubHeartbeatGenerator
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE GTest::gtest_main
)
add_test(NAME PeriodicSenderTest COMMAND PeriodicSenderTest)

add_executable(HeartbeatAgentParseConfigTest
  test/heartbeat_agent_definition_parse_config_test.cpp
)
target_include_directories(HeartbeatAgentParseConfigTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(HeartbeatAgentParseConfigTest
  PRIVATE MavlinkHubHeartbeatGenerator
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE GTest::gtest_main
)
add_test(NAME HeartbeatAgentParseConfigTest COMMAND HeartbeatAgentParseConfigTest)

add_executable(HeartbeatMessageTest
  test/heartbeat_message_test.cpp
)
target_include_directories(HeartbeatMessageTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(HeartbeatMessageTest
  PRIVATE MavlinkHubHeartbeatGenerator
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE GTest::gtest_main
)
add_test(NAME HeartbeatMessageTest COMMAND HeartbeatMessageTest)

add_executable(HeartbeatAgentTargetingTest
  test/heartbeat_agent_targeting_test.cpp
)
target_include_directories(HeartbeatAgentTargetingTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(HeartbeatAgentTargetingTest
  PRIVATE MavlinkHubHeartbeatGenerator
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE GTest::gtest_main
)
add_test(NAME HeartbeatAgentTargetingTest COMMAND HeartbeatAgentTargetingTest)

