#################################################################################
#
# Copyright (c) 2018-2019 PX4 Pro Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##################################################################################

##################################################################################
# Cmake module to generate micro-RTPS agent code Depends on:
#   - px4_msgs
#   - templates/urtps_bridge_topics.yaml
#   - scripts/uorb_rtps_classifier.py
#   - scripts/generate_microRTPS_bridge.py
#   - FASTRTPSGEN_DIR
##################################################################################

# Check if FastRTPSGen exists
if($ENV{FASTRTPSGEN_DIR})
  set(FASTRTPSGEN_DIR $ENV{FASTRTPSGEN_DIR})
  message(STATUS "fastrtpsgen found in ${FASTRTPSGEN_DIR}}")
  set(FASTRTPSGEN "${FASTRTPSGEN_DIR}/fastrtpsgen")
else()
  find_file(FASTRTPSGEN NAMES fastrtpsgen PATH_SUFFIXES bin)
  if(FASTRTPSGEN-NOTFOUND)
    message(FATAL_ERROR "fastrtpsgen not found")
  else()
    get_filename_component(FASTRTPSGEN_DIR ${FASTRTPSGEN} DIRECTORY)
    message(STATUS "fastrtpsgen found in ${FASTRTPSGEN_DIR}")
  endif()
endif()

# Get FastRTPSGen version
set(FASTRTPSGEN_VERSION)
execute_process(COMMAND ${FASTRTPSGEN} -version
  OUTPUT_VARIABLE FASTRTPSGEN_VERSION_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
string(REGEX MATCH
  "([0-9]+)\\.([0-9]+)\\.([0-9]+)$"
  FASTRTPSGEN_VERSION
  "${FASTRTPSGEN_VERSION_OUTPUT}")
# If the above command fails, force FastRTPSGen to 1.0.0
if(NOT FASTRTPSGEN_VERSION)
  set(FASTRTPSGEN_VERSION "1.0.0")
endif()
message(STATUS "fastrtpsgen version ${FASTRTPSGEN_VERSION}")

# Set IDL dir
set(IDL_DIR ${MSGS_DIR})
message(STATUS "px4_msgs messages under ${MSGS_DIR}")
message(STATUS "IDL definitions under ${IDL_DIR}")

# Check if the RTPS ID's mapper yaml file exists and if yes, configure topics to generate code for
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/templates/urtps_bridge_topics.yaml)
  # Create list of messages to send
  set(CONFIG_RTPS_SEND_TOPICS)
  message(STATUS "Retrieving list of msgs to send...")
  # The msgs the client receives, are the messages the agent sends
  execute_process(COMMAND ${PYTHON_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/uorb_rtps_classifier.py"
    --receive
    --alias
    --topic-msg-dir "${MSGS_DIR}"
    --rtps-ids-file "${CMAKE_CURRENT_SOURCE_DIR}/src/templates/urtps_bridge_topics.yaml"
    OUTPUT_VARIABLE CONFIG_RTPS_SEND_TOPICS)
  set(CONFIG_RTPS_SEND_ALIAS_TOPICS "")
  string(FIND ${CONFIG_RTPS_SEND_TOPICS} "alias" found_send_alias)
  if (NOT ${found_send_alias} EQUAL "-1")
    string(REGEX REPLACE ".*alias " "" CONFIG_RTPS_SEND_ALIAS_TOPICS "${CONFIG_RTPS_SEND_TOPICS}")
    string(REPLACE "\n" "" CONFIG_RTPS_SEND_ALIAS_TOPICS "${CONFIG_RTPS_SEND_ALIAS_TOPICS}")
    string(REGEX REPLACE " alias.*" "" CONFIG_RTPS_SEND_TOPICS "${CONFIG_RTPS_SEND_TOPICS}")
  endif()
  string(REGEX REPLACE "\n" "" CONFIG_RTPS_SEND_TOPICS "${CONFIG_RTPS_SEND_TOPICS}")
  message(STATUS "List of msgs to send: ${CONFIG_RTPS_SEND_TOPICS}, ${CONFIG_RTPS_SEND_ALIAS_TOPICS}")
  string(REPLACE ", " ";" CONFIG_RTPS_SEND_ALIAS_TOPICS "${CONFIG_RTPS_SEND_ALIAS_TOPICS}")
  string(REGEX REPLACE ", " ";" CONFIG_RTPS_SEND_TOPICS "${CONFIG_RTPS_SEND_TOPICS}")
  set(CONFIG_RTPS_SEND_TOPICS "${CONFIG_RTPS_SEND_TOPICS};${CONFIG_RTPS_SEND_ALIAS_TOPICS}")

  # Create list of messages to receive
  set(CONFIG_RTPS_RECEIVE_TOPICS)
  message(STATUS "Retrieving list of msgs to receive...")
  # The msgs the client sends, are the messages the agent receives
  execute_process(COMMAND ${PYTHON_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/uorb_rtps_classifier.py"
    --send
    --alias
    --topic-msg-dir "${MSGS_DIR}"
    --rtps-ids-file "${CMAKE_CURRENT_SOURCE_DIR}/src/templates/urtps_bridge_topics.yaml"
    OUTPUT_VARIABLE CONFIG_RTPS_RECEIVE_TOPICS)
  set(CONFIG_RTPS_RECEIVE_ALIAS_TOPICS "")
  string(FIND ${CONFIG_RTPS_RECEIVE_TOPICS} "alias" found_receive_alias)
  if (NOT ${found_receive_alias} EQUAL "-1")
    STRING(REGEX REPLACE ".*alias " "" CONFIG_RTPS_RECEIVE_ALIAS_TOPICS "${CONFIG_RTPS_RECEIVE_TOPICS}")
    STRING(REPLACE "\n" "" CONFIG_RTPS_RECEIVE_ALIAS_TOPICS "${CONFIG_RTPS_RECEIVE_ALIAS_TOPICS}")
    STRING(REGEX REPLACE " alias.*" "" CONFIG_RTPS_RECEIVE_TOPICS "${CONFIG_RTPS_RECEIVE_TOPICS}")
  endif()
  string(REGEX REPLACE "\n" "" CONFIG_RTPS_RECEIVE_TOPICS "${CONFIG_RTPS_RECEIVE_TOPICS}")
  message(STATUS "List of msgs to receive: ${CONFIG_RTPS_RECEIVE_TOPICS}")
  STRING(REPLACE ", " ";" CONFIG_RTPS_RECEIVE_ALIAS_TOPICS "${CONFIG_RTPS_RECEIVE_ALIAS_TOPICS}")
  string(REGEX REPLACE ", " ";" CONFIG_RTPS_RECEIVE_TOPICS "${CONFIG_RTPS_RECEIVE_TOPICS}")
  set(CONFIG_RTPS_RECEIVE_TOPICS "${CONFIG_RTPS_RECEIVE_TOPICS};${CONFIG_RTPS_RECEIVE_ALIAS_TOPICS}")
else()
  message(FATAL_ERROR "RTPS msg ID yaml file not found!")
endif()

# Set the microRTPS Agent generated code directory
set(MICRORTPS_AGENT_DIR "${CMAKE_BINARY_DIR}/src/micrortps_agent" CACHE INTERNAL "MICRORTPS_AGENT_DIR")
file(MAKE_DIRECTORY ${MICRORTPS_AGENT_DIR})
message(STATUS "Generating Agent code in ${MICRORTPS_AGENT_DIR}")

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/micrortps_agent/params.yaml"
  OUT_FILE "src/micrortps_agent/init_parameters.cpp")

# Copy Timesync source code
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/micrortps_agent/timesync.cpp
  ${MICRORTPS_AGENT_DIR}/timesync.cpp
  COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/micrortps_agent/timesync.hpp
  ${MICRORTPS_AGENT_DIR}/timesync.hpp
  COPYONLY)

# Set the list of files to be compiled
set(MICRORTPS_AGENT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/micrortps_agent/microRTPS_agent.cpp)
list(APPEND MICRORTPS_AGENT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/micrortps_agent/microRTPS_agent_utils.cpp)
list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/timesync.cpp)
list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/init_parameters.cpp)
list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/RTPSTopics.hpp)
list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/RTPSTopics.cpp)

# Configure generated source file names for message types
set(ALL_TOPIC_NAMES ${CONFIG_RTPS_SEND_TOPICS} ${CONFIG_RTPS_RECEIVE_TOPICS})
list(REMOVE_DUPLICATES ALL_TOPIC_NAMES)
foreach(topic ${ALL_TOPIC_NAMES})
  # Requires differentiation between FastRTPSGen versions
  if(FASTRTPSGEN_VERSION VERSION_GREATER 1.4 AND FASTRTPSGEN_VERSION VERSION_LESS 1.8)
    list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}_.cpp)
  else()
    list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}.cpp)
  endif()
  if(FASTRTPSGEN_VERSION VERSION_GREATER 1.4 AND FASTRTPSGEN_VERSION VERSION_LESS 1.8)
    list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}_PubSubTypes.cpp)
    list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}_PubSubTypes.h)
  else()
    list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}PubSubTypes.cpp)
    list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}PubSubTypes.h)
  endif()
endforeach()

# Configure generated source file names for publisher and subscriber wrappers
foreach(topic ${CONFIG_RTPS_RECEIVE_TOPICS}) # received topics are to be published to the DDS layer
  list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}_Publisher.cpp)
  list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}_Publisher.hpp)
endforeach()
foreach(topic ${CONFIG_RTPS_SEND_TOPICS}) # advertised topics should be subscribed to the DDS layer
  list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}_Subscriber.cpp)
  list(APPEND MICRORTPS_AGENT_FILES ${MICRORTPS_AGENT_DIR}/${topic}_Subscriber.hpp)
endforeach()

# Define code generation script invocation as build recipe
get_filename_component(px4_msgs_FASTRTPSGEN_INCLUDE "../../" ABSOLUTE BASE_DIR ${px4_msgs_DIR})
add_custom_command(
  OUTPUT  ${MICRORTPS_AGENT_FILES}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_micrortps_agent.py
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/px_generate_uorb_topic_files.py
          ${FASTRTPSGEN_DIR}
  COMMAND
    ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_micrortps_agent.py
        --fastrtpsgen-dir ${FASTRTPSGEN_DIR}
        --fastrtpsgen-include ${px4_msgs_FASTRTPSGEN_INCLUDE}
        --topic-msg-dir ${MSGS_DIR}
        --urtps-templates-dir ${CMAKE_CURRENT_SOURCE_DIR}/src/templates
        --rtps-ids-file ${CMAKE_CURRENT_SOURCE_DIR}/src/templates/urtps_bridge_topics.yaml
        --agent-outdir ${MICRORTPS_AGENT_DIR}
        --idl-dir ${IDL_DIR}
        --ros2-distro humble
    | tee ${CMAKE_BINARY_DIR}/micrortps_bridge.log &&
    echo "microRTPS Agent code generation complete!"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating microRTPS Agent code...")

set(MICRORTPS_AGENT_FILES "${MICRORTPS_AGENT_FILES}" CACHE INTERNAL "MICRORTPS_AGENT_FILES")
