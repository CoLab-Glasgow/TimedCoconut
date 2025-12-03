cmake_minimum_required(VERSION 3.10)


project(CoconutAll LANGUAGES C CXX ASM)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Put artifacts in the build folder root
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


option(BUILD_DESKTOP "Build for desktop (g++)" ON)
option(BUILD_AVR     "Build for AVR (avr-g++)" OFF)
option(BUILD_ARM7    "Build for ARM7TDMI (arm-none-eabi-g++)" OFF)

if( (BUILD_DESKTOP AND BUILD_AVR) OR (BUILD_DESKTOP AND BUILD_ARM7) OR (BUILD_AVR AND BUILD_ARM7) )
  message(FATAL_ERROR "Choose exactly one of BUILD_DESKTOP, BUILD_AVR, BUILD_ARM7.")
endif()


# Common (used by AVR/ARM7 )
# 
set(PLUGIN_SRC ${CMAKE_SOURCE_DIR}/src/Coconut.cpp)           # Coconut plugin source
set(AVR_APP    ${CMAKE_SOURCE_DIR}/FoodMixer/main.cpp)        # AVR/desktop app
set(ARM7_APP   ${CMAKE_SOURCE_DIR}/FoodMixer/fmarm.cpp)       # ARM7 app
set(ARM7_STARTUP ${CMAKE_SOURCE_DIR}/startup_arm7.S)          # ARM7 startup (ARM state)
set(ARM7_LINKER_SCRIPT
    ${CMAKE_SOURCE_DIR}/cmake/ldscripts/arm7.ld
    CACHE FILEPATH "ARM7 linker script (.ld)")

# Where to drop host-built GCC plugins (AVR/ARM7)
set(COCONUT_PLUGIN_DIR "${CMAKE_BINARY_DIR}/plugins")
file(MAKE_DIRECTORY "${COCONUT_PLUGIN_DIR}")

# Optional Bound-T paths (override if different)
set(BOUNDT_AVR_EXECUTABLE  "/home/adduser/boundt/bin/boundt_avr"  CACHE FILEPATH "Path to boundt_avr")
set(BOUNDT_ARM7_EXECUTABLE "/mnt/c/Users/Arwa Alsubhi/Desktop/Coconut-Project/V2/boundt_arm7/boundt_arm7"
    CACHE FILEPATH "Path to boundt_arm7")


# --- Helper: auto-run Bound-T after link, print tail of report (fixed) ---
# Tweak defaults per target at configure time:
#   -DBOUNDT_AVR_ARGS="-elf_sym -loop full -time -table -draw total"
#   -DBOUNDT_ARM7_ARGS="-loop full -time -table -draw total"

set(BOUNDT_AVR_ARGS  "-elf_sym -loop full -time -table -draw total"  CACHE STRING "Args for boundt_avr")
set(BOUNDT_ARM7_ARGS "-loop full -time -table -draw total"           CACHE STRING "Args for boundt_arm7")

function(attach_boundt_post_build TARGET_ELF BOUNDT_EXE EXTRA_ARGS)
  if(NOT EXISTS "${BOUNDT_EXE}")
    message(WARNING "Bound-T not found: ${BOUNDT_EXE} → skipping for ${TARGET_ELF}")
    return()
  endif()

  set(_BT_DIR   "${CMAKE_BINARY_DIR}/boundt")
  set(_ELF      $<TARGET_FILE:${TARGET_ELF}>)
  set(_REPORT   "${_BT_DIR}/report.txt")
  set(_DOT      "${_BT_DIR}/graphs.dot")

  # AVR-only device switch you used before
  set(_DEVICE "")
  if("${TARGET_ELF}" STREQUAL "foodmixer")
    set(_DEVICE "-device atmega644")
  endif()

  # Build the exact shell command once (no $-vars; we insert the full path directly)
  # NOTE: we use /bin/sh -lc so we can redirect stdout/stderr to a file.
  set(_CMD "\"${BOUNDT_EXE}\" ${_DEVICE} ${EXTRA_ARGS} -dot '${_DOT}' '${_ELF}' main > '${_REPORT}' 2>&1 || true")

  add_custom_command(TARGET ${TARGET_ELF} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "==> Bound-T: analyzing ${_ELF}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_BT_DIR}"
    COMMAND /bin/sh -lc "${_CMD}"
    COMMAND ${CMAKE_COMMAND} -E echo "---- Bound-T tail (${TARGET_ELF}) ----"
    COMMAND /bin/sh -lc "tail -n 40 '${_REPORT}' || true"
    BYPRODUCTS "${_REPORT}" "${_DOT}"
    VERBATIM
  )
endfunction()



if(BUILD_DESKTOP)
  # Desktop host compiler 
  find_program(GCC_COMPILER g++-13)
  if(NOT GCC_COMPILER)
    message(FATAL_ERROR "g++-13 not found. Ensure it is installed.")
  endif()
  set(CMAKE_CXX_COMPILER ${GCC_COMPILER} CACHE FILEPATH "Path to g++-13" FORCE)

  # Language / globals (as in your original)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  add_definitions(-DBOOST_ASIO_HAS_CO_AWAIT)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
  if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
  elseif(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")
  elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g -DNDEBUG")
  endif()

  
  find_package(OpenCV REQUIRED)
  find_package(OpenSSL REQUIRED)
  find_package(Boost REQUIRED COMPONENTS program_options)

  # GCC plugin headers for host plugin build
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=plugin
    OUTPUT_VARIABLE GCC_PLUGIN_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message(STATUS "GCC Plugin Directory: ${GCC_PLUGIN_DIR}")
  include_directories(
    ${GCC_PLUGIN_DIR}/include
    ${GCC_PLUGIN_DIR}/include/gcc
    ${GCC_PLUGIN_DIR}/include/gcc/cp
    ${GCC_PLUGIN_DIR}/include/gcc/c-family
  )

  # MQTT-CPP 
  set(MQTT_CPP_BUILD_EXAMPLES OFF)
  set(MQTT_CPP_LIBRARIES mqtt_cpp CACHE STRING "MQTT-CPP Library")
  set(MQTT_CPP_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/mqtt_cpp/include CACHE PATH "MQTT-CPP Include Directory")
  add_subdirectory(mqtt_cpp)

  # Boost roots 
  set(BOOST_ROOT /usr/local)
  set(BOOST_INCLUDEDIR ${BOOST_ROOT}/include)
  set(BOOST_LIBRARYDIR ${BOOST_ROOT}/lib)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${BOOST_LIBRARYDIR})

  #  Sources / targets 
  set(PLUGIN_NAME Coconut)
  set(PLUGIN_SRC  src/Coconut.cpp)

  add_library(${PLUGIN_NAME} SHARED ${PLUGIN_SRC})
  set_target_properties(${PLUGIN_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    OUTPUT_NAME "Coconut"
    INSTALL_RPATH "$ORIGIN"
  )

  # Executables 
  add_executable(Http-connection_ Http-connection/main.cpp)
  add_executable(Pillbox_         PillBox/main.cpp)
  add_executable(LightSwitch_     LightSwitch_Example/main.cpp)
  add_executable(Sound_           Sound/main.cpp)
  add_executable(Basket_          Basket/main.cpp)
  add_executable(File_            File_example/main.cpp)
  add_executable(Student_         Student/main.cpp)
  add_executable(Case2_           Inheritance/Case_2/main.cpp)
  add_executable(Case3_           Inheritance/Case_3/main.cpp)
  # add_executable(Case4_         Inheritance/Case_4/main.cpp)
  # add_executable(Case41_        Inheritance/Case_4/Cmain.cpp)

  add_executable(lightCT          TrafficLightController/main.cpp)

  add_executable(robot            Robot/main.cpp)
  add_executable(robotOP          Robot/OPmain.cpp)
  add_executable(robotmay         Robot/may.cpp)
  add_executable(robotth          Robot/threads.cpp)

  add_executable(control_system   Robot_System/ControlSystem.cpp)
  add_executable(sensor_system    Robot_System/SensorSystem.cpp)
  add_executable(vision_system    Robot_System/VisionSystem.cpp)
  add_executable(Robot_sys        Robot_System/Robot.cpp)

  add_executable(robot_single     Robot_System/main.cpp)

  #add_executable(broker_t         Beverage_Bottling_Plant/Broker/main.cpp)
 # add_executable(wt               Beverage_Bottling_Plant/Units/WTmain.cpp)
 # add_executable(Mixing           Beverage_Bottling_Plant/Units/Mixmain.cpp)
 # add_executable(ht               Beverage_Bottling_Plant/Units/Hmain.cpp)
 # add_executable(pt               Beverage_Bottling_Plant/Units/Pmain.cpp)
  #add_executable(filling_t        Beverage_Bottling_Plant/Units/Fillingmain.cpp)
 # add_executable(pact             Beverage_Bottling_Plant/Units/Pacmain.cpp)

  add_executable(CM_              CoffeeMachine/main.cpp)
  add_executable(buzzer           Buzzer/main.cpp)
  #add_executable(foodmixer        FoodMixer/main.cpp)

  # MQTT includes for plant units
  #target_include_directories(wt         PRIVATE ${CMAKE_SOURCE_DIR}/mqtt_cpp/include)
  #target_include_directories(Mixing     PRIVATE ${CMAKE_SOURCE_DIR}/mqtt_cpp/include)
  #target_include_directories(pt         PRIVATE ${CMAKE_SOURCE_DIR}/mqtt_cpp/include)
  #target_include_directories(filling_t  PRIVATE ${CMAKE_SOURCE_DIR}/mqtt_cpp/include)
  #target_include_directories(pact       PRIVATE ${CMAKE_SOURCE_DIR}/mqtt_cpp/include)

  # Inject Coconut plugin into desktop compile
  foreach(tgt IN ITEMS
    Http-connection_ Pillbox_ LightSwitch_ Sound_ Basket_ File_ Student_
    Case2_ Case3_ lightCT
    robot robotOP robotmay robotth
    control_system sensor_system vision_system Robot_sys
    robot_single CM_ buzzer 
    #foodmixer
  )
    target_compile_options(${tgt} PRIVATE -fplugin=$<TARGET_FILE:${PLUGIN_NAME}>)
  endforeach()

  # TLS + Boost::program_options for plant units
  #foreach(tgt IN ITEMS broker_t wt Mixing ht pt filling_t pact)
 #   target_compile_options(${tgt} PRIVATE -fplugin=$<TARGET_FILE:${PLUGIN_NAME}> -std=c++17 -DMQTT_USE_TLS)
   # target_link_libraries(${tgt} PRIVATE pthread OpenSSL::SSL OpenSSL::Crypto Boost::program_options)
 # endforeach()

  # Boost links for robot components
  target_link_libraries(control_system PRIVATE ${Boost_LIBRARIES})
  target_link_libraries(sensor_system  PRIVATE ${Boost_LIBRARIES})
  target_link_libraries(vision_system  PRIVATE ${Boost_LIBRARIES})
  target_link_libraries(Robot_sys      PRIVATE ${Boost_LIBRARIES})

  # OpenCV links
  target_link_libraries(vision_system PRIVATE ${OpenCV_LIBS})
  target_link_libraries(robot_single  PRIVATE ${OpenCV_LIBS})
endif()

# AVR BUILD (Coconut GCC plugin + HEX + Bound-T auto-run)

# Build the host-side annotator once, using the host g++
# (HOST_GXX is already found earlier in your script)


# List ONLY .h files here (no .cpp will be touched)
set(ANNOTATE_HEADERS
  "${CMAKE_SOURCE_DIR}/FoodMixer/FoodMixer.h"
 
   "${CMAKE_SOURCE_DIR}/FoodMixer/FoodMixer-arm.h"
     "${CMAKE_SOURCE_DIR}/Robot-Arm.h"
)

if(BUILD_AVR)
  if(NOT CMAKE_CXX_COMPILER MATCHES "avr-g\\+\\+$")
    message(FATAL_ERROR "BUILD_AVR=ON but CMAKE_CXX_COMPILER='${CMAKE_CXX_COMPILER}' is not avr-g++.")
  endif()
  if(NOT EXISTS "${AVR_APP}")
    message(FATAL_ERROR "Missing AVR source: ${AVR_APP}")
  endif()

  set(AVR_MCU  atmega328p   CACHE STRING "AVR MCU")
  set(AVR_F_CPU 16000000UL  CACHE STRING "F_CPU")
  add_definitions(-D__AVR__ -DF_CPU=${AVR_F_CPU})

  add_executable(foodmixer ${AVR_APP})
  set_target_properties(foodmixer PROPERTIES SUFFIX ".elf")

  target_compile_options(foodmixer PRIVATE
    -mmcu=${AVR_MCU} -Os -std=gnu++17
    -ffunction-sections -fdata-sections -fno-exceptions
    -Wall -Wextra -Wpedantic
  )
  target_link_options(foodmixer PRIVATE
    -mmcu=${AVR_MCU} -Wl,--gc-sections
  )

  # Build Coconut GCC plugin (host g++) and inject into avr-g++
  find_program(HOST_GXX NAMES g++-13 g++ REQUIRED)
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=plugin
    OUTPUT_VARIABLE AVR_PLUGIN_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message(STATUS "AVR GCC plugin dir: ${AVR_PLUGIN_DIR}")

  set(COCONUT_AVR_SO "${COCONUT_PLUGIN_DIR}/libCoconut-avr.so")
  add_custom_command(
    OUTPUT  "${COCONUT_AVR_SO}"
    COMMAND ${HOST_GXX} -shared -fPIC -O2 -std=c++17
            -I${AVR_PLUGIN_DIR}/include
            -I${AVR_PLUGIN_DIR}/include/gcc
            -I${AVR_PLUGIN_DIR}/include/gcc/cp
            -I${AVR_PLUGIN_DIR}/include/gcc/c-family
            -DCOCONUT_TARGET_AVR
            -o "${COCONUT_AVR_SO}"
            "${PLUGIN_SRC}"
    DEPENDS "${PLUGIN_SRC}"
    COMMENT "Building Coconut plugin for AVR GCC → ${COCONUT_AVR_SO}"
    VERBATIM
  )
  add_custom_target(CoconutPluginAVR ALL DEPENDS "${COCONUT_AVR_SO}")
  add_dependencies(foodmixer CoconutPluginAVR)
  target_compile_options(foodmixer PRIVATE -fplugin=${COCONUT_AVR_SO})

  # HEX artifact
  find_program(AVR_OBJCOPY avr-objcopy REQUIRED)
  add_custom_command(TARGET foodmixer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Generating Intel HEX for foodmixer"
    COMMAND ${AVR_OBJCOPY} -O ihex -R .eeprom
            $<TARGET_FILE:foodmixer>
            $<TARGET_FILE_DIR:foodmixer>/foodmixer.hex
    VERBATIM
  )

  # Auto-run Bound-T (prints tail in build log)
attach_boundt_post_build(foodmixer "${BOUNDT_AVR_EXECUTABLE}" "${BOUNDT_AVR_ARGS}")

  message(STATUS "AVR build → ${CMAKE_BINARY_DIR}/foodmixer.elf + .hex (MCU=${AVR_MCU})")
  set(BOUND_JSON_EXE "${CMAKE_SOURCE_DIR}/boundt_to_json_ms")
  add_custom_command(
    OUTPUT "${BOUND_JSON_EXE}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}"
    COMMAND ${HOST_GXX} -O2 -std=c++17
            "${CMAKE_SOURCE_DIR}/boundt_to_json_ms.cpp"
            -o "${BOUND_JSON_EXE}"
    DEPENDS "${CMAKE_SOURCE_DIR}/boundt_to_json_ms.cpp"
    COMMENT "Building host tool: ${BOUND_JSON_EXE}"
    VERBATIM
  )
  add_custom_target(boundt_to_json_ms_build ALL DEPENDS "${BOUND_JSON_EXE}")

   set(ARM7_F_CPU_HZ 16000000 CACHE STRING "AVR CPU clock in Hz")

  add_custom_command(TARGET foodmixer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Converting Bound-T (AVR) → JSON"
    COMMAND "${BOUND_JSON_EXE}" "${CMAKE_BINARY_DIR}/boundt/report.txt" "${ARM7_F_CPU_HZ}"
            "${CMAKE_BINARY_DIR}/boundt/wcet_avr.json"
    DEPENDS boundt_to_json_ms_build
    VERBATIM
  )

  add_dependencies(foodmixer annotate_wcet_build)
  set(ANNOTATOR_EXE "${CMAKE_BINARY_DIR}/annotate_wcet")
add_custom_command(
  OUTPUT "${ANNOTATOR_EXE}"
  COMMAND ${HOST_GXX} -O2 -std=c++17
          -I"${CMAKE_SOURCE_DIR}/external"
          "${CMAKE_SOURCE_DIR}/annotate_wcet_inplace.cpp"
          -o "${ANNOTATOR_EXE}"
  DEPENDS "${CMAKE_SOURCE_DIR}/annotate_wcet_inplace.cpp"
  COMMENT "Building host tool: ${ANNOTATOR_EXE}"
  VERBATIM
)
add_custom_target(annotate_wcet_build ALL DEPENDS "${ANNOTATOR_EXE}")
add_custom_command(TARGET foodmixer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E echo "Annotating FoodMixer.h with WCET (AVR)"
  COMMAND "${ANNOTATOR_EXE}" "${CMAKE_BINARY_DIR}/boundt/wcet_avr.json"
          "${CMAKE_SOURCE_DIR}/FoodMixer/FoodMixer.h"
  VERBATIM
)
endif()



# ================================================================
# ARM7TDMI BUILD (ARM state) + Coconut GCC plugin + BIN + Bound-T
# ================================================================
if(BUILD_ARM7)
  enable_language(ASM)

  # ----------------------------------------------------------------
  #  Toolchain sanity check
  # ----------------------------------------------------------------
  if(NOT CMAKE_C_COMPILER MATCHES "arm-none-eabi-.*gcc")
    message(FATAL_ERROR
      "BUILD_ARM7=ON but C compiler is '${CMAKE_C_COMPILER}'.\n"
      "Re-configure with: -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm7.cmake")
  endif()

  # ----------------------------------------------------------------
  #  FoodMixer inputs
  # ----------------------------------------------------------------
  foreach(f IN ITEMS "${ARM7_APP}" "${ARM7_STARTUP}" "${ARM7_LINKER_SCRIPT}")
    if(NOT EXISTS "${f}")
      message(FATAL_ERROR "Missing ARM7 file: ${f}")
    endif()
  endforeach()

  # ----------------------------------------------------------------
  #  Robot inputs
  # ----------------------------------------------------------------
  set(ROBOT_ARM7_APP           "${CMAKE_SOURCE_DIR}/Robot_System/Robot-Arm.cpp")
  set(ROBOT_ARM7_STARTUP       "${ARM7_STARTUP}")
  set(ROBOT_ARM7_LINKER_SCRIPT "${ARM7_LINKER_SCRIPT}")

  foreach(f IN ITEMS "${ROBOT_ARM7_APP}" "${ROBOT_ARM7_STARTUP}" "${ROBOT_ARM7_LINKER_SCRIPT}")
    if(NOT EXISTS "${f}")
      message(FATAL_ERROR "Missing Robot ARM7 file: ${f}")
    endif()
  endforeach()

  # Robot is header-only (RobotSingle.h has all definitions)
  set(ROBOT_SOURCES
    ${ROBOT_ARM7_APP}
    ${ROBOT_ARM7_STARTUP}
  )

  # ----------------------------------------------------------------
  #  TrafficLight inputs
  # ----------------------------------------------------------------
  set(TRAFFICLIGHT_ARM7_APP           "${CMAKE_SOURCE_DIR}/TrafficLightController/main.cpp")
  set(TRAFFICLIGHT_ARM7_STARTUP       "${ARM7_STARTUP}")
  set(TRAFFICLIGHT_ARM7_LINKER_SCRIPT "${ARM7_LINKER_SCRIPT}")

  foreach(f IN ITEMS "${TRAFFICLIGHT_ARM7_APP}" "${TRAFFICLIGHT_ARM7_STARTUP}" "${TRAFFICLIGHT_ARM7_LINKER_SCRIPT}")
    if(NOT EXISTS "${f}")
      message(FATAL_ERROR "Missing TrafficLight ARM7 file: ${f}")
    endif()
  endforeach()

  set(TRAFFICLIGHT_SOURCES
    ${TRAFFICLIGHT_ARM7_APP}
    ${TRAFFICLIGHT_ARM7_STARTUP}
  )

  # ----------------------------------------------------------------
  #  Common ARM7 flags
  # ----------------------------------------------------------------
  set(ARM7_CPU "arm7tdmi" CACHE STRING "ARM7 CPU (e.g. arm7tdmi)")
  option(ARM7_USE_THUMB "Compile/link in Thumb mode" OFF)

  set(ARM7_MCU_FLAGS -mcpu=${ARM7_CPU})
  if(ARM7_USE_THUMB)
    list(APPEND ARM7_MCU_FLAGS -mthumb)
  endif()

  set(ARM7_C_FLAGS
    -ffreestanding -Os
    -ffunction-sections -fdata-sections
    -Wall -Wextra -Wpedantic
    -D__ARM7__
  )
  set(ARM7_CXX_FLAGS
    -ffreestanding -Os
    -ffunction-sections -fdata-sections
    -fno-exceptions -fno-rtti -fno-use-cxa-atexit
    -Wall -Wextra -Wpedantic
    -std=gnu++17
    -D__ARM7__
  )
  set(ARM7_LINK_FLAGS
    -Wl,--gc-sections
    --specs=nosys.specs
    --specs=nano.specs
  )

  # ----------------------------------------------------------------
  #  Copy linker script into build dir
  # ----------------------------------------------------------------
  add_custom_command(
    OUTPUT "${CMAKE_BINARY_DIR}/arm7.ld"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${ARM7_LINKER_SCRIPT}" "${CMAKE_BINARY_DIR}/arm7.ld"
    DEPENDS "${ARM7_LINKER_SCRIPT}"
    COMMENT "Copying ARM7 linker script -> ${CMAKE_BINARY_DIR}/arm7.ld"
    VERBATIM
  )
  add_custom_target(copy_arm7_ld ALL DEPENDS "${CMAKE_BINARY_DIR}/arm7.ld")

  # ================================================================
  #  Coconut GCC plugin 
  # ================================================================
  find_program(HOST_GXX NAMES g++-13 g++ REQUIRED)
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=plugin
    OUTPUT_VARIABLE ARM_PLUGIN_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message(STATUS "ARM GCC plugin dir: ${ARM_PLUGIN_DIR}")

  set(COCONUT_ARM_SO "${COCONUT_PLUGIN_DIR}/libCoconut-arm7.so")
  add_custom_command(
    OUTPUT  "${COCONUT_ARM_SO}"
    COMMAND ${HOST_GXX} -shared -fPIC -O2 -std=c++17
            -I${ARM_PLUGIN_DIR}/include
            -I${ARM_PLUGIN_DIR}/include/gcc
            -I${ARM_PLUGIN_DIR}/include/gcc/cp
            -I${ARM_PLUGIN_DIR}/include/gcc/c-family
            -DCOCONUT_TARGET_ARM7
            -o "${COCONUT_ARM_SO}"
            "${PLUGIN_SRC}"
    DEPENDS "${PLUGIN_SRC}"
    COMMENT "Building Coconut plugin for ARM GCC → ${COCONUT_ARM_SO}"
    VERBATIM
  )
  add_custom_target(CoconutPluginARM ALL DEPENDS "${COCONUT_ARM_SO}")

  # ================================================================
  #  Common tools: objcopy, Bound-T → JSON, annotator
  # ================================================================
  find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)

  set(BOUNDT_ARM7_EXECUTABLE "/mnt/c/Users/Arwa Alsubhi/Desktop/Coconut-Project/V2/boundt_arm7/boundt_arm7"
      CACHE FILEPATH "Path to boundt_arm7")
  set(BOUNDT_ARM7_ARGS "-device arm7 -loop full -time -table -draw total"
      CACHE STRING "Args for boundt_arm7")

  set(BOUND_JSON_EXE "${CMAKE_SOURCE_DIR}/boundt_to_json_ms")
  add_custom_command(
    OUTPUT "${BOUND_JSON_EXE}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}"
    COMMAND ${HOST_GXX} -O2 -std=c++17
            "${CMAKE_SOURCE_DIR}/boundt_to_json_ms.cpp"
            -o "${BOUND_JSON_EXE}"
    DEPENDS "${CMAKE_SOURCE_DIR}/boundt_to_json_ms.cpp"
    COMMENT "Building host tool: ${BOUND_JSON_EXE}"
    VERBATIM
  )
  add_custom_target(boundt_to_json_ms_build ALL DEPENDS "${BOUND_JSON_EXE}")

  set(ANNOTATOR_EXE "${CMAKE_BINARY_DIR}/annotate_wcet")
  add_custom_command(
    OUTPUT "${ANNOTATOR_EXE}"
    COMMAND ${HOST_GXX} -O2 -std=c++17
            -I"${CMAKE_SOURCE_DIR}/external"
            "${CMAKE_SOURCE_DIR}/annotate_wcet_inplace.cpp"
            -o "${ANNOTATOR_EXE}"
    DEPENDS "${CMAKE_SOURCE_DIR}/annotate_wcet_inplace.cpp"
    COMMENT "Building host tool: ${ANNOTATOR_EXE}"
    VERBATIM
  )
  add_custom_target(annotate_wcet_build ALL DEPENDS "${ANNOTATOR_EXE}")

  set(ARM7_F_CPU_HZ 16000000 CACHE STRING "ARM7 CPU clock in Hz")

 
  
  function(add_arm7_example NAME SOURCES ROOTS_LIST BOUNDT_SUBDIR ANNOTATE_FILE)
    # NAME          : logical name, e.g. "foodmixer" or "robot"
    # SOURCES       : source files for firmware
    # ROOTS_LIST    : CMake list of Bound-T root functions
    # BOUNDT_SUBDIR : subdir under CMAKE_BINARY_DIR for Bound-T output
    # ANNOTATE_FILE : file to annotate with WCET

    set(TGT_MAIN    "${NAME}_arm7.elf")
    set(TGT_BOUNDT  "${NAME}_arm7.boundt.elf")
    set(BOUNDT_DIR  "${CMAKE_BINARY_DIR}/${BOUNDT_SUBDIR}")

    # Make a space-separated string for /bin/sh
    string(REPLACE ";" " " ROOTS_STR "${ROOTS_LIST}")

    #  Main firmware (with Coconut) 
    add_executable(${TGT_MAIN}
      ${SOURCES}
    )
    add_dependencies(${TGT_MAIN} copy_arm7_ld CoconutPluginARM annotate_wcet_build)

    target_compile_definitions(${TGT_MAIN} PRIVATE
      TARGET_LPC214x
      F_CPU=16000000UL
      PCLK_HZ=16000000UL
    )

    target_compile_options(${TGT_MAIN} PRIVATE
      $<$<COMPILE_LANGUAGE:C>:${ARM7_MCU_FLAGS} ${ARM7_C_FLAGS}>
      $<$<COMPILE_LANGUAGE:CXX>:${ARM7_MCU_FLAGS} ${ARM7_CXX_FLAGS}>
      $<$<COMPILE_LANGUAGE:ASM>:${ARM7_MCU_FLAGS}>
      -fplugin=${COCONUT_ARM_SO}
    )

    target_link_options(${TGT_MAIN} PRIVATE
      ${ARM7_MCU_FLAGS}
      ${ARM7_LINK_FLAGS}
      -T "${CMAKE_BINARY_DIR}/arm7.ld"
      -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${NAME}_arm7.map
    )

    #  Slim Bound-T ELF 
    add_executable(${TGT_BOUNDT}
      ${SOURCES}
    )
    add_dependencies(${TGT_BOUNDT} copy_arm7_ld)

    target_compile_definitions(${TGT_BOUNDT} PRIVATE
      TARGET_LPC214x
      F_CPU=16000000UL
      PCLK_HZ=16000000UL
    )

    target_compile_options(${TGT_BOUNDT} PRIVATE
      $<$<COMPILE_LANGUAGE:C>:${ARM7_MCU_FLAGS} ${ARM7_C_FLAGS}>
      $<$<COMPILE_LANGUAGE:CXX>:${ARM7_MCU_FLAGS} ${ARM7_CXX_FLAGS}>
      $<$<COMPILE_LANGUAGE:ASM>:${ARM7_MCU_FLAGS}>
    )

    target_link_options(${TGT_BOUNDT} PRIVATE
      ${ARM7_MCU_FLAGS}
      ${ARM7_LINK_FLAGS}
      -T "${CMAKE_BINARY_DIR}/arm7.ld"
      -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${NAME}_arm7.boundt.map
      -Wl,--wrap=printf
      -Wl,--wrap=puts
      -Wl,--wrap=putchar
    )

    # Main firmware waits for Bound-T ELF
    add_dependencies(${TGT_MAIN} ${TGT_BOUNDT})

    #  .bin from main ELF 
    add_custom_command(TARGET ${TGT_MAIN} POST_BUILD
      COMMAND ${CMAKE_OBJCOPY} -O binary
              $<TARGET_FILE:${TGT_MAIN}>
              $<TARGET_FILE_DIR:${TGT_MAIN}>/${NAME}_arm7.bin
      COMMENT "Generating raw binary ${NAME}_arm7.bin"
      VERBATIM
    )

    #  Bound-T analysis 
    if(NOT EXISTS "${BOUNDT_ARM7_EXECUTABLE}")
      message(WARNING "Bound-T not found: ${BOUNDT_ARM7_EXECUTABLE} → skipping ARM7 WCET run for ${NAME}")
    else()
      add_custom_command(TARGET ${TGT_BOUNDT} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "==> Bound-T(ARM7): analyzing ${NAME} roots on slim ELF"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${BOUNDT_DIR}"

        COMMAND ${CMAKE_COMMAND} -E echo "CMD: \"${BOUNDT_ARM7_EXECUTABLE}\" ${BOUNDT_ARM7_ARGS} -dot \"${BOUNDT_DIR}/graphs.dot\" \"$<TARGET_FILE:${TGT_BOUNDT}>\" ${ROOTS_STR}"

        COMMAND /bin/sh -lc "\"${BOUNDT_ARM7_EXECUTABLE}\" ${BOUNDT_ARM7_ARGS} \
          -dot '${BOUNDT_DIR}/graphs.dot' \
          '$<TARGET_FILE:${TGT_BOUNDT}>' \
          ${ROOTS_STR} \
          > '${BOUNDT_DIR}/report.txt' 2>&1 || true"

        BYPRODUCTS "${BOUNDT_DIR}/report.txt" "${BOUNDT_DIR}/graphs.dot"
        VERBATIM
      )
    endif()

    #  Bound-T → JSON 
    set(JSON_FILE "${BOUNDT_DIR}/wcet_${NAME}_arm7.json")

    add_custom_command(TARGET ${TGT_MAIN} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E echo "Converting Bound-T (ARM7) → JSON (${NAME})"
      COMMAND "${BOUND_JSON_EXE}" "${BOUNDT_DIR}/report.txt" "${ARM7_F_CPU_HZ}" "${JSON_FILE}"
      DEPENDS boundt_to_json_ms_build ${TGT_BOUNDT}
      VERBATIM
    )

    #  Annotate source/header 
    add_custom_command(TARGET ${TGT_MAIN} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E echo "Annotating ${ANNOTATE_FILE} with WCET (ARM, ${NAME})"
      COMMAND "${ANNOTATOR_EXE}" "${JSON_FILE}" "${ANNOTATE_FILE}"
      VERBATIM
    )

    message(STATUS "ARM7 ${NAME} build → ${CMAKE_BINARY_DIR}/${NAME}_arm7.elf + .bin")
  endfunction()

  
  # FoodMixer roots – main (from cache)
  set(BOUNDT_ARM7_ROOT_FOODMIXER "main"
      CACHE STRING "Root function(s) for FoodMixer WCET (Bound-T)")
  separate_arguments(BOUNDT_ARM7_ROOT_FOODMIXER_LIST NATIVE_COMMAND "${BOUNDT_ARM7_ROOT_FOODMIXER}")

  # Robot roots – mangled names
  set(BOUNDT_ARM7_ROOT_ROBOT_LIST
    _ZN5Robot15Read_force_dataEv
    _ZN5Robot15Read_path_angleEv
    _ZN5Robot16ComputePathAngleEv
    _ZN5Robot15AdjustActuatorsEi
    _ZN5Robot23Compute_set_points_slowEf
    _ZN5Robot23Compute_set_points_fastEf
    _ZN5Robot3EndEv
  )
  

  # TrafficLight roots – default main (override via cache if needed)
  set(BOUNDT_ARM7_ROOT_TRAFFICLIGHT "_ZN12TrafficLight13SwitchToAmberEv, _ZN12TrafficLight14SwitchToGreenEv, _ZN12TrafficLight11SwitchToRedEv"
      CACHE STRING "Root function(s) for TrafficLight WCET (Bound-T)")
  separate_arguments(BOUNDT_ARM7_ROOT_TRAFFICLIGHT_LIST NATIVE_COMMAND "${BOUNDT_ARM7_ROOT_TRAFFICLIGHT}")

  # ================================================================
  #  Instantiate pipeline: FoodMixer + Robot + TrafficLight
  # ================================================================
  add_arm7_example(
    foodmixer
    "${ARM7_APP};${ARM7_STARTUP}"
    "${BOUNDT_ARM7_ROOT_FOODMIXER_LIST}"
    "boundt"
    "${CMAKE_SOURCE_DIR}/FoodMixer/FoodMixer-arm.h"
  )

  add_arm7_example(
    robot
    "${ROBOT_SOURCES}"
    "${BOUNDT_ARM7_ROOT_ROBOT_LIST}"
    "boundt_robot"
    "${CMAKE_SOURCE_DIR}/Robot_System/Robot-Arm.h"
  )

  add_arm7_example(
    trafficlight
    "${TRAFFICLIGHT_SOURCES}"
    "${BOUNDT_ARM7_ROOT_TRAFFICLIGHT_LIST}"
    "boundt_trafficlight"
    "${CMAKE_SOURCE_DIR}/TrafficLightController/TrafficLightController.h"
  )

  message(STATUS "Using ARM7 linker script: ${ARM7_LINKER_SCRIPT} (copied to ${CMAKE_BINARY_DIR}/arm7.ld)")
endif()



# Summary

message(STATUS "Build mode → DESKTOP=${BUILD_DESKTOP}  AVR=${BUILD_AVR}  ARM7=${BUILD_ARM7}")

