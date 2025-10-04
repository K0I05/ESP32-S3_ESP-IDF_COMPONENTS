#
# Versioning Information for ESP-IDF Components with GitHub, GitVersion and CMake
#
# Inspired by: https://www.esp32.com/viewtopic.php?f=2&t=45054&p=146150#p146150
#
# Install Git-Version via command prompt: dotnet tool install --global GitVersion.Tool
# Create a GitVersion.yml file in the root of your project with the following content:
#
#   major-version-bump-message: '\+semver:\s?(breaking|major)'
#   minor-version-bump-message: '\+semver:\s?(feature|minor)'
#   patch-version-bump-message: '\+semver:\s?(fix|patch)'
#   commit-message-incrementing: Enabled
#
# Download CMake JSON-Parser: https://github.com/sbellus/json-cmake/blob/master/JSONParser.cmake
# Copy the CMake JSONParser.cmake file to the tools/cmake directory of your ESP-IDF installation.
# i.e. PIO Env: C:\Users\user\.platformio\packages\framework-espidf\tools\cmake
# i.e. ESP Env: C:\Users\lavco\esp\v5.4\esp-idf\tools\cmake
#
include( $ENV{IDF_PATH}/tools/cmake/version.cmake )

# validate JSONParser library, version.h.in, pio_lib_sync.py, esp_cmp_sync.py, 
# library.json.in, and idf_component.yml.in files are available for preprocessing
if( EXISTS "$ENV{IDF_PATH}/tools/cmake/JSONParser.cmake"
    AND EXISTS "${CMAKE_SOURCE_DIR}/templates/component/include/version.h.in" 
    AND EXISTS "${CMAKE_SOURCE_DIR}/templates/components/${COMPONENT_NAME}/library.json.in" 
    AND EXISTS "${CMAKE_SOURCE_DIR}/templates/components/${COMPONENT_NAME}/idf_component.yml.in")

    include( $ENV{IDF_PATH}/tools/cmake/JSONParser.cmake )

    # Get latest versioning information from git repository with GitVersion
    execute_process(
        COMMAND dotnet-gitversion
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_VERSION_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Instantiate json variable
    sbeParseJson( GIT_VERSION_JSON GIT_VERSION_OUTPUT )

    # Parse versioning variables from json output
    set( GIT_VERSION_DATE  ${GIT_VERSION_JSON.CommitDate} )
    set( GIT_SEM_VERSION   ${GIT_VERSION_JSON.MajorMinorPatch} )
    set( GIT_VERSION_MAJOR ${GIT_VERSION_JSON.Major} )
    set( GIT_VERSION_MINOR ${GIT_VERSION_JSON.Minor} )
    set( GIT_VERSION_PATCH ${GIT_VERSION_JSON.Patch} )
    set( GIT_FULL_SEM_VER  ${GIT_VERSION_JSON.FullSemVer} )
    set( GIT_SHORT_SHA     ${GIT_VERSION_JSON.ShortSha} )

    # Release json variable
    sbeClearJson( GIT_VERSION_JSON )

    # Components should be named as "esp_<component_name>"
    string( FIND "${COMPONENT_NAME}" "esp_" ESP_PREFIX )

    # Check if the component name starts with "esp_"
    if(ESP_PREFIX EQUAL -1)
        # Use the component name as is
        string( CONCAT COMPONENT_HEADER_NAME "" "${COMPONENT_NAME}" )
    else()
        # Parse component file name from component name
        string( REPLACE "esp_" "" COMPONENT_HEADER_NAME "${COMPONENT_NAME}" )
    endif()

    # Set the component header name to upper case
    string( TOUPPER "${COMPONENT_HEADER_NAME}" COMPONENT_HEADER_NAME_UPPER )


    # REMOVE TEMPLATE GENERATED FILES FROM COMPONENT DIRECTORY (FORCED REGENERATION)

    # Remove C header versioning file from component directory
    file( REMOVE "${COMPONENT_DIR}/include/${COMPONENT_HEADER_NAME}_version.h" )

    # Remove json library file from component directory
    file( REMOVE "${COMPONENT_DIR}/library.json" )

    # Remove yml idf component file from component directory
    file( REMOVE "${COMPONENT_DIR}/idf_component.yml" )


    # GENERATE FILES FROM TEMPLATES FOR COMPONENT DIRECTORY

    # Generate C header file from template with versioning information
    configure_file( "${CMAKE_SOURCE_DIR}/templates/component/include/version.h.in" "${COMPONENT_DIR}/include/${COMPONENT_HEADER_NAME}_version.h" @ONLY )

    # Generate json library file from template with versioning information
    configure_file( "${CMAKE_SOURCE_DIR}/templates/components/${COMPONENT_NAME}/library.json.in" "${COMPONENT_DIR}/library.json" @ONLY )

    # Generate yml idf component file from template with versioning information
    configure_file( "${CMAKE_SOURCE_DIR}/templates/components/${COMPONENT_NAME}/idf_component.yml.in" "${COMPONENT_DIR}/idf_component.yml" @ONLY )
endif()


# Register the component
idf_component_register(
    SRCS heart_rate.c
    INCLUDE_DIRS include
)

# Get global variables from idf build property
idf_build_get_property( cmp_reg_sync COMPONENT_REGISTRY_SYNC )
idf_build_get_property( cmp_reg_o COMPONENT_REGISTRY_OWNER )

# Validate if the component should be synchronized with esp-idf and platformio registries
string( COMPARE EQUAL "${cmp_reg_sync}" "ENABLED" STR_CMP_RESULT )

# Register the component with the ESP-IDF components and platformio registries
#if(STR_CMP_RESULT EQUAL 1)


    #add_custom_target(
    #    my_component_sync ALL
    #    COMMAND python ${CMAKE_SOURCE_DIR}/staging/esp_cmp_sync.py OK
    #    DEPENDS ${CMAKE_SOURCE_DIR}/.pio/build/esp32s3box/esp-idf/esp_uuid/libesp_uuid.a
    #    COMMENT "python testing with add_custom_command..."
    #)
    #add_dependencies( ${CMAKE_SOURCE_DIR}/.pio/build/esp32s3box/esp-idf/esp_uuid/libesp_uuid.a my_component_sync )

# .pio\build\esp32s3box\components\peripherals\owb\esp_ds18b20\ds18b20.c.o
# .pio/build/esp32s3box/esp-idf/esp_hdc1080/libesp_hdc1080.a

    #add_custom_command(
    #    TARGET ${CMAKE_SOURCE_DIR}/.pio/build/esp32s3box/components/utilities/esp_uuid/uuid.c.o
    #    POST_BUILD
    #    COMMAND python ${CMAKE_SOURCE_DIR}/stg/pio_lib_sync.py OK
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #    COMMENT "python testing with add_custom_command..."
    #)

    # Package the component and upload it to the ESP-IDF component registry
    #add_custom_command(
    #    TARGET ${COMPONENT_LIB}
    #    POST_BUILD
    #    COMMAND powershell compote component register --name ${COMPONENT_NAME} --project-dir '${COMPONENT_DIR}' --dest-dir '${CMAKE_SOURCE_DIR}/log'
    #    COMMENT "Registering component ${COMPONENT_NAME} with ESP-IDF component registry..."
    #)

    #add_custom_target(
    #    my
    #    COMMAND powershell compote component register --name ${COMPONENT_NAME} --project-dir '${COMPONENT_DIR}' --dest-dir '${CMAKE_SOURCE_DIR}/log'
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #    COMMENT "Registering component ${COMPONENT_NAME} with ESP-IDF component registry..."
    #)
    #add_dependencies( ${COMPONENT_LIB} cmp_reg_sync )
    #add_dependencies(${PROJECT_NAME}.elf cmp_reg_sync)


    # Package the component and upload it to the ESP-IDF component registry
    #if(NOT CMAKE_BUILD_EARLY_EXPANSION) 
    #execute_process(
    #    COMMAND powershell compote component upload --name "${COMPONENT_NAME}" --project-dir '"${COMPONENT_DIR}"' --dest-dir '"${CMAKE_SOURCE_DIR}/log"'
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #    OUTPUT_FILE "${CMAKE_SOURCE_DIR}/log/esp_cmp_sync.log"
    #)
    #endif()
    # Package the components and upload it to the platformio library registry
    #execute_process(
    #    COMMAND powershell pio pkg publish '"${COMPONENT_DIR}"' --owner "${cmp_reg_o}" --type library --no-interactive
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #    OUTPUT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/log/pio_lib_sync.log"
    #)
#endif()
