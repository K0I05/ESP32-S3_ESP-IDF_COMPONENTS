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
# i.e. C:\Users\user\.platformio\packages\framework-espidf\tools\cmake
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

idf_component_register(
    SRCS "cla_common.c" "cla_filter.c" "cla_vector.c" "cla_matrix.c" "cla.c"
    INCLUDE_DIRS "include"
)
