###################################################################
#
# gnssLab core library
#
###################################################################

# Preserve the current source discovery strategy for Stage 3A-1.
aux_source_directory(
    ${PROJECT_SOURCE_DIR}/lib
    DIR_LIB_SRCS
)

add_library(
    gnss
    SHARED

    ${DIR_LIB_SRCS}

    ${PROJECT_SOURCE_DIR}/lib/Time2020Convert.h
    ${PROJECT_SOURCE_DIR}/lib/Time2020Convert.cpp
    ${PROJECT_SOURCE_DIR}/lib/write.h
    ${PROJECT_SOURCE_DIR}/lib/write.cpp
    ${PROJECT_SOURCE_DIR}/lib/SPPCode.h
    ${PROJECT_SOURCE_DIR}/lib/SPPCode.cpp
    ${PROJECT_SOURCE_DIR}/lib/SPPGFCode.cpp
    ${PROJECT_SOURCE_DIR}/lib/SPPGFCode.h
    ${PROJECT_SOURCE_DIR}/lib/NavEphGLONASS.hpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphGLONASS.cpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphBase.hpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphBase.cpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphRegistry.cpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphGalileo.hpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphGalileo.cpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphQZSS.hpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphQZSS.cpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphIRNSS.hpp
    ${PROJECT_SOURCE_DIR}/lib/NavEphIRNSS.cpp
    ${PROJECT_SOURCE_DIR}/lib/DataExporter.h
    ${PROJECT_SOURCE_DIR}/lib/DataExporter.cpp
    ${PROJECT_SOURCE_DIR}/lib/OrbitExporter.h
    ${PROJECT_SOURCE_DIR}/lib/OrbitExporter.cpp
    ${PROJECT_SOURCE_DIR}/lib/QualityAnalyzer.h
    ${PROJECT_SOURCE_DIR}/lib/QualityAnalyzer.cpp
)

# Public because gnssLab headers and all consumers need these include paths.
target_include_directories(
    gnss
    PUBLIC
        ${PROJECT_SOURCE_DIR}/lib
        ${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.4.0
)

if(MSVC)
    set_target_properties(
        gnss
        PROPERTIES
            WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endif()
