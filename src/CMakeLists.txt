###################################################################
#
# gnssLab core library
#
###################################################################

add_library(gnss SHARED)

# Explicit manifest: every translation unit compiled into gnss, plus headers
# shown in generated IDE projects.  Keep this list authoritative.
target_sources(
    gnss
    PRIVATE
        ARLambda.cpp
        ConfigReader.cpp
        CSDetector.cpp
        DataExporter.cpp
        GnssFunc.cpp
        GnssStruct.cpp
        KalmanFilter.cpp
        NavEphBase.cpp
        NavEphBDS.cpp
        NavEphGalileo.cpp
        NavEphGLONASS.cpp
        NavEphGPS.cpp
        NavEphIRNSS.cpp
        NavEphQZSS.cpp
        NavEphRegistry.cpp
        OrbitExporter.cpp
        QualityAnalyzer.cpp
        RinexNavStore.cpp
        RinexObsReader.cpp
        Rx3ClockReader.cpp
        SolverKalman.cpp
        SolverLSQ.cpp
        SP3Store.cpp
        SPPCode.cpp
        SPPGFCode.cpp
        SPPIFCode.cpp
        SPPUCCodePhase.cpp
        Time2020Convert.cpp
        TimeConvert.cpp
        TimeStruct.cpp
        write.cpp

        ${PROJECT_SOURCE_DIR}/include/gnsslab/ARLambda.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/ConfigReader.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/Const.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/CoordConvert.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/CoordStruct.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/CSDetector.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/DataExporter.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/EphStore.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/Exception.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/GnssFunc.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/GnssStruct.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/KalmanFilter.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/NavEphBase.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/NavEphBDS.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/NavEphGalileo.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/NavEphGLONASS.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/NavEphGPS.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/NavEphIRNSS.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/NavEphQZSS.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/OrbitExporter.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/QualityAnalyzer.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/RinexNavStore.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/RinexObsReader.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/Rx3ClockReader.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/SolverKalman.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/SolverLSQ.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/SP3Store.hpp
        ${PROJECT_SOURCE_DIR}/include/gnsslab/SPPCode.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/SPPGFCode.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/SPPIFCode.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/SPPUCCodePhase.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/StringUtils.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/Time2020Convert.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/TimeConvert.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/TimeStruct.h
        ${PROJECT_SOURCE_DIR}/include/gnsslab/write.h
        internal/MathUtils.hpp
)

# Public because gnssLab headers and all consumers need these include paths.
target_include_directories(
    gnss
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.4.0
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src
)

if(MSVC)
    set_target_properties(
        gnss
        PROPERTIES
            WINDOWS_EXPORT_ALL_SYMBOLS ON
    )
endif()
