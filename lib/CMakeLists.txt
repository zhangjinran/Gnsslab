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

        ARLambda.hpp
        ConfigData.h
        ConfigReader.h
        Const.h
        CoordConvert.h
        CoordStruct.h
        CSDetector.h
        DataExporter.h
        EphStore.h
        Exception.h
        GnssFunc.h
        GnssStruct.h
        KalmanFilter.hpp
        MathUtils.hpp
        NavEphBase.hpp
        NavEphBDS.hpp
        NavEphGalileo.hpp
        NavEphGLONASS.hpp
        NavEphGPS.hpp
        NavEphIRNSS.hpp
        NavEphQZSS.hpp
        OrbitExporter.h
        QualityAnalyzer.h
        RinexNavStore.hpp
        RinexObsReader.h
        Rx3ClockReader.hpp
        SolverKalman.h
        SolverLSQ.h
        SP3Store.hpp
        SPPCode.h
        SPPGFCode.h
        SPPIFCode.h
        SPPUCCodePhase.h
        StringUtils.h
        Time2020Convert.h
        TimeConvert.h
        TimeStruct.h
        write.h
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
