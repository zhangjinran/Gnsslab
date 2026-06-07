#pragma once

#include <string>
#include <vector>
#include "TimeStruct.h"
#include "CoordStruct.h"
#include "CoordConvert.h"

namespace gnss {
    struct Satellite;

    class DataExporter {
public:
    static std::string getOutputBasePath();
    
    static bool exportTimeConversionData(const CommonTime& gpsTime, 
                                        const std::string& subFolder = "");
    
    static bool exportTimeSystemParams(const std::string& subFolder = "");
    
    static bool exportTimeConversionErrors(const std::string& subFolder = "");
    
    static bool exportCoordConversionData(const XYZ& xyz,
                                         const std::vector<ReferenceFrame*>& frames,
                                         const std::string& subFolder = "");
    
    static bool exportEllipsoidParams(const std::string& subFolder = "");
    
    static bool exportCoordConversionErrors(const XYZ& testPoint,
                                           const std::string& subFolder = "");
    
    static bool exportFrameDifferenceMatrix(const XYZ& testPoint,
                                           const std::string& subFolder = "");
    
    static bool exportSatelliteSkyplotData(const std::vector<gnss::Satellite>& satellites,
                                          const std::string& subFolder = "");
    
    static bool exportENUConversionData(const XYZ& targetXYZ,
                                       const XYZ& refXYZ,
                                       const std::vector<ReferenceFrame*>& frames,
                                       const std::string& subFolder = "");

private:
    static bool ensureDirectoryExists(const std::string& path);
    
    static std::string getTimeSystemOutputPath(const std::string& subFolder = "");
    
    static std::string getCoordSystemOutputPath(const std::string& subFolder = "");
    
    static std::string getFigureOutputPath(const std::string& subFolder = "");
};

struct Satellite {
    double azimuth;
    double elevation;
    std::string id;
    std::string system;
};

} // namespace gnss