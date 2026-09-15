#pragma once

#include <string>
#include <vector>
#include <map>
#include <gnsslab/GnssStruct.h>
#include <gnsslab/TimeStruct.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/SP3Store.hpp>

namespace gnss {

struct OrbitPoint {
    CommonTime time;
    SatID sat;
    double x;
    double y;
    double z;
    double radius;
    double lat;
    double lon;
    double svURA;
};

typedef std::map<SatID, std::vector<OrbitPoint>> OrbitDataMap;

class OrbitExporter {
public:
    static std::string getOutputBasePath();
    
    static bool exportOrbitData(const OrbitDataMap& orbitData, 
                               const std::string& subFolder = "");
    
    static bool exportSingleSatOrbit(const std::vector<OrbitPoint>& orbitPoints,
                                     const SatID& sat,
                                     const std::string& subFolder = "");
    
    static bool exportRadiusAnalysis(const OrbitDataMap& orbitData,
                                     const std::string& subFolder = "");
    
    static bool exportVelocityAnalysis(const OrbitDataMap& orbitData,
                                       const std::string& subFolder = "");
                                       
    static bool exportVelocityAnalysis(const OrbitDataMap& navOrbitData,
                                       SP3Store* sp3Store,
                                       const std::string& subFolder = "");
    
    static bool exportGroundTrack(const OrbitDataMap& orbitData,
                                  const std::string& subFolder = "");
    
    static std::vector<OrbitPoint> computeOrbitPoints(RinexNavStore& navStore,
                                                      const SatID& sat,
                                                      const CommonTime& startTime,
                                                      double durationHours = 24.0,
                                                      double intervalSeconds = 300.0);
    
    static std::vector<OrbitPoint> computeSP3OrbitPoints(SP3Store& sp3Store,
                                                         const SatID& sat,
                                                         const CommonTime& startTime,
                                                         double durationHours = 24.0,
                                                         double intervalSeconds = 300.0);
    
    static bool exportCombinedOrbitData(const OrbitDataMap& navOrbitData,
                                        SP3Store* sp3Store,
                                        const std::string& subFolder = "");
    
    static bool exportCombinedSingleSatOrbit(const std::vector<OrbitPoint>& navOrbitPoints,
                                             const std::vector<OrbitPoint>& sp3OrbitPoints,
                                             const SatID& sat,
                                             const std::string& subFolder = "");

private:
    static bool ensureDirectoryExists(const std::string& path);
    
    static std::string getOrbitOutputPath(const std::string& subFolder = "");
    
    static bool computeLatLon(double x, double y, double z, double& lat, double& lon);
};

} // namespace gnss