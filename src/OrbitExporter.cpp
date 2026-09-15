#include <gnsslab/OrbitExporter.h>
#include <gnsslab/RinexNavStore.hpp>
#include <gnsslab/SP3Store.hpp>
#include <gnsslab/CoordConvert.h>
#include <gnsslab/CoordStruct.h>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <filesystem>

namespace gnss {

inline long long timeToMillis(const CommonTime& time) {
    return time.m_day * 86400000LL + static_cast<long long>(time.m_sod * 1000.0);
}

std::string OrbitExporter::getOutputBasePath() {
    namespace fs = std::filesystem;

    fs::path path = fs::current_path() / "outputs" / "orbit";
    fs::create_directories(path);

    return (path / "").string();
}

bool OrbitExporter::ensureDirectoryExists(const std::string& path) {
    std::error_code error;
    std::filesystem::create_directories(path, error);
    return !error;
}

std::string OrbitExporter::getOrbitOutputPath(const std::string& subFolder) {
    std::string path = getOutputBasePath();
    if (!subFolder.empty()) {
        path += subFolder + "/";
    }
    ensureDirectoryExists(path);
    return path;
}

bool OrbitExporter::computeLatLon(double x, double y, double z, double& lat, double& lon) {
    XYZ xyz(x, y, z);
    WGS84 wgs84;
    BLH blh = xyz2blh(xyz, wgs84);
    lat = blh.B();
    lon = blh.L();
    return true;
}

std::vector<OrbitPoint> OrbitExporter::computeOrbitPoints(RinexNavStore& navStore,
                                                          const SatID& sat,
                                                          const CommonTime& startTime,
                                                          double durationHours,
                                                          double intervalSeconds) {
    std::vector<OrbitPoint> points;
    
    double durationSeconds = durationHours * 3600.0;
    int numPoints = static_cast<int>(durationSeconds / intervalSeconds) + 1;
    
    for (int i = 0; i < numPoints; ++i) {
        CommonTime currentTime = startTime + i * intervalSeconds;
        
        try {
            Xvt xvt = navStore.getXvt(sat, currentTime);
            Eigen::Vector3d pos = xvt.getPos();
            
            OrbitPoint point;
            point.time = currentTime;
            point.sat = sat;
            point.x = pos(0);
            point.y = pos(1);
            point.z = pos(2);
            point.radius = pos.norm();
            
            computeLatLon(point.x, point.y, point.z, point.lat, point.lon);
            
            std::unique_ptr<NavEphBase> eph = navStore.findEph(sat, currentTime);
            if (eph) {
                point.svURA = eph->svURA(currentTime);
            } else {
                point.svURA = 0.0;
            }
            
            points.push_back(point);
        } catch (const std::exception& e) {
            continue;
        }
    }
    
    return points;
}

bool OrbitExporter::exportOrbitData(const OrbitDataMap& orbitData, const std::string& subFolder) {
    std::string path = getOrbitOutputPath(subFolder);
    std::ofstream file(path + "orbit_data.txt");
    
    if (!file) return false;
    
    file << "# GNSS Orbit Data\n";
    file << "# Format: time,sat,x(m),y(m),z(m),radius(m),lat(rad),lon(rad),svURA(m)\n";
    
    for (const auto& entry : orbitData) {
        const SatID& sat = entry.first;
        const std::vector<OrbitPoint>& points = entry.second;
        
        for (const OrbitPoint& point : points) {
            file << point.time << ","
                 << sat.system << sat.id << ","
                 << std::fixed << std::setprecision(3) << point.x << ","
                 << std::fixed << std::setprecision(3) << point.y << ","
                 << std::fixed << std::setprecision(3) << point.z << ","
                 << std::fixed << std::setprecision(3) << point.radius << ","
                 << std::fixed << std::setprecision(10) << point.lat << ","
                 << std::fixed << std::setprecision(10) << point.lon << ","
                 << std::fixed << std::setprecision(3) << point.svURA << "\n";
        }
    }
    
    return true;
}

bool OrbitExporter::exportSingleSatOrbit(const std::vector<OrbitPoint>& orbitPoints,
                                         const SatID& sat,
                                         const std::string& subFolder) {
    std::string path = getOrbitOutputPath(subFolder);
    std::string filename = path + sat.system + std::to_string(sat.id) + "_orbit.txt";
    std::ofstream file(filename);
    
    if (!file) return false;
    
    file << "# Orbit Data for Satellite " << sat.system << sat.id << "\n";
    file << "# Format: time(MJD+SOD),x(m),y(m),z(m),radius(m),lat(deg),lon(deg)\n";
    
    for (const OrbitPoint& point : orbitPoints) {
        double mjd = point.time.m_day + point.time.m_sod / 86400.0;
        file << std::fixed << std::setprecision(10) << mjd << ","
             << std::fixed << std::setprecision(3) << point.x << ","
             << std::fixed << std::setprecision(3) << point.y << ","
             << std::fixed << std::setprecision(3) << point.z << ","
             << std::fixed << std::setprecision(3) << point.radius << ","
             << std::fixed << std::setprecision(8) << rad2deg(point.lat) << ","
             << std::fixed << std::setprecision(8) << rad2deg(point.lon) << "\n";
    }
    
    return true;
}

bool OrbitExporter::exportRadiusAnalysis(const OrbitDataMap& orbitData, const std::string& subFolder) {
    std::string path = getOrbitOutputPath(subFolder);
    std::ofstream file(path + "radius_analysis.txt");
    
    if (!file) return false;
    
    file << "# Radius Analysis\n";
    file << "# Format: time(MJD+SOD),sat,radius(km),dr_prev(km)\n";
    
    for (const auto& entry : orbitData) {
        const SatID& sat = entry.first;
        const std::vector<OrbitPoint>& points = entry.second;
        
        for (size_t i = 0; i < points.size(); ++i) {
            const OrbitPoint& point = points[i];
            double mjd = point.time.m_day + point.time.m_sod / 86400.0;
            double dr_prev = 0.0;
            
            if (i > 0) {
                double dx = point.x - points[i-1].x;
                double dy = point.y - points[i-1].y;
                double dz = point.z - points[i-1].z;
                dr_prev = sqrt(dx*dx + dy*dy + dz*dz) / 1000.0;
            }
            
            file << std::fixed << std::setprecision(10) << mjd << ","
                 << sat.system << sat.id << ","
                 << std::fixed << std::setprecision(3) << point.radius / 1000.0 << ","
                 << std::fixed << std::setprecision(6) << dr_prev << "\n";
        }
    }
    
    return true;
}

bool OrbitExporter::exportVelocityAnalysis(const OrbitDataMap& orbitData, const std::string& subFolder) {
    std::string path = getOrbitOutputPath(subFolder);
    std::ofstream file(path + "velocity_analysis.txt");
    
    if (!file) return false;
    
    file << "# Velocity Analysis\n";
    file << "# Format: time(MJD+SOD),sat,radius(km),velocity(km/s)\n";
    
    for (const auto& entry : orbitData) {
        const SatID& sat = entry.first;
        const std::vector<OrbitPoint>& points = entry.second;
        
        for (size_t i = 1; i < points.size(); ++i) {
            const OrbitPoint& curr = points[i];
            const OrbitPoint& prev = points[i-1];
            
            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            double dz = curr.z - prev.z;
            
            double dt = curr.time - prev.time;
            if (fabs(dt) < 1e-6) {
                continue;
            }
            
            double velocity = sqrt(dx*dx + dy*dy + dz*dz) / dt / 1000.0;
            double mjd = curr.time.m_day + curr.time.m_sod / 86400.0;
            
            file << std::fixed << std::setprecision(10) << mjd << ","
                 << sat.system << sat.id << ","
                 << std::fixed << std::setprecision(3) << curr.radius / 1000.0 << ","
                 << std::fixed << std::setprecision(6) << velocity << "\n";
        }
    }
    
    return true;
}

bool OrbitExporter::exportVelocityAnalysis(const OrbitDataMap& navOrbitData, 
                                           SP3Store* sp3Store,
                                           const std::string& subFolder) {
    std::string path = getOrbitOutputPath(subFolder);
    std::ofstream file(path + "velocity_analysis.txt");
    
    if (!file) return false;
    
    file << "# Velocity Analysis\n";
    file << "# Format: time(MJD+SOD),sat,radius(km),velocity_broadcast(km/s),velocity_sp3(km/s)\n";
    
    for (const auto& entry : navOrbitData) {
        const SatID& sat = entry.first;
        const std::vector<OrbitPoint>& navPoints = entry.second;
        
        std::vector<OrbitPoint> sp3Points;
        bool hasSP3 = false;
        if (sp3Store != nullptr && sat.system != "I") {
            sp3Points = computeSP3OrbitPoints(*sp3Store, sat, navPoints.front().time,
                                              24.0, 300.0);
            hasSP3 = !sp3Points.empty();
        }
        
        std::map<long long, OrbitPoint> sp3Map;
        if (hasSP3) {
            for (const OrbitPoint& point : sp3Points) {
                long long key = timeToMillis(point.time);
                sp3Map[key] = point;
            }
        }
        
        for (size_t i = 1; i < navPoints.size(); ++i) {
            const OrbitPoint& curr = navPoints[i];
            const OrbitPoint& prev = navPoints[i-1];
            
            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            double dz = curr.z - prev.z;
            
            double dt = curr.time - prev.time;
            if (fabs(dt) < 1e-6) {
                continue;
            }
            
            double navVelocity = sqrt(dx*dx + dy*dy + dz*dz) / dt / 1000.0;
            double mjd = curr.time.m_day + curr.time.m_sod / 86400.0;
            
            double sp3Velocity = -1.0;
            if (hasSP3) {
                long long key = timeToMillis(curr.time);
                auto it = sp3Map.find(key);
                if (it != sp3Map.end()) {
                    const OrbitPoint& sp3Curr = it->second;
                    long long prevKey = timeToMillis(prev.time);
                    auto prevIt = sp3Map.find(prevKey);
                    if (prevIt != sp3Map.end()) {
                        const OrbitPoint& sp3Prev = prevIt->second;
                        double sp3Dx = sp3Curr.x - sp3Prev.x;
                        double sp3Dy = sp3Curr.y - sp3Prev.y;
                        double sp3Dz = sp3Curr.z - sp3Prev.z;
                        sp3Velocity = sqrt(sp3Dx*sp3Dx + sp3Dy*sp3Dy + sp3Dz*sp3Dz) / dt / 1000.0;
                    }
                }
            }
            
            file << std::fixed << std::setprecision(10) << mjd << ","
                 << sat.system << sat.id << ","
                 << std::fixed << std::setprecision(3) << curr.radius / 1000.0 << ","
                 << std::fixed << std::setprecision(6) << navVelocity;
            
            if (hasSP3 && sp3Velocity >= 0) {
                file << "," << std::fixed << std::setprecision(6) << sp3Velocity;
            } else {
                file << ",NaN";
            }
            file << "\n";
        }
    }
    
    return true;
}

bool OrbitExporter::exportGroundTrack(const OrbitDataMap& orbitData, const std::string& subFolder) {
    std::string path = getOrbitOutputPath(subFolder);
    std::ofstream file(path + "ground_track.txt");
    
    if (!file) return false;
    
    file << "# Ground Track\n";
    file << "# Format: sat,lat(deg),lon(deg)\n";

    for (const auto& entry : orbitData) {
        const SatID& sat = entry.first;
        const std::vector<OrbitPoint>& points = entry.second;

        for (const OrbitPoint& point : points) {
            file << sat.system << sat.id << ","
                 << std::fixed << std::setprecision(8) << rad2deg(point.lat) << ","
                 << std::fixed << std::setprecision(8) << rad2deg(point.lon) << "\n";
        }
    }
    
    return true;
}

std::vector<OrbitPoint> OrbitExporter::computeSP3OrbitPoints(SP3Store& sp3Store,
                                                             const SatID& sat,
                                                             const CommonTime& startTime,
                                                             double durationHours,
                                                             double intervalSeconds) {
    std::vector<OrbitPoint> points;
    
    try {
        SatIDSet satSet = sp3Store.getSatSet();
        if (satSet.find(sat) == satSet.end()) {
            return points;
        }
    } catch (...) {
        return points;
    }
    
    double durationSeconds = durationHours * 3600.0;
    int numPoints = static_cast<int>(durationSeconds / intervalSeconds) + 1;
    
    for (int i = 0; i < numPoints; ++i) {
        CommonTime currentTime = startTime + i * intervalSeconds;
        
        try {
            Xvt xvt = sp3Store.getXvt(sat, currentTime);
            Eigen::Vector3d pos = xvt.getPos();
            
            double posNorm = pos.norm();
            double scaleFactor = 1.0;
            if (posNorm < 50000.0) {
                scaleFactor = 1000.0;
            }
            
            OrbitPoint point;
            point.time = currentTime;
            point.sat = sat;
            point.x = pos(0) * scaleFactor;
            point.y = pos(1) * scaleFactor;
            point.z = pos(2) * scaleFactor;
            point.radius = posNorm * scaleFactor;
            
            computeLatLon(point.x, point.y, point.z, point.lat, point.lon);
            
            points.push_back(point);
        } catch (const std::exception& e) {
            continue;
        }
    }
    
    return points;
}

bool OrbitExporter::exportCombinedSingleSatOrbit(const std::vector<OrbitPoint>& navOrbitPoints,
                                                 const std::vector<OrbitPoint>& sp3OrbitPoints,
                                                 const SatID& sat,
                                                 const std::string& subFolder) {
    std::string path = getOrbitOutputPath(subFolder);
    std::string filename = path + sat.system + std::to_string(sat.id) + "_combined_orbit.txt";
    std::ofstream file(filename);
    
    if (!file) return false;
    
    file << "# Combined Orbit Data for Satellite " << sat.system << sat.id << "\n";
    file << "# Format: time(MJD+SOD),x_broadcast(m),y_broadcast(m),z_broadcast(m),x_precise(m),y_precise(m),z_precise(m)\n";
    file << "# Note: SP3 precise ephemeris sampled at broadcast epochs\n";
    file << "#       For IRNSS satellites, precise ephemeris columns are filled with NaN\n";
    
    std::map<long long, OrbitPoint> sp3Map;
    for (const OrbitPoint& point : sp3OrbitPoints) {
        long long key = timeToMillis(point.time);
        sp3Map[key] = point;
    }
    
    for (const OrbitPoint& navPoint : navOrbitPoints) {
        long long key = timeToMillis(navPoint.time);
        double mjd = navPoint.time.m_day + navPoint.time.m_sod / 86400.0;
        
        file << std::fixed << std::setprecision(10) << mjd << ","
             << std::fixed << std::setprecision(3) << navPoint.x << ","
             << std::fixed << std::setprecision(3) << navPoint.y << ","
             << std::fixed << std::setprecision(3) << navPoint.z;
        
        auto it = sp3Map.find(key);
        if (it != sp3Map.end()) {
            const OrbitPoint& sp3Point = it->second;
            file << ","
                 << std::fixed << std::setprecision(3) << sp3Point.x << ","
                 << std::fixed << std::setprecision(3) << sp3Point.y << ","
                 << std::fixed << std::setprecision(3) << sp3Point.z << "\n";
        } else {
            file << ",NaN,NaN,NaN\n";
        }
    }
    
    return true;
}

bool OrbitExporter::exportCombinedOrbitData(const OrbitDataMap& navOrbitData,
                                            SP3Store* sp3Store,
                                            const std::string& subFolder) {
    for (const auto& entry : navOrbitData) {
        const SatID& sat = entry.first;
        const std::vector<OrbitPoint>& navPoints = entry.second;
        
        std::vector<OrbitPoint> sp3Points;
        bool temp=1;
        if (sat.system=="I")
             temp=0;
        if (sp3Store != nullptr &&temp) {
            sp3Points = computeSP3OrbitPoints(*sp3Store, sat, navPoints.front().time,
                                              24.0, 300.0);
        }
        
        exportCombinedSingleSatOrbit(navPoints, sp3Points, sat, subFolder);
    }
    
    return true;
}

} // namespace gnss
