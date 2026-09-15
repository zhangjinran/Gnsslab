/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: Shoujian Zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */

#ifndef GNSSLAB_RINEXOBSREADER_H
#define GNSSLAB_RINEXOBSREADER_H
#include <fstream>
#include <gnsslab/GnssStruct.h>

class RinexObsReader {
public:
    RinexObsReader()
        : pFileStream(nullptr), isHeaderRead(false), ownStream(false), currentLine(0)
    {};

    RinexObsReader(const RinexObsReader& other) = delete;
    RinexObsReader& operator=(const RinexObsReader& other) = delete;

    RinexObsReader(RinexObsReader&& other) noexcept;
    RinexObsReader& operator=(RinexObsReader&& other) noexcept;

    ~RinexObsReader();

    bool loadFile(const std::string& filePath);

    void setFileStream(std::fstream* pStream, bool takeOwnership = false);

    void setSelectedTypes(std::map<string, std::set<string>>& systemTypes)
    {
        sysTypes = systemTypes;
    };
    std::map<string, std::set<string>> getSystemTypes() const { return sysTypes; }


    void parseRinexHeader();
    ObsData parseRinexObs();

    ObsData parseRinexObs(CommonTime& syncEpoch)
    {
        streampos sp(pFileStream->tellg());
        ObsData obsData;
        while(true){
            if(pFileStream->peek() == EOF){
                break;
            }
            obsData = parseRinexObs();
            if(obsData.epoch >= syncEpoch)
            {
                break;
            }
        }
        if(obsData.epoch > (syncEpoch + 0.001))
        {
            pFileStream->seekg(sp);
            SyncException e("Rx3ObsData::can't synchronize the obs at line " + std::to_string(currentLine));
            throw(e);
        }
        return obsData;
    };

    CommonTime parseTime(const string &line);
    void chooseObs(ObsData &obsData);
    void static_Obs(ObsData &obsData, ObsDataStaticSum* obs_data_static_sum);
    
    // 统计指定系统中同时有两种观测码的卫星数量
    int countDualCodeSatellites(ObsData &obsData, const std::string& system, 
                                const std::string& code1, const std::string& code2);

    bool isOpen() const { return pFileStream && *pFileStream; }
    const RinexHeader& getHeader() const { return rinexHeader; }
    size_t getCurrentLine() const { return currentLine; }

private:
    std::fstream* pFileStream;
    RinexHeader rinexHeader;
    std::map<string, std::set<string>> sysTypes;
    std::set<std::string> allowedSystems;
    bool isHeaderRead;
    bool ownStream;
    size_t currentLine;

    bool isSystemAllowed(const std::string& system) const;
};


#endif //GNSSLAB_RINEXOBSREADER_H