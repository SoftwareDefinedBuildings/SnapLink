#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/utilite/UStl.h>
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "data/Words.h"
#include "data/Signatures.h"
#include "data/Labels.h"

class Words;

class RTABMapDBAdapter : public Adapter
{
public:
    RTABMapDBAdapter();
    virtual ~RTABMapDBAdapter();

    /**
     * read data from database files, NULL pointers will be ignored
     */
    static bool readData(std::vector<std::string> &dbs, Words *words, Signatures *signatures, Labels *labels);
    
    bool init(const rtabmap::ParametersMap &parameters = rtabmap::ParametersMap());

private:
    virtual void parseParameters(const rtabmap::ParametersMap &parameters);
    static std::map<int, Transform> RTABMapDBAdapter::getOptimizedPoses(std::string dbPath);
    static std::list<Label *> readLabels(int dbId, std::string dbPath);
    static bool getPoint3World(int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld);

    std::map<int, int> getNeighborsId(
        int signatureId,
        const std::map<int, rtabmap::Signature *> &signatureMap,
        bool incrementMarginOnLoop = false,
        bool ignoreLoopIds = false,
        bool ignoreIntermediateNodes = false) const;
    std::map<int, rtabmap::Link> getNeighborLinks(int dbId, int sigId) const;
    void getMetricConstraints(
        int dbId,
        const std::set<int> &ids,
        std::map<int, rtabmap::Transform> &poses,
        std::multimap<int, rtabmap::Link> &links);
};
