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

private:
    static std::list<rtabmap::Signature *> readSignatures(std::string &dbPath, int dbId);
    static std::list<rtabmap::VisualWord *> readWords(std::string &dbPath, int dbId, std::list<Signature *> &signatures);
    static std::list<Label *> readLabels(int dbId, std::string dbPath);

    static std::map<int, Transform> getOptimizedPoses(std::string &dbPath);
    
    static Signature *convertSignature(const rtabmap::Signature *signature);

    static std::map<std::pair<int, int>, int> getMergeWordsIdMap(const std::map< int, std::list<rtabmap::VirtualWord *> > &words);
    static std::map<std::pair<int, int>, int> getMergeSignaturesIdMap(const std::map< int, std::list<rtabmap::Signature *> > &signaturesMap);

    static std::list<rtabmap::VirtualWord *> mergeWords(std::map< int, std::list<rtabmap::VirtualWord *> >, std::map<std::pair<int, int>, int> *IdMap = NULL);
    static std::list<rtabmap::Signature *> mergeSignatures(std::map< int, std::list<rtabmap::Signature *> >, std::map<std::pair<int, int>, int> *IdMap = NULL);

    static bool getPoint3World(int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld);
};
