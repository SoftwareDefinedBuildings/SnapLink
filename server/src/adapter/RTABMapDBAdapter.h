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

class RTABMapDBAdapter
{
public:
    /**
     * read data from database files, NULL pointers will be ignored
     */
    static bool readData(const std::vector<std::string> &dbPaths, Words &words, Signatures &signatures, Labels &labels);

private:
    static std::list<Signature *> readSignatures(const std::string &dbPath, int dbId);
    static std::list<rtabmap::VisualWord *> readWords(const std::string &dbPath, int dbId, std::list<Signature *> &signatures);
    static std::list< std::unique_ptr<Label> > readLabels(const std::string &dbPath, int dbId, const std::list<Signature *> &signatures);

    static std::map<int, rtabmap::Transform> getOptimizedPoses(const std::string &dbPath);

    static Signature *convertSignature(const rtabmap::Signature &signature, int dbId);
    static bool getPoint3World(const std::list<Signature *> &signatures, int dbId, int imageId, int x, int y, pcl::PointXYZ &pWorld);

    static std::map<std::pair<int, int>, int> getMergeWordsIdMap(const std::map< int, std::list<rtabmap::VisualWord *> > &words);
    static std::map<std::pair<int, int>, int> getMergeSignaturesIdMap(const std::list<Signature *> &signatures);
    static std::list<rtabmap::VisualWord *> mergeWords(const std::map< int, std::list<rtabmap::VisualWord *> > &words, const std::map<std::pair<int, int>, int> &mergeWordsIdMap, const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap);
    static std::list<Signature *> mergeSignatures(const std::list<Signature *> &signatures, const std::map<std::pair<int, int>, int> &mergeSignaturesIdMap, const std::map<std::pair<int, int>, int> &mergeWordsIdMap);
};
