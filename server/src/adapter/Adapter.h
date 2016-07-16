#pragma once

#include <vector>
#include "data/Words.h"
#include "data/Signatures.h"
#include "data/Labels.h"

class Adapter
{
public:
    /**
     * read data from database files, NULL pointers will be ignored
     */
    virtual static bool readData(std::vector<std::string> &dbs, Words *words, Signatures *signatures, Labels *labels) = 0;
};
