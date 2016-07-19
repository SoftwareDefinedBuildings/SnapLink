#pragma once

#include <map>
#include <list>
#include "data/Label.h"

class Labels
{
public:
    /**
     * Add labels, ownership transfer
     */
    virtual void addLabels(const std::list<Label *> &labels) = 0;

    /**
     * get all labels
     */
    virtual const std::map< int, std::list<Label *> > &getLabels() const = 0;
};
