#pragma once

#include <map>
#include <list>
#include <memory>
#include "data/Label.h"

class Labels
{
public:
    /**
     * Add labels, ownership transfer
     */
    virtual void putLabels(std::list< std::unique_ptr<Label> > &&labels) = 0;

    /**
     * get all labels
     */
    virtual const std::map< int, std::list< std::unique_ptr<Label> > > &getLabels() const = 0;
};
