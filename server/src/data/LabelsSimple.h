#pragma once

#include "data/Labels.h"

class LabelsSimple : public Labels
{
public:
    LabelsSimple();
    virtual ~LabelsSimple();

    /**
     * Add labels, ownership transfer
     */
    void addLabels(const std::list<Label *> &labels);

    /**
     * get all labels
     */
    const std::map< int, std::list<Label *> > &getLabels() const;

private:
    std::map< int, std::list<Label *> > _labels;
};
