#include <cassert>
#include "data/LabelsSimple.h"

LabelsSimple::LabelsSimple()
{
}

LabelsSimple::~LabelsSimple()
{
    for (std::map< int, std::list<Label *> >::iterator iter = _labels.begin(); iter != _labels.end(); iter++)
    {
        for (std::list<Label *>::iterator jter = iter->second.begin(); jter != iter->second.end(); jter++)
        {
            delete *jter;
            *jter = NULL;
        }
    }
    _labels.clear();
}

void LabelsSimple::addLabels(const std::list<Label *> &labels)
{
    for (std::list<Label *>::const_iterator iter = labels.begin(); iter != labels.end(); iter++)
    {
        Label *label = *iter;
        if (label != NULL)
        {
            std::map< int, std::list<Label *> >::iterator jter = _labels.find(label->getDbId());
            if (jter == _labels.end())
            {
                _labels.insert(std::pair<int, std::list<Label *> >(label->getDbId(), std::list<Label *>(1, label)));
            }
            else
            {
                jter->second.push_back(label);
            }
        }
    }
}

const std::map< int, std::list<Label *> > &LabelsSimple::getLabels() const
{
    return _labels;
}
