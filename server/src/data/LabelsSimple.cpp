#include <cassert>
#include "data/LabelsSimple.h"

LabelsSimple::LabelsSimple()
{
}

LabelsSimple::~LabelsSimple()
{
    for (auto & _label : _labels)
    {
        for (auto & jter : _label.second)
        {
            delete jter;
            jter = nullptr;
        }
    }
    _labels.clear();
}

void LabelsSimple::addLabels(const std::list<Label *> &labels)
{
    for (auto label : labels)
    {
        if (label != nullptr)
        {
            auto jter = _labels.find(label->getDbId());
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
