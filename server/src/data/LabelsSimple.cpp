#include "data/LabelsSimple.h"

void LabelsSimple::putLabels(std::list< std::unique_ptr<Label> > &&labels)
{
    for (auto & label : labels)
    {
        if (label != nullptr)
        {
            auto iter = _labels.find(label->getDbId());
            if (iter == _labels.end())
            {
                int dbId = label->getDbId();
                auto labelList = std::list< std::unique_ptr<Label> >();
                labelList.emplace_back(std::move(label));
                _labels.insert(std::make_pair(dbId, std::move(labelList)));
            }
            else
            {
                iter->second.push_back(std::move(label));
            }
        }
    }
}

const std::map< int, std::list< std::unique_ptr<Label> > > &LabelsSimple::getLabels() const
{
    return _labels;
}
