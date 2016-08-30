#pragma once

#include <QObject>
#include "algo/Feature.h"

class WordSearchStage;

class FeatureStage : public QObject {
public:
  FeatureStage();
  ~FeatureStage();

  void setWordSearchStage(WordSearchStage *wordSearchStage);

protected:
  virtual bool event(QEvent *event);

private:
  WordSearchStage *_wordSearchStage;
  Feature _feature;
};
