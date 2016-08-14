#pragma once

#include <memory>
#include <QObject>
#include <QEvent>
#include "data/SensorData.h"
#include "adapter/RTABMapDBAdapter.h"
#include "stage/SignatureSearch.h"
#include "data/Transform.h"

class SignatureSearch;
class HTTPServer;

class WordSearch :
    public QObject
{
public:
    WordSearch();
    virtual ~WordSearch();

    void putWords(std::unique_ptr<Words> &&words);
    void setSignatureSearch(SignatureSearch *imageSearch);

protected:
    virtual bool event(QEvent *event);

private:
    std::vector<int> searchWords(const SensorData &sensorData) const;

private:
    std::unique_ptr<Words> _words;
    SignatureSearch *_imageSearch;
};
