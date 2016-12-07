#include "libbw.h"
#include "bosswave.h"
#include <QQmlExtensionPlugin>
#include <qqml.h>

void initLibBW()
{
    qDebug() << "doing initlibbw";
    qmlRegisterSingletonType<BW>("BOSSWAVE", 1, 0, "BW", &BW::qmlSingleton);
    qmlRegisterType<MetadataTupleJS>("BOSSWAVE", 1, 0, "MetadataTuple");
    qmlRegisterType<BalanceInfo>("BOSSWAVE", 1, 0, "BalanceInfo");
    qmlRegisterType<RoutingObject>("BOSSWAVE", 1, 0, "RoutingObject");
    qmlRegisterType<Entity>("BOSSWAVE", 1, 0, "Entity");
}
