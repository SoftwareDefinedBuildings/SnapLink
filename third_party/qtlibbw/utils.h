#ifndef QTLIBBW_UTILS_H
#define QTLIBBW_UTILS_H

#include <QJSValue>
#include <QJSValueList>
#include <functional>
#include <QJSEngine>


using std::function;

template<typename... Tz> void invokeOnThread(QThread* t, function<void (Tz...)> f, Tz... args)
{
    QTimer* timer = new QTimer();
    timer->moveToThread(t);
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, [=]{
        f(args...);
        timer->deleteLater();
    });
    QMetaObject::invokeMethod(timer, "start", Qt::QueuedConnection, Q_ARG(int, 0));
}

template <typename F, typename ...R> void convert(QJSValueList &l, F f, R... rest)
{
    l.append(QJSValue(f));
    convert(l, rest...);
}
template <typename F> void convert(QJSValueList &l, F f)
{
    l.append(QJSValue(f));
}
template <typename F, typename ...R> void convertE(QJSEngine* e, QJSValueList &l, F f, R... rest)
{
    l.append(e->toScriptValue(f));
    convertE(e, l, rest...);
}
template <typename F> void convertE(QJSEngine* e, QJSValueList &l, F f)
{
    l.append(e->toScriptValue(f));
}
/*
template<typename ...Tz> function<void(Tz...)> mkcb(QJSValue callback)
{
    if (!callback.isCallable())
    {
        qFatal("Called mkcb with non function");
    }
    return [=](Tz... args) mutable
    {
        QJSValueList l;
        convert(l, args...);
        callback.call(l);
    };
}*/


#if 0
/**
 * This is an internal class that allows you to convert a javascript callback function to
 * a std::function with a specific signature e.g.
 * CB<QString>(myjscb);
 */
template<typename ...Tz>  class CB
{
public:
    CB(QJSValue callback)
        :callback(callback)
    {

    }
    typedef function<void(Tz...)> fun_t;
    operator fun_t()
    {
        return [=](Tz ...args) mutable
        {
            QJSValueList l;
            convert(l, args...);
            callback.call(l);
        };
    }

    private:
    //This is basically so that we can convert every argument into a QJSValue
    template <typename F, typename ...R> void convert(QJSValueList &l, F f, R... rest)
    {
        l.append(QJSValue(f));
        convert(l, rest...);
    }
    template <typename F> void convert(QJSValueList &l, F f)
    {
        l.append(QJSValue(f));
    }
    QJSValue callback;
};
#endif
#endif // QTLIBBW_UTILS_H
