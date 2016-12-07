#ifndef CRYPTO_H
#define CRYPTO_H
#include <QByteArray>
#include <QString>
void SignBlob(QByteArray &sk, QByteArray &vk, QByteArray *data, QByteArray *sig);
bool VerifyBlob(QByteArray &vk, QByteArray &sig, QByteArray &data);
QString FmtKey(QByteArray &k);
QByteArray UnFmtKey(QByteArray &k);
#endif // CRYPTO_H
