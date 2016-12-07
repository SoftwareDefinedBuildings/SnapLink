#include <ed25519/ed25519.h>
#include <crypto.h>

void SignBlob(QByteArray &sk, QByteArray &vk, QByteArray *data, QByteArray *sig)
{
    Q_ASSERT(sig->length() == 64);
    Q_ASSERT(vk.length() == 32);
    Q_ASSERT(sk.length() == 32);
    ed25519_sign((unsigned char* )data->data(),data->size(),(unsigned char* )sk.data(),(unsigned char* )vk.data(),(unsigned char* )sig->data());
}

bool VerifyBlob(QByteArray &vk, QByteArray &sig, QByteArray &data)
{
    Q_ASSERT(sig.length() == 64);
    Q_ASSERT(vk.length() == 32);
    return ed25519_sign_open((unsigned char* )data.data(),data.size(),(unsigned char* )vk.data(),(unsigned char* )sig.data()) == 0;
}

QString FmtKey(QByteArray &k)
{
    return QString(k.toBase64(QByteArray::Base64UrlEncoding).data());
}
QByteArray UnFmtKey(QByteArray &k)
{
    return QByteArray::fromBase64(k,QByteArray::Base64UrlEncoding);
}
void ed25519_randombytes_unsafe (void *p, size_t len)
{
    Q_UNUSED(p);
    Q_UNUSED(len);
    qFatal("you should not call this");
    Q_UNREACHABLE();
}
