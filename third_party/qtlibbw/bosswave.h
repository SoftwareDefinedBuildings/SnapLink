#ifndef QTLIBBW_BOSSWAVE_H
#define QTLIBBW_BOSSWAVE_H

#include <QDateTime>
#include <QQuickItem>
#include <QTimer>
#include <QJSValueList>
#include <QSharedPointer>

#include "utils.h"
#include "agentconnection.h"
#include "message.h"

QT_FORWARD_DECLARE_CLASS(MetadataTuple)
QT_FORWARD_DECLARE_CLASS(BalanceInfo)
QT_FORWARD_DECLARE_CLASS(SimpleChain)
QT_FORWARD_DECLARE_CLASS(BWView)

const QString elaborateDefault("");
const QString elaborateFull("full");
const QString elaboratePartial("partial");
const QString elaborateNone("none");


/*! \mainpage BOSSWAVE Wavelet Viewer
 *
 * \section intro_sec Introduction
 *
 * This contains the documentation for the BOSSWAVE Wavelet API.
 *
 * The functions are grouped into two modules, those accessible only from C++,
 * and those accessible from both C++ and QML.
 *
 * To view the documentation, click the "Wavelet API overview" tab above.
 */

/**
 * \defgroup cpp C++ API
 * \brief Functions for interacting with BOSSWAVE from C++ code
 *
 * These functions are not accessible from Wavelets, but for programs that
 * are fully standalone (compiled against libqtbw) they can be used to obtain
 * functionality not found in the QML API. All the QML functions can also be
 * invoked from C++.
 */

/**
 * \defgroup qml QML API
 * \brief Functions accessible to Wavelets
 *
 * These functions can be used in Wavelets by importing the BOSSWAVE module:
 * \code{.qml}
 * import BOSSWAVE 1.0
 *
 * Button {
 *   onClicked:
 *   {
 *     BW.publishMsgPack("my/url", "2.0.7.2", {"key":"value"},
 *       function(status) {
 *         console.log("publish status:", status);
 *       });
 *   }
 * }
 * \endcode
 *
 * They can also be used in C++ via normal function invocations.
 */

class BW : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(BW)

public:
    BW(QObject *parent = 0);
    ~BW();

    enum RegistryValidity
    {
        StateUnknown = 0,
        StateValid = 1,
        StateExpired = 2,
        StateRevoked = 3,
        StateError = 4
    };
    Q_ENUM(RegistryValidity)

    // This is used by the QML engine to instantiate the bosswave singleton
    static QObject *qmlSingleton(QQmlEngine *engine, QJSEngine *scriptEngine);

    /**
     * @brief This can be used to obtain references to the BW object
     * @return The BW2 singleton instance
     *
     * @ingroup cpp
     * @since 1.0
     */
    static BW *instance();

    /**
     * @brief Get the integer PO Number from dot form
     * @param df The dot form, e.g. 2.0.3.1
     * @return The integer PONum
     *
     * @ingroup cpp
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE static int fromDF(QString df);

    /**
     * @brief Connect to a BOSSWAVE agent
     * @param ourentity The entity to use to connect
     *
     * Set the agent to the given host and port. Any existing connection will
     * be torn down. Any existing subscriptions / views will not be recreated
     * and the entity must be set again. agentConnected() will be signalled
     * when this process is complete
     *
     * @ingroup cpp
     * @since 1.4
     */
    void connectAgent(QByteArray &ourentity);

    /**
     * @brief Create a new entity
     * @param expiry The time at which the entity expires. Ignored if invalid (year == 0)
     * @param expiryDelta The number of milliseconds after the current time at which the entity expires
     * @param contact Contact info
     * @param comment Comment
     * @param revokers List of revoker VKs
     * @param omitCreationDate If true, the creation date is omitted
     * @param on_done Callback that takes as arguments (1) the error message (empty string if none), (2) the VK of the new entity, and (3) binary representation
     *
     * @ingroup cpp
     * @since 1.4
     */
    void createEntity(QDateTime expiry, qreal expiryDelta, QString contact,
                      QString comment, QList<QString> revokers, bool omitCreationDate,
                      Res<QString, QString, QByteArray> on_done);

    /**
     * @brief Create a new entity
     * @param params A map of parameters. Keys are: (1) Expiry, (2) ExpiryDelta, (3) Contact, (4) Comment, (5) Revokers, and (6) OmitCreationDate.
     * @param on_done Javascript callback executed with a single argument: an error message, or the empty string if no error occurred
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void createEntity(QVariantMap params, QJSValue on_done);

    /**
     * @brief Create a Declaration of Trust (DOT)
     * @param isPermission True if this DOT is a permission DOT (which are not yet implemented)
     * @param to The entity to which the DOT is created
     * @param ttl The time-to-live (TTL) of this DOT
     * @param expiry The time at which the entity expires. Ignored if invalid (year == 0)
     * @param expiryDelta The number of milliseconds after the current time at which the entity expires
     * @param contact Contact info
     * @param comment Comment
     * @param revokers List of revokers
     * @param omitCreationDate If true, the creation date is omitted
     * @param uri The URI to which permissions are granted
     * @param accessPermissions String describing the permissions granted
     * @param appPermissions Used for permission DOTs
     * @param on_done Callback that takes are arguments (1) the error message (empty string if none), (2) the hash of the new dot, and (3) binary representation
     *
     * @ingroup cpp
     * @since 1.4
     */
    void createDOT(bool isPermission, QString to, unsigned int ttl, QDateTime expiry,
                   qreal expiryDelta, QString contact, QString comment,
                   QList<QString> revokers, bool omitCreationDate, QString uri,
                   QString accessPermissions, QVariantMap appPermissions,
                   Res<QString, QString, QByteArray> on_done);

    /**
     * @brief Create a Declaration of Trust (DOT)
     * @param params A map of parameters. Keys are: (1) IsPermission, (2) To, (3) TTL, (4) Expiry, (5) ExpiryDelta, (6) Contact, (7) Comment, (8) Revokers, (9) OmitCreationDate, (10) URI, (11) AccessPermissions, and (12) AppPermissions
     * @param on_done Javascript callback executed with a single argument: an error message, or the empty string if no error occurred
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void createDOT(QVariantMap params, QJSValue on_done);

    /**
     * @brief Create a Chain of DOTs
     * @param dots The dots to use
     * @param isPermission True if this is a permission chain (not supported)
     * @param unElaborate If true, returns only the hashes of the DOTs, rather than the DOTs themselves
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the hash of the DOT chain, and (3) the DOT chain itself
     *
     * @ingroup cpp
     * @since 1.4
     */
    void createDOTChain(QList<QString> dots, bool isPermission, bool unElaborate,
                        Res<QString, QString, RoutingObject*> on_done);

    /**
     * @brief Create a Chain of DOTs
     * @param params A map of parameters. Keys are: (1) DOTs, (2) IsPermission, and (3) UnElaborate
     * @param on_done Javascript callback executed with a single argument: an error message, or the empty string if no error occurred
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void createDOTChain(QVariantMap params, QJSValue on_done);

    /**
     * @brief Publish to a resource
     * @param uri The resource to publish to
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param poz Payload objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param persist If true, the message is persisted
     * @param on_done The callback that is executed when the publish process is complete. Takes one argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publish(QString uri, QString primaryAccessChain, bool autoChain,
                 QList<RoutingObject*> roz, QList<PayloadObject*> poz,
                 QDateTime expiry, qreal expiryDelta, QString elaboratePAC, bool doNotVerify,
                 bool persist, Res<QString> on_done = _nop_res_status);

    /**
     * @brief Publish a MsgPack object to a resource
     * @param uri The resource to publish to
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param ponum The payload object number for the message
     * @param val The message, as a QVariantMap
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param persist If true, the message is persisted
     * @param on_done The callback that is executed when the publish process is complete. Takes one argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishMsgPack(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                        int ponum, QVariantMap val, QDateTime expiry, qreal expiryDelta,
                        QString elaboratePAC, bool doNotVerify, bool persist,
                        Res<QString> on_done = _nop_res_status);

    /**
     * @brief Publish a MsgPack object to a resource
     * @param params A map of parameters. Keys are: (1) URI, (2) PrimaryAccessChain, (3) AutoChain, (4) RoutingObjects, (5) Payload, (6) PONum, (7) Expiry, (8) ExpiryDelta, (9) ElaboratePAC, (10) DoNotVerify, and (11) Persist
     * @param on_done Javascript callback invoked at the end of the publish process with one argument: an error message, or the empty string if no error occurred
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishMsgPack(QVariantMap params, QJSValue on_done);

    /**
     * @brief Publish text to a resource
     * @param uri The resource to publish to
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param ponum The payload object number for the message
     * @param val The message, as a QVariantMap
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param persist If true, the message is persisted
     * @param on_done The callback that is executed when the publish process is complete. Takes one argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishText(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                     int ponum, QString msg, QDateTime expiry, qreal expiryDelta,
                     QString elaboratePAC, bool doNotVerify, bool persist,
                     Res<QString> on_done = _nop_res_status);

    /**
     * @brief Publish text to a resource
     * @param params A map of parameters. Keys are: (1) URI, (2) PrimaryAccessChain, (3) AutoChain, (4) RoutingObjects, (5) Payload, (6) PONum, (7) Expiry, (8) ExpiryDelta, (9) ElaboratePAC, (10) DoNotVerify, and (11) Persist
     * @param on_done Javascript callback invoked at the end of the publish process with one argument: an error message, or the empty string if no error occurred
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishText(QVariantMap params, QJSValue on_done);

    /**
     * @brief Subscribe to a resource
     * @param uri The resource to subscribe to
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_msg The callback that is executed when a message is received
     * @param on_done The callback that is executed when the subscribe process is complete. First argument is an error message, or the empty string if no error occurred. Second argument is the subscription handle.
     *
     * @ingroup cpp
     * @since 1.4
     */
    void subscribe(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                   QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
                   bool doNotVerify, bool leavePacked, Res<PMessage> on_msg,
                   Res<QString, QString> on_done = _nop_res_status2);

    /**
     * @brief Subscribe to a MsgPack resource
     * @param uri The resource to subscribe to
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_msg The callback that is executed when a message is received, with two arguments: (1) the PO number, and (2) the unpacked contents
     * @param on_done The callback that is executed when the subscribe process is complete. First argument is an error message, or the empty string if no error occurred. Second argument is the subscription handle.
     *
     * @ingroup cpp
     * @since 1.4
     */
    void subscribeMsgPack(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                          QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
                          bool doNotVerify, bool leavePacked, Res<int, QVariantMap, QVariantMap> on_msg,
                          Res<QString, QString> on_done = _nop_res_status2);

    /**
     * @brief Subscribe to a MsgPack resource
     * @param params A map of parameters. Keys are: (1) URI, (2) PrimaryAccessChain, (3) AutoChain, (4) RoutingObjects, (5) Expiry, (6) ExpiryDelta, (7) ElaboratePAC, (8) DoNotVerify, and (9) LeavePacked
     * @param on_msg Javascript callback invoked on the arrival of each message with two arguments: (1) the payload object number, and (2) the payload as an object
     * @param on_done The callback that is executed when the subscribe process is complete. First argument is an error message, or the empty string if no error occurred. Second argument is the subscription handle.
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void subscribeMsgPack(QVariantMap params, QJSValue on_msg, QJSValue on_done);

    /**
     * @brief Subscribe to a text resource
     * @param uri The resource to subscribe to
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_msg The callback that is executed when a message is received, with two arguments: (1) the PO number, and (2) the text contents
     * @param on_done The callback that is executed when the subscribe process is complete. First argument is an error message, or the empty string if no error occurred. Second argument is the subscription handle.
     *
     * @ingroup cpp
     * @since 1.4
     */
    void subscribeText(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                       QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
                       bool doNotVerify, bool leavePacked, Res<int, QString> on_msg,
                       Res<QString, QString> on_done = _nop_res_status2);

    /**
     * @brief Javascript version of subscribeText
     * @param params A map of parameters. Keys are: (1) URI, (2) PrimaryAccessChain, (3) AutoChain, (4) RoutingObjects, (5) Expiry, (6) ExpiryDelta, (7) ElaboratePAC, (8) DoNotVerify, and (9) LeavePacked
     * @param on_msg Javascript callback invoked on the arrival of each message with two arguments: (1) the payload object number, and (2) the payload as a String
     * @param on_done The callback that is executed when the subscribe process is complete. First argument is an error message, or the empty string if no error occurred. Second argument is the subscription handle.
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void subscribeText(QVariantMap params, QJSValue on_msg, QJSValue on_done);

    /**
     * @brief Unsubscribe from a resource
     * @param handle The handle obtained from the on_done callback parameter to subscribe
     * @param on_done The callback to be executed with the error message. "" implies success.
     *
     * @ingroup cpp
     * @since 1.4
     */
    void unsubscribe(QString handle, Res<QString> on_done = _nop_res_status);

    /**
     * @brief Unsubscribe from a resource
     * @param handle The handle obtained from the on_done callback parameter to subscribe
     * @param on_done The callback to be executed with the error message. "" implies success.
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void unsubscribe(QString handle, QJSValue on_done);

    /**
     * @brief Set the entity
     * @param filename a BW entity file to use
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if no error occurred, and (2) the VK
     *
     * @ingroup cpp
     * @since 1.4
     */
    void setEntityFile(QString filename, Res<QString, QString> on_done);

    /**
     * @brief Javascript version of setEntityFile.
     * @param filename The BW entity file to use
     * @param on_done Javascript callback invoked with two arguments: (1) an error message, or the empty string if no error occurred, and (2) the VK
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void setEntityFile(QString filename, QJSValue on_done);

    /**
     * @brief Set the entity
     * @param keyfile The binary contents of an entity descriptor
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if no error occurred, and (2) the VK
     *
     * Note that if read from an entity file, the first byte of the file
     * must be stripped as it is an RO type indicator.
     *
     * @see setEntityFile
     * @ingroup cpp
     * @since 1.4
     */
    void setEntity(QByteArray keyfile, Res<QString, QString> on_done);

    /**
     * @brief Set the entity
     * @param keyfile The binary contents of an entity descriptor
     * @param on_done Javascript callback invoked with two arguments: (1) an error message, or the empty string if no error occurred, and (2) the VK
     *
     * Note that if read from an entity file, the first byte of the file
     * must be stripped as it is an RO type indicator.
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void setEntity(QByteArray keyfile, QJSValue on_done);

    /**
     * @brief Set the entity by reading the file denoted by $BW2_DEFAULT_ENTITY
     * @param on_done Javascript callback invoked with two arguments: (1) an error message, or the empty string if no error occurred, and (2) the VK
     *
     * @see setEntityFile
     * @ingroup cpp
     * @since 1.4
     */
    void setEntityFromEnviron(Res<QString, QString> on_done);

    /**
     * @brief Set the entity by reading the file denoted by $BW2_DEFAULT_ENTITY
     * @param on_done Javascript callback invoked with two arguments: (1) an error message, or the empty string if no error occurred, and (2) the VK
     *
     * @see setEntityFile
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void setEntityFromEnviron(QJSValue on_done);

    /**
     * @brief Builds a DOT chain.
     * @param uri The URI to which to build the chain
     * @param permissions The permissions that the chains must provide
     * @param to The entity to which the chain is built
     * @param on_done Called for each chain that is built. Arguments are (1) error message (or empty string if none), (2) DOT chain object, and (3) boolean which is true if this is the final DOT chain
     *
     * @ingroup cpp
     * @since 1.4
     */
    void buildChain(QString uri, QString permissions, QString to,
                    Res<QString, SimpleChain*, bool> on_done);

    /**
     * @brief Builds a DOT chain.
     * @param params A map of parameters. Keys are: (1) URI, (2) Permission, amd (3) To
     * @param on_done Called for each chain that is built. Arguments are (1) error message (or empty string if none), (2) DOT chain object, and (3) boolean which is true if this is the final DOT chain
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void buildChain(QVariantMap params, QJSValue on_done);

    /**
     * @brief Builds a DOT chain and gives only the first result, if any.
     * @param uri The URI to which to build the chain
     * @param permissions The permissions that the chains must provide
     * @param to The entity to which the chain is built
     * @param on_done Callback that is called once. Arguments are (1) error message (or empty string if none), and (2) DOT chain object
     *
     * @ingroup cpp
     * @since 1.4
     */
    void buildAnyChain(QString uri, QString permissions, QString to,
                       Res<QString, SimpleChain*> on_done);

    /**
     * @brief Builds a DOT chain and gives only the first result, if any.
     * @param params A map of parameters. Keys are: (1) URI, (2) Permission, amd (3) To
     * @param on_done Callback that is called once. Arguments are (1) error message (or empty string if none), and (2) DOT chain object
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void buildAnyChain(QVariantMap params, QJSValue on_done);

    /**
     * @brief Query a resource for persisted messages
     * @param uri The resource to query
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_result Callback that is invoked multiple times. Takes three arguments: (1) error message, or the empty string if there was no error, (2) a persisted message or nullptr, (3) a boolean indicating whether all persisted messages have been delivered (in which case the callback will not be invoked again)
     *
     * @ingroup cpp
     * @since 1.4
     */
    void query(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
               QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
               bool doNotVerify, bool leavePacked,
               Res<QString, PMessage, bool> on_result);

    /**
     * @brief Query a resource for persisted MsgPack messages and decode them as MsgPack
     * @param uri The resource to query
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_result Callback that is invoked multiple times. Takes five arguments: (1) error message, or the empty string if there was no error, (2) the payload object number of a persisted message, (3) the decoded message, as a QVariantMap, (4) a boolean indicating whether a message is included in this invocation, and (5) a boolean indicating whether all persisted messages have been delivered (in which case the callback will not be invoked again)
     *
     * @ingroup cpp
     * @since 1.4
     */
    void queryMsgPack(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                      QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
                      bool doNotVerify, bool leavePacked,
                      Res<QString, int, QVariantMap, bool, bool> on_result);

    /**
     * @brief Query a resource for persisted MsgPack messages and decode them as MsgPack
     * @param params A map of parameters. Keys are: (1) URI, (2) PrimaryAccessChain, (3) AutoChain, (4) RoutingObjects, (5) Expiry, (6) ExpiryDelta, (7) ElaboratePAC, (8) DoNotVerify, and (9) LeavePacked
     * @param on_result Callback that is invoked multiple times. Takes five arguments: (1) error message, or the empty string if there was no error, (2) the payload object number of a persisted message, (3) the decoded message, as an object, (4) a boolean indicating whether a message is included in this invocation, and (5) a boolean indicating whether all persisted messages have been delivered (in which case the callback will not be invoked again)
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void queryMsgPack(QVariantMap params, QJSValue on_result);

    /**
     * @brief Query a resource for persisted text messages and decode them as text
     * @param uri The resource to query
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_result Callback that is invoked multiple times. Takes five arguments: (1) error message, or the empty string if there was no error, (2) the payload object number of a persisted message, (3) the decoded message, as a QString, (4) a boolean indicating whether a message is included in this invocation, and (5) a boolean indicating whether all persisted messages have been delivered (in which case the callback will not be invoked again)
     *
     * @ingroup cpp
     * @since 1.4
     */
    void queryText(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                   QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
                   bool doNotVerify, bool leavePacked,
                   Res<QString, int, QString, bool, bool> on_result);

    /**
     * @brief Query a resource for persisted text messages and decode them as text
     * @param params A map of parameters. Keys are: (1) URI, (2) PrimaryAccessChain, (3) AutoChain, (4) RoutingObjects, (5) Expiry, (6) ExpiryDelta, (7) ElaboratePAC, (8) DoNotVerify, and (9) LeavePacked
     * @param on_result Callback that is invoked multiple times. Takes five arguments: (1) error message, or the empty string if there was no error, (2) the payload object number of a persisted message, (3) the decoded message, as a String, (4) a boolean indicating whether a message is included in this invocation, and (5) a boolean indicating whether all persisted messages have been delivered (in which case the callback will not be invoked again)
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void queryText(QVariantMap params, QJSValue on_result);

    /**
     * @brief Query a resource for persisted messages, returning all of them as a list
     * @param uri The resource to query
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_result Callback that is invoked once. Takes two arguments: (1) error message, or the empty string if there was no error, (2) a list of persisted messages
     *
     * @ingroup cpp
     * @since 1.4
     */
    void queryList(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                   QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
                   bool doNotVerify, bool leavePacked,
                   Res<QString, QList<PMessage>> on_done);

    /**
     * @brief Query a resource for persisted messages, giving only one result
     * @param uri The resource to query
     * @param primaryAccessChain The Primary Access Chain to use
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param roz Routing objects to include in the message
     * @param expiry The time at which the message should expire (ignored if invalid)
     * @param expiryDelta The number of milliseconds after which the message should expire (ignored if negative)
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param leavePacked If true, the POs and ROs are left in the bosswave format
     * @param on_result Callback that is invoked once. Takes two arguments: (1) error message, or the empty string if there was no error, (2) a persisted message, or nullptr if none are present
     *
     * @ingroup cpp
     * @since 1.4
     */
    void queryOne(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
                  QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
                  bool doNotVerify, bool leavePacked, Res<QString, PMessage> on_done);

    /**
     * @brief Lists all immediate children of a URI that have persisted messages in their children
     * @param uri The URI whose children to list
     * @param primaryAccessChain The primary access chain
     * @param autoChain If true, the DOT chain is inferred automatically
     * @param expiry Expiry time for this message. Ignored if invalid (year == 0)
     * @param expiryDelta Milliseconds after now when this message should expire
     * @param elaboratePAC Elaboration level for the Primary Access Chain
     * @param doNotVerify If false, the router will verify this message as if it were hostile
     * @param on_done Callback that is invoked once for each URI. Takes three arguments: (1) error message, or the empty string if there was no error, (2) the child URI, and (3) a boolean indicating whether all child URIs have been delivered (in which case the callback will not be invoked again)
     *
     * @ingroup cpp
     * @since 1.4
     */
    void list(QString uri, QString primaryAccessChain, bool autoChain, QList<RoutingObject*> roz,
              QDateTime expiry, qreal expiryDelta, QString elaboratePAC,
              bool doNotVerify, Res<QString, QString, bool> on_result);

    /**
     * @brief Lists all immediate children of a URI that have persisted messages in their children
     * @param params A map of parameters. Keys are: (1) URI, (2) PrimaryAccessChain, (3) RoutingObjects, (4) Expiry, (5) ExpiryDelta, (6) ElaboratePAC, and (7) DoNotVerify
     * @param on_done Callback that is invoked once for each URI. Takes three arguments: (1) error message, or the empty string if there was no error, (2) the child URI, and (3) a boolean indicating whether all child URIs have been delivered (in which case the callback will not be invoked again)
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void list(QVariantMap params, QJSValue on_result);

    /**
     * @brief Publish a DOT, using an account to bankroll the operation
     * @param blob The DOT to publish
     * @param account The account number to use
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the hash
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishDOTWithAcc(QByteArray blob, int account, Res<QString, QString> on_done);

    /**
     * @brief Publish a DOT, using an account to bankroll the operation
     * @param blob The DOT to publish
     * @param account The account number to use
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the hash
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishDOTWithAcc(QByteArray blob, int account, QJSValue on_done);

    /**
     * @brief Like publishDOTWithAcc, but always uses account number 1
     * @param blob The DOT to publish
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the hash
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishDOT(QByteArray blob, Res<QString, QString> on_done);

    /**
     * @brief Like publishDOTWithAcc, but always uses account number 1
     * @param blob The DOT to publish
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the hash
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishDOT(QByteArray blob, QJSValue on_done);

    /**
     * @brief Publish an entity, using an account to bankroll the operation
     * @param blob The entity to publish
     * @param account The account number to use
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the VK
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishEntityWithAcc(QByteArray blob, int account, Res<QString, QString> on_done);

    /**
     * @brief Publish an entity, using an account to bankroll the operation
     * @param blob The entity to publish
     * @param account The account number to use
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the VK
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishEntityWithAcc(QByteArray blob, int account, QJSValue on_done);

    /**
     * @brief Like publishEntityWithAcc, but always uses account number 1
     * @param blob The entity to publish
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the VK
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishEntity(QByteArray blob, Res<QString, QString> on_done);

    /**
     * @brief Like publishEntityWithAcc, but always uses account number 1
     * @param blob The entity to publish
     * @param on_done Callback called with two arguments: (1) the error message (or empty string if no error), and (2) the VK
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishEntity(QByteArray blob, QJSValue on_done);

    /**
     * @brief Sets metadata published at the URI
     * @param uri The URI at which the metadata should be set
     * @param key The metadata key to set
     * @param val The value to which the key should be bound
     * @param on_done Callback invoked with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void setMetadata(QString uri, QString key, QString val, Res<QString> on_done);

    /**
     * @brief Sets metadata published at the URI
     * @param uri The URI at which the metadata should be set
     * @param key The metadata key to set
     * @param val The value to which the key should be bound
     * @param on_done Callback invoked with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void setMetadata(QString uri, QString key, QString val, QJSValue on_done);

    /**
     * @brief Deletes the metadata, for a given key, published at a URI
     * @param uri The URI from which the key should be deleted
     * @param key The metadata key to delete
     * @param on_done Callback invoked with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void delMetadata(QString uri, QString key, Res<QString> on_done);

    /**
     * @brief Deletes the metadata, for a given key, published at a URI
     * @param uri The URI from which the key should be deleted
     * @param key The metadata key to delete
     * @param on_done Callback invoked with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void delMetadata(QString uri, QString key, QJSValue on_done);

    /**
     * @brief Get all of the metadata at a URI
     * @param uri The URI at which to resolve the metadata
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) a map with the key-value pairs of that metadata, and (3) a map indicating the URI at which each key was resolved
     *
     * @ingroup cpp
     * @since 1.4
     */
    void getMetadata(QString uri, Res<QString, QMap<QString, MetadataTuple>, QMap<QString, QString>> on_done);

    /**
     * @brief Get all of the metadata at a URI
     * @param uri The URI at which to resolve the metadata
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) a map with the key-value pairs of that metadata, and (3) a map indicating the URI at which each key was resolved
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void getMetadata(QString uri, QJSValue on_done);

    /**
     * @brief Get the metadata at a URI corresponding to a single key
     * @param uri The URI at which to resolve the metadata
     * @param key The key whose value is to be resolved
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the value corresponding to that key, and (3) the URI at which the key was resolved
     *
     * @ingroup cpp
     * @since 1.4
     */
    void getMetadataKey(QString uri, QString key, Res<QString, MetadataTuple, QString> on_done);

    /**
     * @brief Get the metadata at a URI corresponding to a single key
     * @param uri The URI at which to resolve the metadata
     * @param key The key whose value is to be resolved
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the value corresponding to that key, and (3) the URI at which the key was resolved
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void getMetadataKey(QString uri, QString key, QJSValue on_done);

    /**
     * @brief Publish a DOT chain using the specified account number
     * @param blob The DOT chain as a byte array
     * @param account The acccount number to use
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the hash of the DOT chain
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishChainWithAcc(QByteArray blob, int account, Res<QString, QString> on_done);

    /**
     * @brief Publish a DOT chain using the specified account number
     * @param blob The DOT chain as a byte array
     * @param account The acccount number to use
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the hash of the DOT chain
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishChainWithAcc(QByteArray blob, int account, QJSValue on_done);

    /**
     * @brief Like publishChainWithAcc, but uses account 1
     * @param blob The DOT chain as a byte array
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the hash of the DOT chain
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishChain(QByteArray blob, Res<QString, QString> on_done);

    /**
     * @brief Like publishChainWithAcc, but uses account 1
     * @param blob The DOT chain as a byte array
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the hash of the DOT chain
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishChain(QByteArray blob, QJSValue on_done);

    /**
     * @brief Unresolves an entity to an alias
     * @param blob The entity to unresolve, as a byte array
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the alias that the entity unresolved to
     *
     * @ingroup cpp
     * @since 1.4
     */
    void unresolveAlias(QByteArray blob, Res<QString, QString> on_done);

    /**
     * @brief Unresolves an entity to an alias
     * @param blob The entity to unresolve, as a byte array
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the alias that the entity unresolved to
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void unresolveAlias(QByteArray blob, QJSValue on_done);

    /**
     * @brief Resolve a long alias to a byte array
     * @param al The long alias to resolve
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the byte array that the long alias resolved to, and (3) a boolean indicating whether the long alias was successfully resolved
     *
     * @ingroup cpp
     * @since 1.4
     */
    void resolveLongAlias(QString al, Res<QString, QByteArray, bool> on_done);

    /**
     * @brief Resolve a long alias to a byte array
     * @param al The long alias to resolve
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the byte array that the long alias resolved to, and (3) a boolean indicating whether the long alias was successfully resolved
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void resolveLongAlias(QString al, QJSValue on_done);

    /**
     * @brief Resolve a short alias to a byte array
     * @param al The short alias to resolve
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the byte array that the short alias resolved to, and (3) a boolean indicating whether the short alias was successfully resolved
     *
     * @ingroup cpp
     * @since 1.4
     */
    void resolveShortAlias(QString al, Res<QString, QByteArray, bool> on_done);

    /**
     * @brief Resolve a short alias to a byte array
     * @param al The short alias to resolve
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the byte array that the short alias resolved to, and (3) a boolean indicating whether the short alias was successfully resolved
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void resolveShortAlias(QString al, QJSValue on_done);

    /**
     * @brief Resolve an embedded alias to a string
     * @param al The embedded alias to resolve
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the data that the embedded alias resolved to
     *
     * @ingroup cpp
     * @since 1.4
     */
    void resolveEmbeddedAlias(QString al, Res<QString, QString> on_done);

    /**
     * @brief Resolve an embedded alias to a string
     * @param al The embedded alias to resolve
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the data that the embedded alias resolved to
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void resolveEmbeddedAlias(QString al, QJSValue on_done);

    /**
     * @brief Resolve a key to a Routing Object in the registry
     * @param key The key to resolve
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the Routing Object that the key resolved to, and (3) an enumeration indicating the result of the resolution
     *
     * @ingroup cpp
     * @since 1.4
     */
    void resolveRegistry(QString key, Res<QString, RoutingObject*, RegistryValidity> on_done);

    /**
     * @brief Resolve a key to a Routing Object in the registry
     * @param key The key to resolve
     * @param on_done Callback invoked with three arguments: (1) an error message, or the empty string if there was no error, (2) the Routing Object that the key resolved to, and (3) an enumeration indicating the result of the resolution
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void resolveRegistry(QString key, QJSValue on_done);

    /**
     * @brief Get the balances of the current entity's bank accounts
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the balances of this entity's bank accounts
     *
     * @ingroup cpp
     * @since 1.4
     */
    void entityBalances(Res<QString, QVector<struct balanceinfo>> on_done);

    /**
     * @brief Get the balances of the current entity's bank accounts
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the balances of this entity's bank accounts
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void entityBalances(QJSValue on_done);

    /**
     * @brief Get the balance of the bank account with the given address
     * @param addr The address of the bank account whose balance to query
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the balance of the specified bank account
     *
     * @ingroup cpp
     * @since 1.4
     */
    void addressBalance(QString addr, Res<QString, struct balanceinfo> on_done);

    /**
     * @brief Get the balance of the bank account with the given address
     * @param addr The address of the bank account whose balance to query
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the balance of the specified bank account
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void addressBalance(QString addr, QJSValue on_done);

    /**
     * @brief Get the Block Chain Interaction Parameters
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the interaction parameters
     *
     * @ingroup cpp
     * @since 1.4
     */
    void getBCInteractionParams(Res<QString, struct currbcip> on_done);

    /**
     * @brief Get the Block Chain Interaction Parameters
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the interaction parameters
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void getBCInteractionParams(QJSValue on_done);

    /**
     * @brief Sets the Block Chain Interaction Parameters
     * @param confirmations The new number of confirmations i.e., blocks to wait to consider a transaction secure (ignored if negative)
     * @param timeout The new timeout i.e., blocks to wait before giving up (ignored if negative)
     * @param maxAge The new max age i.e., maximum age of a chain, in seconds, before considering it stale (ignored if negative)
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the interaction parameters
     *
     * @ingroup cpp
     * @since 1.4
     */
    void setBCInteractionParams(int64_t confirmations, int64_t timeout, int64_t maxAge,
                                Res<QString, struct currbcip> on_done);

    /**
     * @brief Sets the Block Chain Interaction Parameters
     * @param confirmations The new number of confirmations (ignored if negative)
     * @param timeout The new timeout (ignored if negative)
     * @param maxAge The new max age (ignored if negative)
     * @param on_done Callback invoked with two arguments: (1) an error message, or the empty string if there was no error, and (2) the interaction parameters
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void setBCInteractionParams(qreal confirmations, qreal timeout, qreal maxAge,
                                            QJSValue on_done);

    /**
     * @brief Transfer Ether from this entity to another entity
     * @param from The bank account of this entity from which to tranfer Ether
     * @param to The entity to which Ether should be transferred
     * @param ether The number of Ether to transfer
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void transferEther(int from, QString to, double ether, Res<QString> on_done);

    /**
     * @brief Transfer Ether from this entity to another entity
     * @param from The bank account of this entity from which to tranfer Ether
     * @param to The entity to which Ether should be transferred
     * @param ether The number of Ether to transfer
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void transferEther(int from, QString to, double ether, QJSValue on_done);

    /**
     * @brief Make a new Designated Router Offer
     * @param account The account to use to make the offer
     * @param nsvk The namespace verifying key
     * @param dr The router to which to make the offer
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void newDesignatedRouterOffer(int account, QString nsvk, Entity* dr,
                                  Res<QString> on_done);

    /**
     * @brief Make a new Designated Router Offer
     * @param account The account to use to make the offer
     * @param nsvk The namespace verifying key
     * @param dr The router to which to make the offer
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void newDesignatedRouterOffer(int account, QString nsvk, Entity* dr,
                                              QJSValue on_done);

    /**
     * @brief Revoke a Designated Router Offer
     * @param account The account to use to revoke the offer
     * @param nsvk The namespace verifying key
     * @param dr The router to which the offer was made
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void revokeDesignatedRouterOffer(int account, QString nsvk, Entity* dr,
                                     Res<QString> on_done);

    /**
     * @brief Revoke a Designated Router Offer
     * @param account The account to use to revoke the offer
     * @param nsvk The namespace verifying key
     * @param dr The router to which the offer was made
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void revokeDesignatedRouterOffer(int account, QString nsvk, Entity* dr,
                                                 QJSValue on_done);

    /**
     * @brief Revoke acceptance of a Designated Router Offer
     * @param account The account to use to revoke the offer
     * @param drvk The verifying key of the designated router
     * @param ns The namespace
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void revokeAcceptanceOfDesignatedRouterOffer(int account, QString drvk, Entity* ns,
                                                 Res<QString> on_done);

    /**
     * @brief Revoke acceptance of a Designated Router Offer
     * @param account The account to use to revoke the offer
     * @param drvk The verifying key of the designated router
     * @param ns The namespace
     * @param on_done Callback called with a single argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void revokeAcceptanceOfDesignatedRouterOffer(int account, QString drvk,
                                                             Entity* ns, QJSValue on_done);

    /**
     * @brief Revoke the entity with the specified VK
     * @param vk The verifying key
     * @param on_done Callback called with three arguments: (1) an error message, or the empty string if there was no error, (2) the hash, and (3) the contents of the Payload Object
     *
     * @ingroup cpp
     * @since 1.4
     */
    void revokeEntity(QString vk, Res<QString, QString, QByteArray> on_done);

    /**
     * @brief Revoke the entity with the specified VK
     * @param vk The verifying key
     * @param on_done Callback called with three arguments: (1) an error message, or the empty string if there was no error, (2) the hash, and (3) the contents of the Payload Object
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void revokeEntity(QString vk, QJSValue on_done);

    /**
     * @brief Revoke the DOT with the specified hash
     * @param hash The hash of the dot to revoke
     * @param on_done Callback called with three arguments: (1) an error message, or the empty string if there was no error, (2) the hash, and (3) the contents of the Payload Object
     *
     * @ingroup cpp
     * @since 1.4
     */
    void revokeDOT(QString hash, Res<QString, QString, QByteArray> on_done);

    /**
     * @brief Revoke the DOT with the specified hash
     * @param hash The hash of the dot to revoke
     * @param on_done Callback called with three arguments: (1) an error message, or the empty string if there was no error, (2) the hash, and (3) the contents of the Payload Object
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void revokeDOT(QString hash, QJSValue on_done);

    /**
     * @brief Publishes a revocation
     * @param account The account to use
     * @param blob The revocation to publish
     * @param on_done Callback called with two arguments: (1) an error message, or the empty string if there was no error, and (2) the hash
     *
     * @ingroup cpp
     * @since 1.4
     */
    void publishRevocation(int account, QByteArray blob, Res<QString, QString> on_done);

    /**
     * @brief Publishes a revocation
     * @param account The account to use
     * @param blob The revocation to publish
     * @param on_done Callback called with two arguments: (1) an error message, or the empty string if there was no error, and (2) the hash
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void publishRevocation(int account, QByteArray blob, QJSValue on_done);

    /**
     * @brief Get the designated router offers
     * @param nsvk The namespace verifying key
     * @param on_done Callback invoked with four arguments: (1) an error message, or the empty string if there was no error, (2) the contents of the active header, (3) the contents of the srv header, and (4) the offers, as a list of designated router verifying keys
     *
     * @ingroup cpp
     * @since 1.4
     */
    void getDesignatedRouterOffers(QString nsvk, Res<QString, QString, QString, QList<QString>> on_done);

    /**
     * @brief Get the designated router offers
     * @param nsvk The namespace verifying key
     * @param on_done Callback invoked with four arguments: (1) an error message, or the empty string if there was no error, (2) the contents of the active header, (3) the contents of the srv header, and (4) the offers, as a list of designated router verifying keys
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void getDesignatedRouterOffers(QString nsvk, QJSValue on_done);

    /**
     * @brief Accept a designated router offer
     * @param account The account number to use
     * @param drvk The designated router verifying key to use
     * @param ns The namespace
     * @param on_done Callback invoked with one argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void acceptDesignatedRouterOffer(int account, QString drvk, Entity* ns, Res<QString> on_done);

    /**
     * @brief Accept a designated router offer
     * @param account The account number to use
     * @param drvk The designated router verifying key to use
     * @param ns The namespace
     * @param on_done Callback invoked with one argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void acceptDesignatedRouterOffer(int account, QString drvk, Entity* ns,
                                                 QJSValue on_done);

    /**
     * @brief Sets the designated router SRV record
     * @param account The account number to use
     * @param srv The SRV record
     * @param dr The designated router
     * @param on_done Callback invoked with one argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void setDesignatedRouterSRVRecord(int account, QString srv, Entity* dr, Res<QString> on_done);

    /**
     * @brief Sets the designated router SRV record
     * @param account The account number to use
     * @param srv The SRV record
     * @param dr The designated router
     * @param on_done Callback invoked with one argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void setDesignatedRouterSRVRecord(int account, QString srv, Entity* dr,
                                                  QJSValue on_done);

    /**
     * @brief Creates a long alias
     * @param account The account number to use
     * @param key The new alias (at most 32 bytes)
     * @param val The value that the alias resolves to (at most 32 bytes)
     * @param on_done Callback invoked with one argument: an error message, or the empty string if there was no error
     *
     * @ingroup cpp
     * @since 1.4
     */
    void createLongAlias(int account, QByteArray key, QByteArray val, Res<QString> on_done);

    /**
     * @brief Creates a long alias
     * @param account The account number to use
     * @param key The new alias (at most 32 bytes)
     * @param val The value that the alias resolves to (at most 32 bytes)
     * @param on_done Callback invoked with one argument: an error message, or the empty string if there was no error
     *
     * @ingroup qml
     * @since 1.4
     */
    Q_INVOKABLE void createLongAlias(int account, QByteArray key, QByteArray val,
                                     QJSValue on_done);

    /**
     * @brief Get the current entity's verifying key
     * @return The base64 version of the VK
     *
     * @ingroup qml
     * @since 1.1.7
     */
    Q_INVOKABLE QString getVK();

    /**
     * @brief Create a new BOSSWAVE View
     * @param query The view expression
     * @param on_done A callback to be executed with an error message (or nil) and the View
     *
     * @see https://github.com/immesys/bw2/wiki/Views
     * @ingroup cpp
     * @since 1.2
     */
    Q_INVOKABLE void createView(QVariantMap query, Res<QString, BWView*> on_done);

    /**
     * @brief Create a new BOSSWAVE View
     * @param query The view expression
     * @param on_done A callback to be executed with an error message (or nil) and the View
     *
     * @see https://github.com/immesys/bw2/wiki/Views
     * @ingroup qml
     * @since 1.2
     */
    Q_INVOKABLE void createView(QVariantMap query, QJSValue on_done);

signals:
    /**
     * @brief Fired when the BOSSWAVE agent connection changes (connect or disconnect)
     * @param success True if ok, false otherwise
     * @param msg Empty if ok, error message otherwise
     */
    void agentChanged(bool success, QString msg);

private:
    QQmlEngine *engine;
    QJSEngine *jsengine;
    AgentConnection *agent();
    AgentConnection *m_agent;
    QString m_vk;

    template <typename ...Tz> Res<Tz...> ERes(QJSValue callback)
    {
        return Res<Tz...>(jsengine, callback);
    }

    static Res<QString> _nop_res_status;
    static Res<QString, QString> _nop_res_status2;
    friend BWView;
};

/**
 * @brief A Metadata Tuple, representing the value that corresponds to a key
 *
 * @ingroup qml
 * @since 1.4
 */
class MetadataTupleJS : public QObject {
    Q_OBJECT
    Q_PROPERTY(QString value MEMBER value)
    Q_PROPERTY(QDateTime time MEMBER time)

public:
    MetadataTupleJS(QObject* parent = nullptr) : QObject(parent) {}

    MetadataTupleJS(QString val, int64_t ts, QObject* parent = nullptr)
        : QObject(parent), value(val)
    {
        int64_t msecs = ts / 1000000;
        int64_t nsecs = ts % 1000000;
        if (nsecs < 0)
        {
            msecs -= 1;
            nsecs += 1000000;
        }

        /* Round to the nearest millisecond. */
        if (nsecs >= 500000)
        {
            msecs += 1;
        }
        time = QDateTime::fromMSecsSinceEpoch(msecs);
    }

    /**
     * @brief The value corresponding to a key
     */
    QString value;

    /**
     * @brief The timestamp for this value
     */
    QDateTime time;
};

/**
 * @brief A Metadata Tuple, representing the value that corresponds to a key
 *
 * @ingroup cpp
 * @since 1.4
 */
class MetadataTuple
{
public:
    MetadataTuple()
        : value(), timestamp(0) {}

    MetadataTuple(QString val, int64_t ts)
        : value(val), timestamp(ts) {}

    MetadataTuple(const QVariantMap& metadata)
        : value(metadata["val"].toString()), timestamp(metadata["ts"].toLongLong()) {}

    QVariantMap toVariantMap()
    {
        QVariantMap metadata;
        metadata["val"] = this->value;
        metadata["ts"] = QVariant::fromValue((qlonglong) this->timestamp);
        return metadata;
    }

    MetadataTuple& operator=(const MetadataTuple& other)
    {
        this->value = other.value;
        this->timestamp = other.timestamp;
        return *this;
    }

    /**
     * @brief The value corresponding to a key
     */
    QString value;

    /**
     * @brief The timestamp for this value
     */
    int64_t timestamp;
};


/**
 * @brief The Current Block Chain Interaction Parameters
 *
 * @ingroup cpp
 * @since 1.4
 */
struct currbcip
{
    /**
     * @brief The number of blocks to wait after a transaction to be sure that it is secure
     */
    int64_t confirmations;

    /**
     * @brief The number of blocks to wait before giving up
     */
    int64_t timeout;

    /**
     * @brief How many seconds old the chain can be before it is considered stale
     */
    int64_t maxAge;

    /**
     * @brief The current age of the chain, in seconds
     */
    int64_t currentAge;

    /**
     * @brief The current block number
     */
    uint64_t currentBlock;

    /**
     * @brief The number of peers we are connected to
     */
    int64_t peers;

    /**
     * @brief The number of the highest block we are aware of
     */
    int64_t highestBlock;

    /**
     * @brief The current difficulty of mining a block
     */
    int64_t difficulty;
};

/**
 * @brief The BlockChainInteractionParams class
 *
 * @ingroup qml
 * @since 1.4
 */
class BlockChainInteractionParams : public QObject
{
    Q_OBJECT
    Q_PROPERTY(qreal confirmation MEMBER confirmations)
    Q_PROPERTY(qreal timeout MEMBER timeout)
    Q_PROPERTY(qreal maxAge MEMBER maxAge)
    Q_PROPERTY(qreal currentAge MEMBER currentAge)
    Q_PROPERTY(qreal currentBlock MEMBER currentBlock)
    Q_PROPERTY(qreal peers MEMBER peers)
    Q_PROPERTY(qreal highestBlock MEMBER highestBlock)
    Q_PROPERTY(qreal difficulty MEMBER difficulty)

public:
    BlockChainInteractionParams(QObject* parent = nullptr) : QObject(parent),
        confirmations(0), timeout(0), maxAge(0), currentAge(0), currentBlock(0),
        peers(0), highestBlock(0), difficulty(0) {}
    BlockChainInteractionParams(const struct currbcip& cbcip, QObject* parent = nullptr)
        : QObject(parent), confirmations(cbcip.confirmations), timeout(cbcip.timeout),
        maxAge(cbcip.maxAge), currentAge(cbcip.currentAge), currentBlock(cbcip.currentBlock),
        peers(cbcip.peers), highestBlock(cbcip.highestBlock), difficulty(cbcip.difficulty) {}

    /**
     * @brief The number of blocks to wait after a transaction to be sure that it is secure
     */
    int64_t confirmations;

    /**
     * @brief The number of blocks to wait before giving up
     */
    int64_t timeout;

    /**
     * @brief How many seconds old the chain can be before it is considered stale
     */
    int64_t maxAge;

    /**
     * @brief The current age of the chain, in seconds
     */
    int64_t currentAge;

    /**
     * @brief The current block number
     */
    uint64_t currentBlock;

    /**
     * @brief The number of peers we are connected to
     */
    int64_t peers;

    /**
     * @brief The number of the highest block we are aware of
     */
    int64_t highestBlock;

    /**
     * @brief The current difficulty of mining a block
     */
    int64_t difficulty;
};

/**
 * @brief The balance of a bank account
 *
 * @ingroup cpp
 * @since 1.4
 */
struct balanceinfo
{
    /**
     * @brief Address of the bank account
     */
    QString addr;

    /**
     * @brief Human readable balance (imprecise)
     */
    QString human;

    /**
     * @brief The balance, as a decimal string
     */
    QString decimal;

    /**
     * @brief The balance as a real number
     */
    qreal value;
};

/**
 * @brief The balance of a bank account
 *
 * @ingroup qml
 * @since 1.4
 */
class BalanceInfo : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString addr MEMBER addr)
    Q_PROPERTY(QString human MEMBER human)
    Q_PROPERTY(QString decimal MEMBER decimal)
    Q_PROPERTY(qreal value MEMBER value)

public:
    BalanceInfo(QObject* parent = nullptr) : QObject(parent), value(0.0) {}
    BalanceInfo(const struct balanceinfo& bi, QObject* parent = nullptr)
        : QObject(parent), addr(bi.addr), human(bi.human), decimal(bi.decimal),
        value(bi.value) {}

    /**
     * @brief Address of the bank account
     */
    QString addr;

    /**
     * @brief Human readable balance (imprecise)
     */
    QString human;

    /**
     * @brief The balance, as a decimal string
     */
    QString decimal;

    /**
     * @brief The balance as a real number
     */
    qreal value;
};

/**
 * @brief A DOT Chain
 *
 * @ingroup cpp
 * @ingroup qml
 * @since 1.4
 */
class SimpleChain : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString hash MEMBER hash)
    Q_PROPERTY(QString permissions MEMBER permissions)
    Q_PROPERTY(QString uri MEMBER uri)
    Q_PROPERTY(QString to MEMBER to)
    Q_PROPERTY(QByteArray content MEMBER content)
    Q_PROPERTY(bool valid MEMBER valid)

public:
    /**
     * @brief The hash of this DOT Chain
     */
    QString hash;

    /**
     * @brief The permissions granted by the DOT Chain
     */
    QString permissions;

    /**
     * @brief The URI on which this DOT chain grants permissions
     */
    QString uri;

    /**
     * @brief The Entity to which permissions are granted
     */
    QString to;

    /**
     * @brief The binary contents of this DOT Chain
     */
    QByteArray content;

    /**
     * @brief A boolean indicating if this DOT Chain is valid
     */
    bool valid;
};

/**
 * @brief A BOSSWAVE View
 *
 * @ingroup cpp
 * @ingroup qml
 * @since 1.1
 */
class BWView : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList interfaces READ interfaces NOTIFY interfacesChanged)
    Q_PROPERTY(QStringList services READ services NOTIFY servicesChanged)

public:
    BWView(BW* parent) : QObject(parent), bw(parent)
    {
        Q_ASSERT(this->thread() == QCoreApplication::instance()->thread());
    }
    const QStringList& services();
    const QVariantList& interfaces();
signals:
    void interfacesChanged();
    void servicesChanged();
private:
    BW* bw;
    int m_vid;
    QVariantList m_interfaces;
    QStringList m_services;
    void onChange();
    friend BW;
};

#endif // QTLIBBW_BOSSWAVE_H


