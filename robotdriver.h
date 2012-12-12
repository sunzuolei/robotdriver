#ifndef ROBOTDRIVER_H
#define ROBOTDRIVER_H

#include <QObject>

#include <QEvent>
#include "qextserialport.h"
#include <QTimer>
#include <QtDebug>
#include <QByteArray>
#include <QDateTime>
#include <QQueue>

#define TIMES_RESEND_IF_NO_REPLY 3
#define TIMEOUT_NO_REPLY_MS      2000
//for command type:
#define STOP            0x00
#define MOVE_FORWORD    0x01
#define MOVE_BACKWORD   0x02
#define TURN_RIGHT      0x03
#define TURN_LEFT       0x04
//for command reconstruction:
#define CMD_HEADER      0x7E
#define CMD_VERSION     0x21
#define GUEST_ADD       0x01
#define PC_ADD          0x00
#define CSF_SEND_CMD    0xFF
#define CMD_TAIL        0x0D
//for data parsing:
#define MIN_CMD_LEN     8
//for reply data parsing
#define RSS_CORRECT     0x00
#define RSS_INCORRECT   0x03
#define RSF_MOTION_LOG  0x40

class RobotDriver : public QObject
{
Q_OBJECT
public:
    struct Motion
    {
        QDateTime timeStamp;
        int translation;
        int rotation;
    };

    RobotDriver(const QString & portName);
    ~RobotDriver();
    void sendCommand(const char &cmdType);
    void sendCommand(const char *cmd, const char &cmdLen);
    void sendCommand(const QByteArray &cmdArray);
    Motion& getMotion();


private:
    QextSerialPort *port_;
    QTimer *timer_;
    QByteArray cmdBackup_;
    char resendCounter_;
    unsigned int newIdx_;
    QByteArray replyData_;
    QQueue<Motion> motionLog_;

private slots:
    void onReadyRead();
    void onDsrChanged(bool status);
    void resend();

signals:
    void motionComes(Motion newMot);

};

#endif // ROBOTDRIVER_H
