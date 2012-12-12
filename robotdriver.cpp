#include "RobotDriver.h"

RobotDriver::RobotDriver(const QString & portName)
{
    this->port_  = new QextSerialPort(portName, QextSerialPort::EventDriven);
    this->timer_ = new QTimer(this);

    port_->setBaudRate(BAUD9600); //PLS modify the configuration based on our case.
    port_->setFlowControl(FLOW_OFF);//PLS refer to: http://blog.csdn.net/free2011/article/details/5999325
    port_->setParity(PAR_NONE);
    port_->setDataBits(DATA_8);
    port_->setStopBits(STOP_2);
    resendCounter_ = 0;
    newIdx_ = 0;

    if (port_->open(QIODevice::ReadWrite) == true)
    {
        connect(port_, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
        connect(port_, SIGNAL(dsrChanged(bool)), this, SLOT(onDsrChanged(bool)));//this may make no sense¡£
        connect(timer_, SIGNAL(timeout()), this, SLOT(resend()));
        if (!(port_->lineStatus() & LS_DSR))
        {
            qCritical() << "The serial port is not turned on, the driver is down..." ;
            exit(EXIT_FAILURE);
        }
        qDebug() << "Listening gust's msg on " << port_->portName().toAscii();
    }
    else
    {
       qCritical() << "Serial port failed to open:" << port_->errorString().toAscii();
       exit(EXIT_FAILURE);
    }
}

RobotDriver::~RobotDriver()
{

}

void RobotDriver::sendCommand(const char &comType)
{
    //Check if comType is the one predefined.
    if (comType != STOP          &&
        comType != MOVE_FORWORD  &&
        comType != MOVE_BACKWORD &&
        comType != TURN_RIGHT    &&
        comType != TURN_LEFT)
    {
        qCritical()<< "RobotDriver::sendCommand error: the entry "
                   << int(comType) << " is invalid!";
    }

    QByteArray cmd;
    cmd.append(CMD_HEADER);
    cmd.append(CMD_VERSION);
    cmd.append(GUEST_ADD);
    cmd.append(comType);
    cmd.append(CSF_SEND_CMD);
    cmd.append(char(0x00));
    char checksum = CMD_VERSION + GUEST_ADD + comType + CSF_SEND_CMD;
    cmd.append(checksum);
    cmd.append(CMD_TAIL);

    cmdBackup_ = cmd;
    port_->write(cmd);
    timer_->start(TIMEOUT_NO_REPLY_MS);
}

void RobotDriver::sendCommand(const char *cmd, const char &cmdLen)
{
    //TODO: exception detection, to ensure the command is reasonable.
    cmdBackup_ = QByteArray(cmd, cmdLen);
    port_ -> write(cmd, cmdLen);
    timer_->start(TIMEOUT_NO_REPLY_MS);
}

void RobotDriver::sendCommand(const QByteArray & cmdArray)
{
    cmdBackup_ = QByteArray(cmdArray);
    port_ -> write(cmdArray);
    timer_->start(TIMEOUT_NO_REPLY_MS);
}

void RobotDriver::onReadyRead()
{
    int len = port_->bytesAvailable();
    int sz  = len + newIdx_;
    replyData_.resize(sz);
    port_->read(replyData_.data()+newIdx_, len);
    qDebug() << len << " bytes received:"
             << replyData_;
    /*TODO: display replyData_ as ASCII,
    so it is always readable.*/
    newIdx_ = 0;

    //Start package parsing.
    const char rep[] = {CMD_HEADER, CMD_VERSION, PC_ADD};
    QByteArray sub2 = QByteArray::fromRawData(rep, 2);
    QByteArray sub3 = QByteArray::fromRawData(rep, 3);

    int startPart;
    int findSub3 = 0;
    int n3 = replyData_.count(sub3);
    int paraLen;
    int i3;

    if (n3 == 0)
    {
        qDebug() << "\'Header+version+address\' isn't contained.";
        int i2 = replyData_.lastIndexOf(sub2);
        if ( i2 != -1 )
        {
            qDebug() << "\'Header+version\' is contained.";
            if (sz - i2 > 2)
            {
               qDebug() << "Invalid data follows. Give up!";
               return;
            }
            startPart = i2;
            qDebug() << "A potential msg is partly catched,"\
                        " Waiting for next pack...";
        }
        else
        {
            qDebug() << "\'Header+version\' isn't' contained.";
            int i1 = replyData_.lastIndexOf(CMD_HEADER);
            if( i1 == -1 )
            {
                qDebug() << "\'Header\' isn't contained. Give up!";
                return;
            }

            qDebug() << "\'Header\' is contained";
            if (sz - i1 > 1)
            {
                qDebug() << "Invalid data follows. Give up";;
                return;
            }
            startPart = i1;
            qDebug() << "A potential msg is partly catched,"\
                        " Waiting for next pack...";
        }
    }
    else
    {
        qDebug() << "\'Header+version+address\' is contained.";
        while( n3-- )
        {
            i3 = replyData_.indexOf(sub3, findSub3);
            if (sz - i3 < 6)
            {
                startPart = i3;
                qDebug() << "The length is less than 6, "\
                            "\'data length\' is unavailable. "\
                            "Waiting for next pack...";
                break;
            }
            qDebug() << "\'data length\' is available.";
            paraLen = replyData_[i3 + 5];
            if ( i3 + paraLen + 8 > sz )
            {
                startPart = i3;
                qDebug() << "Too short to construct a msg."\
                            " Waiting for next pack...";
                break;
            }
            qDebug() << "The pack is long enough.";
            startPart = sz;
            if ( replyData_[i3 + paraLen + 7] != (char)CMD_TAIL )
            {
                findSub3 = i3 + 1;
                qDebug() << "Msg tail doesn't coincide.";
            }
            else
            {
                char checksum = 0;
                for (int t = i3+1; t < i3+paraLen+6; t++)
                    checksum = checksum + replyData_[t];
                if ( replyData_[i3+paraLen+6] != checksum )
                {
                    qDebug() << "Checksum doesn't coincide.";
                    findSub3 = i3 + 1;

                }
                else
                {
                    qDebug() << "Everything is fine. Start parsing...";
                    if ( replyData_[i3 + 4] == (char)RSS_INCORRECT )
                    {
                        qDebug() << "Guest received a polluted cmd,"\
                                    " Resending now...";
                        resendCounter_ = 0;
                        timer_->start(TIMEOUT_NO_REPLY_MS);
                    /*NOTE: only resend the last cmd here, it might
                     be problematic if error occured in more than one cmds consecutively.*/
                    }
                    else if ( replyData_[i3 + 4] == (char)RSS_CORRECT )
                    {
                        timer_->stop();
                        qDebug() << "Congratulations on a great correspondence!";
                        if ( replyData_[i3 + 3] == (char)RSF_MOTION_LOG )
                        {
                            if (paraLen%2)
                                qDebug() << "Something must be wrong, paraLen is not even number!";

                            QDateTime time = QDateTime::currentDateTime ();
                            qDebug() << "Time stamp: " << time.toString();

                            Motion newMot;
                            newMot.timeStamp = time;

                            for (int p = 0; p < paraLen / 2; p++ )
                            {
//                               char m1 = replyData_[i3+6+p*2]; qDebug() << m1;
//                               int m2 = (int)m1 << 8; qDebug() << m2;
//                               int m3 = (int)replyData_[i3+7+p*2] & 0xFF; qDebug() << m3; //64bit ffffffff8c
//                               int m4 = m2 | m3; qDebug() << m4;
//                               int mo = ((int)replyData_[i3+6+p*2] << 8) | ((int)replyData_[i3+7+p*2] & 0xFF);
                               int mo = ((int)replyData_[i3+6+p*2] << 8) | (unsigned char)replyData_[i3+7+p*2];

                               if (p%2)
                               {
                                   newMot.rotation = mo;
                                   qDebug() << "Rotation: " << mo  << "/10 degree";
                               }
                               else
                               {
                                   newMot.translation = mo;
                                   qDebug() << "Translation: " << mo  << " cm";
                               }
                            }
                            motionLog_.enqueue(newMot);
                            emit motionComes(newMot);
                        }
                        else
                            qDebug() << "TO IMPLEMENT: other RSF option.";
                    }
                }

            }

        }

    }


    /*The left data might construct a potential command with
    part of the next frame data, so following trick moves
    them to the head of the buffer.*/
    if ( startPart == sz )
        return;

    QByteArray partData = replyData_.mid(startPart);
    replyData_.replace(0, sz, partData);
    newIdx_ = partData.size();

}

void RobotDriver::onDsrChanged(bool status)
{
    if (status)
        qDebug() << "device was turned on" << endl;
    else
        qDebug() << "device was turned off" << endl;
}

void RobotDriver::resend()
{
    if (resendCounter_ < TIMES_RESEND_IF_NO_REPLY)
    {
        resendCounter_++;
        port_-> write(cmdBackup_);
        qDebug() << "Resend command due to timeout: "
                 << TIMEOUT_NO_REPLY_MS
                 << " microseconds!";
    }
    else
    {
        timer_->stop();
        resendCounter_ = 0;
        qDebug() << "The command has been resent "
                 << TIMES_RESEND_IF_NO_REPLY
                 << " times, but guest replied nothing. "\
                    "Is guest alive?";
    }
}

RobotDriver::Motion &RobotDriver::getMotion()
{
    Motion mo;
    if( !motionLog_.isEmpty() )
    {
        mo = motionLog_.dequeue();
        return mo;
    }

    mo.timeStamp = QDateTime::fromString("0", "s");
    mo.translation = 0;
    mo.rotation = 0;
    return mo;
}


