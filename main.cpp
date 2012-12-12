/*The file shows how to use the RobotDriver class.*/

#include <QtCore/QCoreApplication>
#include "robotdriver.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    RobotDriver driver("COM1");

    /*Three options for sending command:*/
    // 1st
    driver.sendCommand(MOVE_FORWORD);

    // 2nd
    char cmd[] = {0x7E, 0x21, 0x01, 0x01, 0xFF, 0x00, 0x22, 0x0D};
    driver.sendCommand(cmd, sizeof(cmd)/sizeof(char));

    // 3rd
    QByteArray cmdArray(cmd, 8);
    driver.sendCommand(cmdArray);

    return a.exec();
}
