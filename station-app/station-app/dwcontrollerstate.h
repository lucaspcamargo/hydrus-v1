#ifndef DWCONTROLLERSTATE_H
#define DWCONTROLLERSTATE_H

#include <QObject>

class dwControllerState : public QObject
{
    Q_OBJECT
public:
    explicit dwControllerState(QObject *parent = 0);

    bool Up;
    bool Down;
    bool Left;
    bool Right;
    bool Start;
    bool Back;
    bool LeftShoulder;
    bool RightShoulder;
    bool AButton;
    bool BButton;
    bool XButton;
    bool YButton;

    qreal StickX;
    qreal StickY;


    // previous state

    void saveStateToPrevious();

    bool prUp;
    bool prDown;
    bool prLeft;
    bool prRight;
    bool prStart;
    bool prBack;
    bool prLeftShoulder;
    bool prRightShoulder;
    bool prAButton;
    bool prBButton;
    bool prXButton;
    bool prYButton;

    qreal prStickX;
    qreal prStickY;

signals:


public slots:
    bool getUp() { return Up; }
    bool getDown() { return Down; }
    bool getLeft() { return Left; }
    bool getRight() { return Right; }
    bool getStart() { return Start; }
    bool getBack() { return Back; }
    bool getLeftShoulder() { return LeftShoulder; }
    bool getRightShoulder() { return RightShoulder; }
    bool getAButton() { return AButton; }
    bool getBButton() { return BButton; }
    bool getXButton() { return XButton; }
    bool getYButton() { return YButton; }

    qreal getStickX() { return StickX; }
    qreal getStickY() { return StickY; }

    // previous state
    bool getPrUp() { return prUp; }
    bool getPrDown() { return prDown; }
    bool getPrLeft() { return prLeft; }
    bool getPrRight() { return prRight; }
    bool getPrStart() { return prStart; }
    bool getPrBack() { return prBack; }
    bool getPrLeftShoulder() { return prLeftShoulder; }
    bool getPrRightShoulder() { return prRightShoulder; }
    bool getPrAButton() { return prAButton; }
    bool getPrBButton() { return prBButton; }
    bool getPrXButton() { return prXButton; }
    bool getPrYButton() { return prYButton; }

    qreal getPrStickX() { return prStickX; }
    qreal getPrStickY() { return prStickY; }
};

#endif // DWCONTROLLERSTATE_H
