#include "dwcontrollerstate.h"

dwControllerState::dwControllerState(QObject *parent) : QObject(parent)
{
    Up = Down = Left = Right = Start = Back = LeftShoulder =
            RightShoulder = AButton = BButton = XButton = YButton = false;
    StickX = StickY = 0.0f;
}

void dwControllerState::saveStateToPrevious()
{
    prUp = Up;
    prDown = Down;
    prLeft = Left;
    prRight = Right;
    prStart = Start;
    prBack = Back;
    prLeftShoulder = LeftShoulder;
    prRightShoulder = RightShoulder;
    prAButton = AButton;
    prBButton = BButton;
    prXButton = XButton;
    prYButton = YButton;
}

