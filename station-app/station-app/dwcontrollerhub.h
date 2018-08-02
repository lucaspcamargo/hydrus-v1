#ifndef DWCONTROLLERHUB_H
#define DWCONTROLLERHUB_H

#include <QObject>
#include <QVector>

#include "dwcontrollerstate.h"

class dwControllerHub : public QObject
{
    Q_OBJECT
public:
    explicit dwControllerHub(QObject *parent = 0);
    virtual ~dwControllerHub();



    class ControllerEntry
    {
    public:
        ControllerEntry(){
            qDebug("CE");
            controller = joystick = haptic = 0;
            state = 0;
            plugged = true;
        }

        ~ControllerEntry()
        {
        }

        void *controller;
        void *joystick;
        void *haptic;

        bool plugged;

        dwControllerState *state;

    };

signals:
    void controllerConnected(int i);
    void controllerDisconnected(int i);

public slots:
   int controllerCount(){return m_controllers.size();}

   bool controllerIsConnected(int i){return m_controllers[i].plugged;}
   dwControllerState* controllerState(int i){return m_controllers[i].state;}

   void update();

   bool getControllerUp(int i) { return m_controllers[i].state->Up; }
   bool getControllerDown(int i) { return m_controllers[i].state->Down; }
   bool getControllerLeft(int i) { return m_controllers[i].state->Left; }
   bool getControllerRight(int i) { return m_controllers[i].state->Right; }
   bool getControllerStart(int i) { return m_controllers[i].state->Start; }
   bool getControllerBack(int i) { return m_controllers[i].state->Back; }
   bool getControllerLeftShoulder(int i) { return m_controllers[i].state->LeftShoulder; }
   bool getControllerRightShoulder(int i) { return m_controllers[i].state->RightShoulder; }
   bool getControllerAButton(int i) { return m_controllers[i].state->AButton; }
   bool getControllerBButton(int i) { return m_controllers[i].state->BButton; }
   bool getControllerXButton(int i) { return m_controllers[i].state->XButton; }
   bool getControllerYButton(int i) { return m_controllers[i].state->YButton; }

   qreal getControllerStickX(int i) { return m_controllers[i].state->StickX; }
   qreal getControllerStickY(int i) { return m_controllers[i].state->StickY; }

   bool getControllerPrUp(int i) { return m_controllers[i].state->prUp; }
   bool getControllerPrDown(int i) { return m_controllers[i].state->prDown; }
   bool getControllerPrLeft(int i) { return m_controllers[i].state->prLeft; }
   bool getControllerPrRight(int i) { return m_controllers[i].state->prRight; }
   bool getControllerPrStart(int i) { return m_controllers[i].state->prStart; }
   bool getControllerPrBack(int i) { return m_controllers[i].state->prBack; }
   bool getControllerPrLeftShoulder(int i) { return m_controllers[i].state->prLeftShoulder; }
   bool getControllerPrRightShoulder(int i) { return m_controllers[i].state->prRightShoulder; }
   bool getControllerPrAButton(int i) { return m_controllers[i].state->prAButton; }
   bool getControllerPrBButton(int i) { return m_controllers[i].state->prBButton; }
   bool getControllerPrXButton(int i) { return m_controllers[i].state->prXButton; }
   bool getControllerPrYButton(int i) { return m_controllers[i].state->prYButton; }

   qreal getControllerPrStickX(int i) { return m_controllers[i].state->prStickX; }
   qreal getControllerPrStickY(int i) { return m_controllers[i].state->prStickY; }

private:
#ifdef DW_USE_SDL2
   void addSDLController(int index);
#endif
   QVector<ControllerEntry> m_controllers;
};

#endif // DWCONTROLLERHUB_H
