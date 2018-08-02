#include "dwcontrollerhub.h"

#ifdef DW_USE_SDL2
#include "SDL.h"
#endif

dwControllerHub::dwControllerHub(QObject *parent) : QObject(parent)
{
    qDebug("[dwControllerHub] Initializing...");

#ifdef DW_USE_SDL2
    SDL_Init(SDL_INIT_GAMECONTROLLER);

    SDL_GameControllerEventState(SDL_ENABLE);
/*
    const char * mappingFilename = "thirdparty/SDL_GameControllerDB/gamecontrollerdb.txt";
    SDL_RWops * mappingRW = SDL_RWFromFile(mappingFilename, "rb");
    int mapC = 0;
    if(mappingRW && (mapC = SDL_GameControllerAddMappingsFromRW(mappingRW, 1)) != -1)
    {
        qDebug("[dwControllerHub] Read %d controller mappings from [%s]", mapC, mappingFilename);
    }
    else qDebug("[dwControllerHub] Failed to read controller mappings from [%s]", mappingFilename);
*/

    int numJoy = SDL_NumJoysticks();

    for(int joyIndex=0; joyIndex < numJoy; ++joyIndex)
    {
        if (!SDL_IsGameController(joyIndex))
        {
            continue;
        }

        addSDLController(joyIndex);
    }

#endif
}

dwControllerHub::~dwControllerHub()
{
    qDebug("[dwControllerHub] Exiting...");
#ifdef DW_USE_SDL2
    SDL_Quit();
#endif
}

void dwControllerHub::update()
{
#ifdef DW_USE_SDL2

    SDL_GameControllerUpdate();


    SDL_Event test_event;

    while (SDL_PollEvent(&test_event)) {
        switch (test_event.type) {
            case SDL_CONTROLLERDEVICEADDED:
                addSDLController(test_event.cdevice.which);
                break;

            default:
                break;
        }
    }

    for(int i = 0; i < m_controllers.size(); i++) {

        ControllerEntry entry = m_controllers[i];

        if(entry.controller && SDL_GameControllerGetAttached((SDL_GameController*) entry.controller))
        {
            SDL_GameController *c = (SDL_GameController*) entry.controller;
            dwControllerState *cs = entry.state;

            if(!entry.plugged)
                emit controllerConnected(i);
            entry.plugged = true;

            cs->saveStateToPrevious();

            cs->Up = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_DPAD_UP);
            cs->Down = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
            cs->Left = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
            cs->Right = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
            cs->Start = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_START);
            cs->Back = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_BACK);
            cs->LeftShoulder = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
            cs->RightShoulder = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
            cs->AButton = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_A);
            cs->BButton = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_B);
            cs->XButton = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_X);
            cs->YButton = SDL_GameControllerGetButton(c, SDL_CONTROLLER_BUTTON_Y);

            cs->StickX = SDL_GameControllerGetAxis(c, SDL_CONTROLLER_AXIS_LEFTX) / 32767.0f;
            cs->StickY = - SDL_GameControllerGetAxis(c, SDL_CONTROLLER_AXIS_LEFTY) / 32767.0f;
        }
        else
        {
            if(entry.plugged)
                emit controllerDisconnected(i);
            entry.plugged = false;
        }

        m_controllers[i] = entry;
    }
#endif
}

#ifdef DW_USE_SDL2
void dwControllerHub::addSDLController(int joyIndex)
{
    SDL_GameController * controller = SDL_GameControllerOpen(joyIndex);

    if(controller)
    {
        ControllerEntry entry;
        entry.state = new dwControllerState();

        entry.controller = controller;
        SDL_Joystick * joy = SDL_GameControllerGetJoystick(controller);
        entry.joystick = joy;

        char guid[33];
        SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(joy), guid, 33);
        qDebug("[dwControllerHub] Found controller %s", guid);

        SDL_Haptic * hap = SDL_HapticOpenFromJoystick( joy );
        entry.haptic = hap;

        if(hap)
        {
            qDebug("[dwControllerHub] Controller %s has haptic support", guid);
            if(SDL_HapticRumbleInit(hap))
            {
                qDebug("[dwControllerHub] Controller %s does not rumble", guid);
                SDL_HapticClose(hap);
                hap = 0;
                entry.haptic = 0;
            }
            else
                SDL_HapticRumblePlay(hap, 0.5, 500);
        }
        else
        {
            qDebug("[dwControllerHub] Controller %s has NO haptic support", guid);
        }

        m_controllers.push_back(entry);

        emit controllerConnected(m_controllers.size() - 1);
    }
}
#endif
