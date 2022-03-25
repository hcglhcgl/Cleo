#include "ubridge.h"

ULine::ULine(UBridge* bridge_ptr, bool openLog)
{
    bridge = bridge_ptr;
}

void ULine::subscribe()
{
    bridge->send("liv subscribe 1\n"); //Line sensor values
}

void ULine::decode(char* msg)
{
    char* p1 = &msg[3]; //space after "liv"

    for (int i=0; i<8; i++)
    {
        lineValues[i] = strtol(p1, &p1, 0);
    }
    updated();
}