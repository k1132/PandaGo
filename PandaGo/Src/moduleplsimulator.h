#pragma once

//includings of App headers
#include "infrastructure.h"
#include "../Planner/plsdk.h"

class ModulePlSimulator
{
public:
    ModulePlSimulator();
    ~ModulePlSimulator();
    void init();
    void call(PLContext * _data, int _cur);
    void quit();
};
