#pragma once

#include "autonav/path.h"

extern int ms;

class BarrelRacing : public path {
    virtual void Run() override;
};