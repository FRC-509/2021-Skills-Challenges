#pragma once

#include "autonav/path.h"

class Bounce : public path {
    public:
    virtual void Run() override;
};