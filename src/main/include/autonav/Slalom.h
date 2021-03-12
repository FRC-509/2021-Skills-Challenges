#pragma once

#include "autonav/path.h"

extern int ms;

class Slalom : public path {
    public:
    virtual void Run() override;
};