//
// Created by shjzhang on 24-5-14.
//
#include <gnsslab/GnssStruct.h>

#ifndef EPHSTORE_H
#define EPHSTORE_H

class EphStore {
public:

    virtual ~EphStore()
    {};
    virtual Xvt getXvt(const SatID& sat, const CommonTime& epoch) = 0;
};

#endif
