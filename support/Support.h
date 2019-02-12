/*
 * Support.h
 *
 *  Created on: 11 de fev de 2019
 *      Author: liborio
 */

#ifndef SUPPORT_SUPPORT_H_
#define SUPPORT_SUPPORT_H_

#include "omnetpp.h"

using namespace omnetpp;

////XXX Network Metrics Statistics
//struct t_NetMetrics{
//    simtime_t delaySum = SimTime(0);
//    simtime_t jitterSum = SimTime(0);
//    simtime_t lastDelay = SimTime(0);
//    simtime_t timeRxFirst = SimTime(0);
//    simtime_t timeRxLast = SimTime(0);
//    simtime_t timeTxFirst = SimTime(0);
//    simtime_t timeTxLast = SimTime(0);
//    uint32_t txBytesSum = 0;
//    uint32_t rxBytesSum = 0;
//    uint32_t generatedPackets = 0;
//    uint32_t receivedPackets = 0;
//    uint32_t receivedNeighborPackets = 0;
//};
//typedef struct t_NetMetrics NetMetrics;


enum WavePsid {
    Entertainment_A = 40,
    Entertainment_B = 41
};

enum WaveEntServiceState{
    REQUESTING = 0,
    RECEIVING = 1
};


namespace Support {

class Support {
    public:
        Support();
        virtual ~Support();


};

} /* namespace Support */

#endif /* SUPPORT_SUPPORT_H_ */
