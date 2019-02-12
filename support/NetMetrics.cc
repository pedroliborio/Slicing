/*
 * NetMetrics.cc
 *
 *  Created on: 11 de fev de 2019
 *      Author: liborio
 */

#include "NetMetrics.h"

namespace Support {

NetMetrics::NetMetrics() {
    delaySum = SimTime(0);
    jitterSum = SimTime(0);
    lastDelay = SimTime(0);
    timeRxFirst = SimTime(0);
    timeRxLast = SimTime(0);
    timeTxFirst = SimTime(0);
    timeTxLast = SimTime(0);
    txBytes = 0;
    rxBytes = 0;
    txPackets = 0;
    rxPackets = 0;
    rxNeighborPackets = 0;
}

NetMetrics::~NetMetrics() {
    // TODO Auto-generated destructor stub
}


simtime_t NetMetrics::Compute_Delay (void) {
    if (rxPackets == 0) {
        return 0;
    }
    else{
        return (delaySum.dbl() / rxPackets);
    }
}

simtime_t NetMetrics::Compute_Jitter (void) {
    if (rxPackets <= 1) {
        return 0;
    }
    else {
        return jitterSum.dbl() / (rxPackets - 1);
    }
}

double NetMetrics::Compute_MeanTxPacketSize (void){
    if (txPackets == 0) {
        return 0;
    }
    else{
        return txBytes / txPackets;
    }
}

double NetMetrics::Compute_MeanRxPacketSize (void) {
    if (rxPackets == 0) {
        return 0;
    }
    else {
        return rxBytes / rxPackets;
    }
}

double NetMetrics::Compute_MeanTxThroughput (void){
    if ( (timeTxLast.dbl() - timeTxFirst.dbl()) == 0) {
        return 0;
    }
    else{
        return (8 * txBytes) / ( timeTxLast.dbl() - timeTxFirst.dbl() );
    }
}

double NetMetrics::Compute_MeanRxThroughput (void) {
    if( (timeRxLast.dbl() - timeRxFirst.dbl()) == 0){
        return 0;
    }
    else{
        return (8 * rxBytes) / (timeRxLast.dbl() - timeRxFirst.dbl()) ;
    }
}

const simtime_t& NetMetrics::getDelaySum() const {
    return delaySum;
}

void NetMetrics::setDelaySum(const simtime_t& delaySum) {
    this->delaySum = delaySum;
}

const simtime_t& NetMetrics::getJitterSum() const {
    return jitterSum;
}

void NetMetrics::setJitterSum(const simtime_t& jitterSum) {
    this->jitterSum = jitterSum;
}

const simtime_t& NetMetrics::getLastDelay() const {
    return lastDelay;
}

void NetMetrics::setLastDelay(const simtime_t& lastDelay) {
    this->lastDelay = lastDelay;
}

uint32_t NetMetrics::getRxBytes() const {
    return rxBytes;
}

void NetMetrics::setRxBytes(uint32_t rxBytes) {
    this->rxBytes = rxBytes;
}

uint32_t NetMetrics::getRxNeighborPackets() const {
    return rxNeighborPackets;
}

void NetMetrics::setRxNeighborPackets(uint32_t rxNeighborPackets) {
    this->rxNeighborPackets = rxNeighborPackets;
}

uint32_t NetMetrics::getRxPackets() const {
    return rxPackets;
}

void NetMetrics::setRxPackets(uint32_t rxPackets) {
    this->rxPackets = rxPackets;
}

const simtime_t& NetMetrics::getTimeRxFirst() const {
    return timeRxFirst;
}

void NetMetrics::setTimeRxFirst(const simtime_t& timeRxFirst) {
    this->timeRxFirst = timeRxFirst;
}

const simtime_t& NetMetrics::getTimeRxLast() const {
    return timeRxLast;
}

void NetMetrics::setTimeRxLast(const simtime_t& timeRxLast) {
    this->timeRxLast = timeRxLast;
}

const simtime_t& NetMetrics::getTimeTxFirst() const {
    return timeTxFirst;
}

void NetMetrics::setTimeTxFirst(const simtime_t& timeTxFirst) {
    this->timeTxFirst = timeTxFirst;
}

const simtime_t& NetMetrics::getTimeTxLast() const {
    return timeTxLast;
}

void NetMetrics::setTimeTxLast(const simtime_t& timeTxLast) {
    this->timeTxLast = timeTxLast;
}

uint32_t NetMetrics::getTxBytes() const {
    return txBytes;
}

void NetMetrics::setTxBytes(uint32_t txBytes) {
    this->txBytes = txBytes;
}

uint32_t NetMetrics::getTxPackets() const {
    return txPackets;
}

void NetMetrics::setTxPackets(uint32_t txPackets) {
    this->txPackets = txPackets;
}

} /* namespace Support */
