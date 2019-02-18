/*
 * NetMetrics.h
 *
 *  Created on: 11 de fev de 2019
 *      Author: liborio
 */

#ifndef SUPPORT_NETMETRICS_H_
#define SUPPORT_NETMETRICS_H_

#include <omnetpp.h>
using namespace omnetpp;

namespace Support {

class NetMetrics {
    public:
        NetMetrics();
        virtual ~NetMetrics();

    private:
            simtime_t delaySum;
            simtime_t jitterSum;
            simtime_t lastDelay;
            simtime_t timeRxFirst;
            simtime_t timeRxLast;
            simtime_t timeTxFirst;
            simtime_t timeTxLast;
            uint32_t txBytes;
            uint32_t rxBytes;
            uint32_t txPackets;
            uint32_t rxPackets;
            uint32_t rxNeighborPackets;
    public:
            simtime_t Compute_Delay (void);
            simtime_t Compute_Jitter (void);
            double Compute_MeanTxPacketSize (void);
            double Compute_MeanRxPacketSize (void);
            double Compute_MeanTxThroughput (void);
            double Compute_MeanRxThroughput (void);
            double Compute_TxTimeOfService (void);
            double Compute_RxTimeOfService (void);
            const simtime_t& getDelaySum() const;
            void setDelaySum(const simtime_t& delaySum);
            const simtime_t& getJitterSum() const;
            void setJitterSum(const simtime_t& jitterSum);
            const simtime_t& getLastDelay() const;
            void setLastDelay(const simtime_t& lastDelay);
            uint32_t getRxBytes() const;
            void setRxBytes(uint32_t rxBytes);
            uint32_t getRxNeighborPackets() const;
            void setRxNeighborPackets(uint32_t rxNeighborPackets);
            uint32_t getRxPackets() const;
            void setRxPackets(uint32_t rxPackets);
            const simtime_t& getTimeRxFirst() const;
            void setTimeRxFirst(const simtime_t& timeRxFirst);
            const simtime_t& getTimeRxLast() const;
            void setTimeRxLast(const simtime_t& timeRxLast);
            const simtime_t& getTimeTxFirst() const;
            void setTimeTxFirst(const simtime_t& timeTxFirst);
            const simtime_t& getTimeTxLast() const;
            void setTimeTxLast(const simtime_t& timeTxLast);
            uint32_t getTxBytes() const;
            void setTxBytes(uint32_t txBytes);
            uint32_t getTxPackets() const;
            void setTxPackets(uint32_t txPackets);
};

} /* namespace Support */

#endif /* SUPPORT_NETMETRICS_H_ */
