//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __SLICING_RSUAPP_H_
#define __SLICING_RSUAPP_H_

#include <map>

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
#include "veins/modules/messages/WaveServiceAdvertisement_m.h"
#include "veins/modules/messages/BasicSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

//XXX My Includes
#include "veins/modules/messages/EntertainmentMessageA_m.h"
#include "veins/modules/messages/EntertainmentMessageB_m.h"


using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

//#define DBG_APP std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath() << " "

#ifndef DBG_APP
#define DBG_APP EV
#endif

/**
 * @brief
 * WAVE application layer base class.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 *
 * @see BaseWaveApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class RSUApp : public BaseApplLayer {

    public:
        enum WavePsid {
            Entertainment_A = 40,
            Entertainment_B = 41
        };

        enum WaveEntServiceState{
            REQUESTING = 0,
            RECEIVING = 1
        };

        //XXX Network Metrics Statistics
        struct t_NetMetrics{
            simtime_t delaySum = SimTime(0);
            simtime_t jitterSum = SimTime(0);
            simtime_t lastDelay = SimTime(0);
            simtime_t timeRxFirst = SimTime(0);
            simtime_t timeRxLast = SimTime(0);
            simtime_t timeTxFirst = SimTime(0);
            simtime_t timeTxLast = SimTime(0);
            uint32_t txBytesSum = 0;
            uint32_t rxBytesSum = 0;
            uint32_t generatedPackets = 0;
            uint32_t receivedPackets = 0;
            uint32_t receivedNeighborPackets = 0;
        };
        typedef struct t_NetMetrics NetMetrics;

        ~RSUApp();
        virtual void initialize(int stage);
        virtual void finish();

        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);

        enum WaveApplMessageKinds {
            SEND_BEACON_EVT,
            SEND_WSA_EVT,
            //XXX Added By Pedro two events for two Entertainment applications
            SEND_ENT_A_EVT,
            SEND_ENT_B_EVT,
            SERVICE_MAINTENANCE_EVT //timer for maintenance of the services
        };

    protected:

        static const simsignalwrap_t mobilityStateChangedSignal;
        static const simsignalwrap_t parkingStateChangedSignal;

        /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
        virtual void handleLowerMsg(cMessage* msg);

        /** @brief handle self messages */
        virtual void handleSelfMsg(cMessage* msg);

        /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
        virtual void populateWSM(WaveShortMessage*  wsm, int rcvId=-1, int serial=0);

        /** @brief this function is called upon receiving a WaveShortMessage */
        virtual void onWSM(WaveShortMessage* wsm) { };

        /** @brief this function is called upon receiving a BasicSafetyMessage, also referred to as a beacon  */
        virtual void onBSM(BasicSafetyMessage* bsm);

        /** @brief this function is called upon receiving a WaveServiceAdvertisement */
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        /** @brief this function is called every time the vehicle receives a position update signal */
        virtual void handlePositionUpdate(cObject* obj);

        /** @brief this function is called every time the vehicle parks or starts moving again */
        virtual void handleParkingUpdate(cObject* obj);

        /** @brief This will start the periodic advertising of the new service on the CCH
         *
         *  @param channel the channel on which the service is provided
         *  @param serviceId a service ID to be used with the service
         *  @param serviceDescription a literal description of the service
         */
        virtual void startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription);

        /** @brief stopping the service and advertising for it */
        virtual void stopService();

        /** @brief compute a point in time that is guaranteed to be in the correct channel interval plus a random offset
         *
         * @param interval the interval length of the periodic message
         * @param chantype the type of channel, either type_CCH or type_SCH
         */
        virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, t_channel chantype);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         */
        virtual void sendDown(cMessage* msg);

        /**
         * @brief overloaded for error handling and stats recording purposes
         *
         * @param msg the message to be sent. Must be a WSM/BSM/WSA
         * @param delay the delay for the message
         */
        virtual void sendDelayedDown(cMessage* msg, simtime_t delay);

        /**
         * @brief helper function for error handling and stats recording purposes
         *
         * @param msg the message to be checked and tracked
         */
        virtual void checkAndTrackPacket(cMessage* msg);

        //XXX My Methods
        virtual void onEntMsgA(EntertainmentMessageA* entMsgA);
        virtual void onEntMsgB(EntertainmentMessageB* entMsgB);

        virtual void SendDataEntService(WaveShortMessage* wsm, uint32_t size);
        virtual void InitializeEntService(BasicSafetyMessage* bsm);
        virtual void MaintenanceEntService(BasicSafetyMessage* bsm);
        virtual void TimeOutEntService();

    protected:

        /* pointers ill be set when used with TraCIMobility */
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;

        AnnotationManager* annotations;
        WaveAppToMac1609_4Interface* mac;

        /* support for parking currently only works with TraCI */
        bool isParked;
        bool communicateWhileParked;

        /* BSM (beacon) settings */
        uint32_t beaconLengthBits;
        uint32_t  beaconUserPriority;
        simtime_t beaconInterval;
        bool sendBeacons;

        /* WSM (data) settings */
        uint32_t  dataLengthBits;
        uint32_t  dataUserPriority;
        bool dataOnSch;

        //XXX For now we defining generic WSM for Entertainment not necessary a video
        //these parameters also can be given on the .ini file
        /* Maximum Transmission Unit*/
        uint32_t maximumTransUnit;
        /*ESM A (data) settings*/
        uint32_t  entMsgADataLengthBits;
        uint32_t  entMsgAUserPriority;
        simtime_t entMsgAInterval;

        /*ESM B (data) settings*/
        uint32_t  entMsgBDataLengthBits;
        uint32_t  entMsgBUserPriority;
        simtime_t entMsgBInterval;

        //send the added Entertainment Messages (this is initially scheduled on initialize method)
        bool sendEntMsg;

        //services maintenance interval
        simtime_t serviceMaintInterval;

        //Pointers to files of video Traces
        std::fstream *videoMedQ;
        std::fstream *videoHighQ;
        uint32_t videoMsgSize;

        //Map to manage video streaming
        std::map<int,std::fstream> videoStreamMap;
        //Map to timers to schedule video stream messages
        std::map<int,cMessage*> timersVideoStreamMap;
        //Map to verify times of last received messages
        std::map<int,simtime_t> lastBeaconVideoStream;


        /* WSA settings */
        int currentOfferedServiceId;
        std::string currentServiceDescription;
        Channels::ChannelNumber currentServiceChannel;
        simtime_t wsaInterval;

        /* state of the vehicle */
        Coord curPosition;
        Coord curSpeed;
        int myId;
        int mySCH;

        /* stats */
        uint32_t generatedWSMs;
        uint32_t generatedWSAs;
        uint32_t generatedBSMs;
        uint32_t receivedWSMs;
        uint32_t receivedWSAs;
        uint32_t receivedBSMs;

        /* XXX stats for network metrics by service  (SCH)*/
        std::map<int,NetMetrics> netMetricsEntA;
        std::map<int,NetMetrics> netMetricsEntB;

        /* XXX stats for network metrics by service  (CCH)*/
        NetMetrics netMetricsBSM;

        //FIXME Probably will be necessary create for WSAs in the final version

        /* messages for periodic events such as beacon and WSA transmissions */
        cMessage* sendBeaconEvt;
        cMessage* sendWSAEvt;
        /* XXX Here I've defined my additional messages for simulate de transmissions on the slices*/
        //Two events for two different applications of entertainment
        cMessage* sendEntMsgAEvt;
        cMessage* sendEntMsgBEvt;
        //a self message for maintenance of the service events
        cMessage* serviceMaintEvt;
};

#endif
