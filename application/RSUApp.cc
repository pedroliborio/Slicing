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

#include "RSUApp.h"

Define_Module(RSUApp);

const simsignalwrap_t RSUApp::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);
const simsignalwrap_t RSUApp::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(RSUApp);

void RSUApp::initialize(int stage) {
    BaseApplLayer::initialize(stage);

    if (stage==0) {

        //initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = NULL;
            mobility = NULL;
            traciVehicle = NULL;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<WaveAppToMac1609_4Interface*>::findSubModule(
                getParentModule());
        assert(mac);

        myId = getParentModule()->getId();

        //read parameters
        headerLength = par("headerLength").longValue();
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits").longValue();
        beaconUserPriority = par("beaconUserPriority").longValue();
        beaconInterval =  par("beaconInterval").doubleValue();

        dataLengthBits = par("dataLengthBits").longValue();
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority").longValue();

        wsaInterval = par("wsaInterval").doubleValue();
        communicateWhileParked = par("communicateWhileParked").boolValue();
        currentOfferedServiceId = -1;


        //XXX Added by Pedro Parameters for the Entertainment Messages

        maximumTransUnit = par("maximumTransUnit").longValue();

        sendEntMsg = par("sendEntMsg").boolValue();

        entMsgADataLengthBytes = par("entMsgADataLengthBytes").longValue();
        entMsgAUserPriority = par("entMsgAUserPriority").longValue();
        entMsgAInterval = par("entMsgAInterval").doubleValue();

        entMsgBDataLengthBytes = par("entMsgBDataLengthBytes").longValue();
        entMsgBUserPriority = par("entMsgBUserPriority").longValue();
        entMsgBInterval = par("entMsgBInterval").doubleValue();

        serviceMaintInterval = SimTime(1.0, SIMTIME_S); //timer to maintenance of the services interval


        isParked = false;


        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);
        //sendEntMsgAEvt = new cMessage("ent msg A evt", SEND_ENT_A_EVT); // event of send video type A
        //sendEntMsgBEvt = new cMessage("ent msg B evt", SEND_ENT_B_EVT); // event of send video type B
        serviceMaintEvt = new cMessage("service maintenance", SERVICE_MAINTENANCE_EVT); //maintenance of the services

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

    }
    else if (stage == 1) {
        //simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            std::cerr << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if ( beaconInterval.raw() % (mac->getSwitchingInterval().raw()*2)) {
                    std::cerr << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2*mac->getSwitchingInterval() << "). "
                            << "This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, type_CCH);
            }

            if (sendBeacons) {
                //XXX Added a little time to send the first beacons respecting the dessincronization
                scheduleAt(firstBeacon, sendBeaconEvt);
            }

//            if (sendEntMsg) {
//                mac->changeServiceChannel(Channels::SCH1);
//                scheduleAt(computeAsynchronousSendingTime(entMsgAInterval, type_SCH), sendEntMsgAEvt);
//                scheduleAt(computeAsynchronousSendingTime(entMsgBInterval, type_SCH), sendEntMsgBEvt);
//                //FIXME verificar a questao de cooordenar a fatia de tempo transmitindo no canal
//            }


        }

        std::cout <<"RSU ID RSU: "<< myId << endl;
        //XXX Schedule Maintenance of service Event
        //scheduleAt(simTime()+serviceMaintInterval, serviceMaintEvt);
        //findHost()->getDisplayString().updateWith("r=500,green");

    }
}

simtime_t RSUApp::computeAsynchronousSendingTime(simtime_t interval, t_channel chan) {

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); //usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earliest in next CCH (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval*2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() %switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    //check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw()  % (2*switchingInterval.raw()) > switchingInterval.raw()) {
        //firstEvent is within a sch interval
        if (chan == type_CCH) firstEvent -= switchingInterval;
    }
    else {
        //firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == type_SCH) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void RSUApp::populateWSM(WaveShortMessage* wsm, int rcvId, int serial) {

    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(-1); //XXX changed by me, before teh default value is rcvId
    wsm->setSerial(serial);
    wsm->setBitLength(headerLength);
    //XXX Set Receive Address Added By Pedro
    // I am not able to use recipient address because of recent implementation of unicast
    //does not support multichannel operations
    wsm->setRcvAddress(rcvId);


    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm) ) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(Channels::CCH);
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(Channels::CCH);
        wsa->setTargetChannel(currentServiceChannel);
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else if (EntertainmentMessageA* entMsgA = dynamic_cast<EntertainmentMessageA*>(wsm)) {
        entMsgA->setChannelNumber(Channels::SCH1);
        entMsgA->setUserPriority(4);
        entMsgA->setPsid(40);
        //the video frame size is setup in the call of this method
    }
    else if (EntertainmentMessageB* entMsgB = dynamic_cast<EntertainmentMessageB*>(wsm)) {
        entMsgB->setChannelNumber(Channels::SCH1);
        entMsgB->setUserPriority(4);
        entMsgB->setPsid(41);
    }
    //XXX Changed By Me
    else{
        if (dataOnSch)
            wsm->setChannelNumber(Channels::SCH1); //will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else
            wsm->setChannelNumber(Channels::CCH);
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void RSUApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void RSUApp::handlePositionUpdate(cObject* obj) {
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getCurrentPosition();
    curSpeed = mobility->getCurrentSpeed();
}

void RSUApp::handleParkingUpdate(cObject* obj) {
    //this code should only run when used with TraCI
    isParked = mobility->getParkingState();
    if (communicateWhileParked == false) {
        if (isParked == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
        }
        else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

void RSUApp::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    //XXX variables to compute network metrics
    simtime_t jitter;
    simtime_t delay;

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm)) {

        if(netMetricsBSM.getRxPackets() == 0) {
            netMetricsBSM.setTimeRxFirst(simTime());
            netMetricsBSM.setTimeRxLast(simTime());
        }
        else{
            netMetricsBSM.setTimeRxLast(simTime());
        }

        netMetricsBSM.setRxPackets(netMetricsBSM.getRxPackets()+1);
        netMetricsBSM.setRxBytes( netMetricsBSM.getRxBytes() + bsm->getByteLength());
        delay = simTime() - bsm->getTimestamp();
        netMetricsBSM.setDelaySum( netMetricsBSM.getDelaySum() + delay);
        jitter = netMetricsBSM.getLastDelay() - delay;
        netMetricsBSM.setLastDelay( delay );

        if (jitter > SimTime(0)) {
            netMetricsBSM.setJitterSum( netMetricsBSM.getJitterSum() + jitter);
        }
        else{
            netMetricsBSM.setJitterSum( netMetricsBSM.getJitterSum() - jitter);
        }
        onBSM(bsm);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else if (EntertainmentMessageA* entMsgA = dynamic_cast<EntertainmentMessageA*>(wsm)) {
        //XXX Only handle msgs destinated to me
        if (entMsgA->getRcvAddress() == myId) {

            std::map<int, NetMetrics*>::iterator itStatsEntMsg = netMetricsEntA.begin();
            itStatsEntMsg = netMetricsEntA.find(entMsgA->getSenderAddress());

            if (itStatsEntMsg != netMetricsEntA.end()) {

                if (itStatsEntMsg->second->getRxPackets() == 0) {
                    itStatsEntMsg->second->setTimeRxFirst(simTime());
                    itStatsEntMsg->second->setTimeRxLast(simTime());
                }
                else {
                    itStatsEntMsg->second->setTimeRxLast(simTime());
                }

                itStatsEntMsg->second->setRxPackets( itStatsEntMsg->second->getRxPackets() + 1 );
                itStatsEntMsg->second->setRxBytes( itStatsEntMsg->second->getRxBytes() + entMsgA->getByteLength());
                delay = simTime() - entMsgA->getTimestamp();
                itStatsEntMsg->second->setDelaySum( itStatsEntMsg->second->getDelaySum() + delay);
                jitter = itStatsEntMsg->second->getLastDelay() - delay;
                itStatsEntMsg->second->setLastDelay( delay );
                if (jitter > SimTime(0)) {
                    itStatsEntMsg->second->setJitterSum( itStatsEntMsg->second->getJitterSum( ) + jitter);
                }
                else{
                    itStatsEntMsg->second->setJitterSum( itStatsEntMsg->second->getJitterSum( ) - jitter);
                }

            }

            onEntMsgA(entMsgA);
        }
        else{
            std::map<int, NetMetrics*>::iterator itStatsEntMsg = netMetricsEntA.begin();
            itStatsEntMsg = netMetricsEntA.find(entMsgA->getSenderAddress());
            if (itStatsEntMsg != netMetricsEntA.end()) {
                itStatsEntMsg->second->setRxNeighborPackets( itStatsEntMsg->second->getRxNeighborPackets() + 1 );
            }
        }
    }
    else if (EntertainmentMessageB* entMsgB = dynamic_cast<EntertainmentMessageB*>(wsm)) {
        //XXX Only handle msgs destinated to me
        if (entMsgB->getRcvAddress() == myId) {

            std::map<int, NetMetrics*>::iterator itStatsEntMsg = netMetricsEntB.begin();
            itStatsEntMsg = netMetricsEntB.find(entMsgB->getSenderAddress());

            if (itStatsEntMsg != netMetricsEntB.end()) {

                if (itStatsEntMsg->second->getRxPackets() == 0) {
                    itStatsEntMsg->second->setTimeRxFirst( simTime() );
                    itStatsEntMsg->second->setTimeRxLast( simTime() );
                }
                else {
                    itStatsEntMsg->second->setTimeRxLast( simTime() );
                }

                itStatsEntMsg->second->setRxPackets( itStatsEntMsg->second->getRxPackets() + 1 );
                itStatsEntMsg->second->setRxBytes( itStatsEntMsg->second->getRxBytes() + entMsgB->getByteLength() );
                delay = simTime() - entMsgB->getTimestamp();
                itStatsEntMsg->second->setDelaySum( itStatsEntMsg->second->getDelaySum() + delay);
                jitter = itStatsEntMsg->second->getLastDelay() - delay;
                itStatsEntMsg->second->setLastDelay( delay );
                if (jitter > SimTime(0)) {
                    itStatsEntMsg->second->setJitterSum( itStatsEntMsg->second->getJitterSum() + jitter );
                }
                else{
                    itStatsEntMsg->second->setJitterSum( itStatsEntMsg->second->getJitterSum() - jitter );
                }

            }
            onEntMsgB(entMsgB);
        }
        else{
            std::map<int, NetMetrics*>::iterator itStatsEntMsg = netMetricsEntB.begin();
            itStatsEntMsg = netMetricsEntB.find(entMsgB->getSenderAddress());
            if (itStatsEntMsg != netMetricsEntB.end()) {
                itStatsEntMsg->second->setRxNeighborPackets( itStatsEntMsg->second->getRxNeighborPackets() + 1 );
            }
        }
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete(msg);
}

void RSUApp::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        BasicSafetyMessage* bsm = new BasicSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    case SEND_WSA_EVT:   {
        WaveServiceAdvertisment* wsa = new WaveServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }
    case SEND_ENT_A_EVT: {
        int vehID = std::stoi(msg->getName() ); // convert vehicle id stored in message event
        uint32_t videoFrameSize, restFragmentSize;
        int i, numFragments;

        std::cout << "RSU: Sending a Video Frame A" << endl;

        std::map<int,std::fstream>::iterator itVSM = videoStreamMap.begin();
        itVSM = videoStreamMap.find(vehID);
        if (itVSM != videoStreamMap.end()) {

            //FIXME Changed to testes in packet size and frequency o data service
            //itVSM->second >> videoFrameSize;
            videoFrameSize = entMsgADataLengthBytes;

            numFragments = std::floor( (videoFrameSize / maximumTransUnit) );
            restFragmentSize = videoFrameSize % maximumTransUnit;

            std::cout << "Frame size: " << videoFrameSize << endl;
            std::cout << "Fragments: " << numFragments << endl;
            std::cout << "Size last fragment: " << restFragmentSize << endl;

            for (i = 0; i < numFragments; i++) {
                EntertainmentMessageA* entMsgA = new EntertainmentMessageA();
                populateWSM(entMsgA,vehID);
                entMsgA->addByteLength(maximumTransUnit);
                std::cout << "Fragment Timestamp: " << entMsgA->getTimestamp() << endl;
                std::cout << "Fragment Size: " << entMsgA->getByteLength() << "bytes." << endl;

                sendDown(entMsgA);
            }

            if (restFragmentSize > 0) {
                EntertainmentMessageA* entMsgA = new EntertainmentMessageA();
                populateWSM(entMsgA,vehID);
                entMsgA->addByteLength(restFragmentSize);
                std::cout << "Fragment Timestamp: " << entMsgA->getTimestamp() << endl;
                std::cout << "Fragment Size: " << entMsgA->getByteLength() << "bytes." << endl;

                sendDown(entMsgA);
            }

            std::map<int,cMessage*>::iterator itTimerVSM = timersVideoStreamMap.begin();
            itTimerVSM = timersVideoStreamMap.find(vehID);
            if (itTimerVSM != timersVideoStreamMap.end()) {
                scheduleAt(simTime() + entMsgAInterval, itTimerVSM->second);
                std::cout << "Timestamp do proximo frame: " <<  simTime() + entMsgAInterval << endl;
            }

        }
        else{
            error ("Got a SelfMessage ENT_A of an vehicle of unknown ID");
        }
        break;
    }
    case SEND_ENT_B_EVT: {
        int vehID = std::stoi( msg->getName() ); // convert vehicle id stored in message event
        uint32_t videoFrameSize, restFragmentSize;
        int i, numFragments;

        std::cout << "RSU: Sending a Video Frame B" << endl;
        std::map<int,std::fstream>::iterator itVSM = videoStreamMap.begin();
        itVSM = videoStreamMap.find(vehID);
        if (itVSM != videoStreamMap.end()) {

            //FIXME Changed to testes in packet size and frequency o data service
            //itVSM->second >> videoFrameSize;
            videoFrameSize = entMsgBDataLengthBytes;

            numFragments = std::floor( (videoFrameSize / maximumTransUnit) );
            restFragmentSize = videoFrameSize % maximumTransUnit;

            std::cout << "Frame size: " << videoFrameSize << endl;
            std::cout << "Fragments: " << numFragments << endl;
            std::cout << "Size last fragment: " << restFragmentSize << endl;

            for (i = 0; i < numFragments; i++) {
                EntertainmentMessageB* entMsgB = new EntertainmentMessageB();
                populateWSM(entMsgB,vehID);
                entMsgB->addByteLength(maximumTransUnit);
                std::cout << "Fragment Timestamp: " << entMsgB->getTimestamp() << endl;
                std::cout << "Fragment Size: " << entMsgB->getByteLength() << "bytes." << endl;
                sendDown(entMsgB);
            }

            if (restFragmentSize > 0) {
                EntertainmentMessageB* entMsgB = new EntertainmentMessageB();
                populateWSM(entMsgB,vehID);
                entMsgB->addByteLength(restFragmentSize);
                std::cout << "Fragment Timestamp: " << entMsgB->getTimestamp() << endl;
                std::cout << "Fragment Size: " << entMsgB->getByteLength() << "bytes." << endl;
                sendDown(entMsgB);
            }

            std::map<int,cMessage*>::iterator itTimerVSM = timersVideoStreamMap.begin();
            itTimerVSM = timersVideoStreamMap.find(vehID);
            if (itTimerVSM != timersVideoStreamMap.end()) {
                scheduleAt(simTime() + entMsgBInterval, itTimerVSM->second);
                std::cout << "Timestamp do proximo frame: " <<  simTime() + entMsgBInterval << endl;
            }

        }
        else{
            error ("Got a SelfMessage ENT_B of an vehicle of unknown ID");
        }
        break;
    }
    case SERVICE_MAINTENANCE_EVT: {
        //FIXME
        //TimeOutEntService();
        scheduleAt(simTime()+serviceMaintInterval, serviceMaintEvt);
        break;
    }
    default: {
        if (msg)
            DBG_APP << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
}

void RSUApp::onWSA(WaveServiceAdvertisment* wsa){

    //Aqui virao as informações de serviço implementacao multicanal do slicng

//        mac->changeServiceChannel(wsa->getTargetChannel());
//        currentOfferedServiceId = wsa->getPsid();
//        currentServiceChannel = (Channels::ChannelNumber) wsa->getTargetChannel();
//        currentServiceDescription = wsa->getServiceDescription();

}

void RSUApp::onBSM(BasicSafetyMessage* bsm){

    if(bsm->getServiceState() == WaveEntServiceState::REQUESTING){

        //Service should be initialized only one time
        //This is necessary because the messages to initialize can be desynchronized over time
        std::map<int,simtime_t>::iterator itLBVS = lastBeaconVideoStream.begin();
        itLBVS = lastBeaconVideoStream.find(bsm->getSenderAddress());
        if (itLBVS != lastBeaconVideoStream.end()) {
            //this stream was already initialized!
            std::cout << "Sender Address: " << bsm->getSenderAddress() << endl;
            std::cout << "Service State Variable: " << bsm->getServiceState() << endl;
            std::cout << "Service Already Initialized!" << endl;

            MaintenanceEntService(bsm); // just perform maintenance
        }
        else{
            //This vehicle not was find in the list of recent received beacons
            //initialize service for it

            //FIXMEInitializeEntService(bsm);
        }
    }
    else{
        if(bsm->getServiceState() == WaveEntServiceState::RECEIVING){
            std::cout << "Sender Address: " << bsm->getSenderAddress() << endl;
            std::cout << "Service State Variable: " << bsm->getServiceState() << endl;
            std::cout << "Vehicle Already Changed it status to RECEIVING!" << endl;
            MaintenanceEntService(bsm);
        }
        else{
            error("Just got a BSM of a unknown service state.");
        }
    }
}

void RSUApp::onEntMsgA(EntertainmentMessageA* entMsgA){

}

void RSUApp::onEntMsgB(EntertainmentMessageB* entMsgB){

}



/*
 * This method will initialize pointer to files for stream video messages
 * as well schedule self messages for maintain the frequency of the video
 * stream (specific for each vehicles). Because each vehicle has their own
 * stream of video to receive with different times of access.
 */

void RSUApp::InitializeEntService(int senderAddress, int psid){

    //XXX Since this module is called by another module, this is necessary
    //@see 4.12 Direct Method Calls Between Modules -  Simulation Manual
    Enter_Method_Silent();

    std::cout << "initializing service on RSU" << endl;

    std::string msgText;
    NetMetrics netmetrics;
    //Initialize Maps data structures
    //based on PSIDs of services
    //Entertainment_A = 40,
    //Entertainment_B = 41

    std::cout << "Sender Address " << senderAddress << endl;
    std::cout << "Service State Variable" << psid << endl;

    if ( psid == WavePsid::Entertainment_A ) {

        //Video File Stream
        videoStreamMap.emplace(
            senderAddress,
            std::fstream("../../video_dataset/Terse_Parkplatz_10.dat.txt",std::fstream::in));

        //Event of send each frame of the video
        std::map<int,cMessage*>::iterator it = timersVideoStreamMap.begin();
        it = timersVideoStreamMap.insert (it, std::pair<int,cMessage*>(senderAddress,
                new cMessage(std::to_string(senderAddress).c_str(), SEND_ENT_A_EVT) ));

        //mac->changeServiceChannel(Channels::SCH1);
        //FIXME this need to be improved because information of the slice will also inform it

        scheduleAt(simTime() + computeAsynchronousSendingTime(entMsgAInterval, type_SCH) + 5.0, it->second);

        //Statistics of this service
        std::map<int,NetMetrics*>::iterator itNetM = netMetricsEntA.begin();
        itNetM = netMetricsEntA.insert (itNetM, std::pair<int,NetMetrics*>(senderAddress,
                new NetMetrics() ));

        //FIXME For now we dont using control packets
        //This could be outside but for effects of error control we willpu in the two coditions
        //std::map<int,simtime_t>::iterator itBVS = lastBeaconVideoStream.begin();
        //itBVS = lastBeaconVideoStream.insert (itBVS, std::pair<int,simtime_t>(senderAddress, bsm->getTimestamp()));

        std::cout << "Service Initialized for vehicle " << senderAddress
                << ", PSID: " << psid << endl;

    }
    else {
        if( psid == WavePsid::Entertainment_B ) {
            videoStreamMap.emplace(
                        senderAddress,
                        std::fstream("../../video_dataset/Terse_Simpsons.dat.txt",std::fstream::in));

            std::map<int,cMessage*>::iterator it = timersVideoStreamMap.begin();
            it = timersVideoStreamMap.insert (it, std::pair<int,cMessage*>( senderAddress,
                    new cMessage(std::to_string(senderAddress).c_str(), SEND_ENT_B_EVT) ));
            //mac->changeServiceChannel(Channels::SCH1);
            scheduleAt(simTime() + computeAsynchronousSendingTime(entMsgBInterval, type_SCH) + 5.0, it->second);

            //Statistics of this service
            std::map<int,NetMetrics*>::iterator itNetM = netMetricsEntB.begin();
            itNetM = netMetricsEntB.insert (itNetM, std::pair<int,NetMetrics*>(senderAddress,
                    new NetMetrics() ));

            //FIXME For now we dont using control packets
            //This could be outside but for effects of error control we willpu in the two coditions
            //std::map<int,simtime_t>::iterator itBVS = lastBeaconVideoStream.begin();
            //itBVS = lastBeaconVideoStream.insert (itBVS, std::pair<int,simtime_t>(bsm->getSenderAddress(), bsm->getTimestamp()));

            std::cout << "Service Initialized for vehicle " << senderAddress
                    << ", PSID: " << psid << endl;
        }
        else{
            error("Got a BSM of unknown PSID.");
        }
    }


    if(videoStreamMap.empty()){
        std::cout << "videostreamMap empty!" << endl;
    }

    if(timersVideoStreamMap.empty()){
        std::cout << "timersVideostreamMap empty!" << endl;
    }

    if(netMetricsEntA.empty()){
        std::cout << "receivedEntMsgA empty!" << endl;
    }

    if(netMetricsEntB.empty()){
        std::cout << "receivedEntMsgB empty!" << endl;
    }

    if(lastBeaconVideoStream.empty()){
        std::cout << "lastBeaconVideoStream empty!" << endl;
    }




    //XXX Added By Pedro Opening Files for Entertainment Messages (Video Traces)
    //Cada arquivo possui as informações:
    //Frame No. | Time[ms] |  Length [byte]
    //videoMedQ->open("../../video_dataset/Terse_Parkplatz_10.dat.txt",std::fstream::in);
    //videoHighQ->open("../../video_dataset/Terse_Simpsons.dat.txt",std::fstream::in);
}


void RSUApp::MaintenanceEntService(BasicSafetyMessage* bsm){

    std::cout << "RSU Maintenance of service." << endl;

    std::map<int,simtime_t>::iterator itLBVS = lastBeaconVideoStream.begin();
    itLBVS = lastBeaconVideoStream.find(bsm->getSenderAddress());
    if (itLBVS != lastBeaconVideoStream.end()) {

        itLBVS->second = bsm->getTimestamp(); // update timestamp of the last beacon msg

        std::cout << "Sender Address: " << bsm->getSenderAddress() << endl;
        std::cout << "Service State Variable: " << bsm->getServiceState() << endl;
        std::cout << "Service PSID: " << bsm->getPsid() << endl;
        std::cout << "Last Timestamp: " << bsm->getTimestamp() << endl;

    }
}


/*
 * This method evaluates all last times of received beacons of
 * each vehicle which RSU is streaming over
 * If its is too old (ex. 4 sec.) this stream need to be closed
 * FIXME The future version need to begin a handover at this point
 * because vehicle will get in coverage of the subsequent RSU.
 */
void RSUApp::TimeOutEntService(int senderAddress, int psid){

    //XXX Since this module is called by another module, this is necessary
    //@see 4.12 Direct Method Calls Between Modules -  Simulation Manual
    Enter_Method_Silent();

    std::cout << "Time Now: " << simTime() << endl;
    std::cout << "Finalizing service instance for Vehicle ID: " << senderAddress << endl;
    std::cout << "PSID: " << psid << endl;
    std::cout << "Active Services: " << videoStreamMap.size() << endl;

    std::map<int,std::fstream>::iterator itVSM = videoStreamMap.begin();
    itVSM = videoStreamMap.find(senderAddress);
    if (itVSM != videoStreamMap.end()) {
        itVSM->second.close(); //fecha o arquivo de stream de video
        videoStreamMap.erase (itVSM); //remove do map
    }
    else{
        cRuntimeError("Vehicle ID does not in list of video stream.");
    }

    std::map<int,cMessage*>::iterator itTimerVSM = timersVideoStreamMap.begin();
    itTimerVSM = timersVideoStreamMap.find(senderAddress);
    if (itTimerVSM != timersVideoStreamMap.end()) {
        //we need to verify if its message is currently scheduled (should be)
        //if so, it needs to be canceled and deleted
        if(itTimerVSM->second->isScheduled()){
            cancelAndDelete(itTimerVSM->second);
        }
        timersVideoStreamMap.erase (itTimerVSM); //remove do map
    }
    else{
        cRuntimeError("Vehicle ID does not in list of video stream. ");
    }

}


void RSUApp::SendDataEntService(WaveShortMessage* wsm, uint32_t size){

//    fullPkt = trec_.trec_size / maximumTransUnit;
//    restPkt = trec_.trec_size % maximumTransUnit;
//    if (fullPkt > 0) {
//        for (i = 0; i < fullPkt; i++) {
//
//        }
//    }
//    if (restPkt != 0) {
//
//    }
}


void RSUApp::finish() {
    std::string text;

    //recordScalar("generatedWSMs",generatedWSMs);
    //recordScalar("receivedWSMs",receivedWSMs);

    //XXX I modified this for best stats of BSM the another ones are untouched for now
    text = "txPackets_CCH_"+std::to_string( myId ) ;
    recordScalar(text.c_str(),netMetricsBSM.getTxPackets() );
    text = "rxPackets_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.getRxPackets() );
    text = "rxNeighborPackets_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.getRxNeighborPackets() );
    text = "meanDelay_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_Delay() );
    text = "meanJitter_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_Jitter() );
    text = "meanRxPacketSize_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_MeanRxPacketSize() );
    text = "meanTxPacketSize_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_MeanTxPacketSize() );
    text = "meanRxThroughput_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_MeanRxThroughput() );
    text = "meanTxThroughput_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_MeanTxThroughput() );
    text = "rxTimeOfService_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_RxTimeOfService() );
    text = "txTimeOfService_CCH_"+std::to_string( myId );
    recordScalar(text.c_str(),netMetricsBSM.Compute_TxTimeOfService() );

    //recordScalar("generatedWSAs",generatedWSAs);
    //recordScalar("receivedWSAs",receivedWSAs);

    //XXX stats

    std::map<int,NetMetrics*>::iterator itStatsEntMsg = netMetricsEntA.begin();
    while (itStatsEntMsg != netMetricsEntA.end()) {
        text = "txPackets_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first);
        recordScalar(text.c_str(),itStatsEntMsg->second->getTxPackets() );
        text = "rxPackets_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->getRxPackets() );
        text = "rxNeighborPackets_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->getRxNeighborPackets() );
        text = "meanDelay_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_Delay() );
        text = "meanJitter_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_Jitter() );
        text = "meanRxPacketSize_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanRxPacketSize() );
        text = "meanTxPacketSize_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanTxPacketSize() );
        text = "meanRxThroughput_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanRxThroughput() );
        text = "meanTxThroughput_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanTxThroughput() );
        text = "rxTimeOfService_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_RxTimeOfService() );
        text = "txTimeOfService_SCH_"+std::to_string(Entertainment_A)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_TxTimeOfService() );
        itStatsEntMsg = netMetricsEntA.erase(itStatsEntMsg);
    }

    itStatsEntMsg = netMetricsEntB.begin();
    while (itStatsEntMsg != netMetricsEntB.end()) {
        text = "txPackets_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first);
        recordScalar(text.c_str(),itStatsEntMsg->second->getTxPackets() );
        text = "rxPackets_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->getRxPackets() );
        text = "rxNeighborPackets_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->getRxNeighborPackets() );
        text = "meanDelay_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_Delay() );
        text = "meanJitter_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_Jitter() );
        text = "meanRxPacketSize_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanRxPacketSize() );
        text = "meanTxPacketSize_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanTxPacketSize() );
        text = "meanRxThroughput_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanRxThroughput() );
        text = "meanTxThroughput_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_MeanTxThroughput() );
        text = "rxTimeOfService_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_RxTimeOfService() );
        text = "txTimeOfService_SCH_"+std::to_string(Entertainment_B)+"_"+std::to_string(itStatsEntMsg->first );
        recordScalar(text.c_str(),itStatsEntMsg->second->Compute_TxTimeOfService() );
        itStatsEntMsg = netMetricsEntB.erase(itStatsEntMsg);
    }



}

RSUApp::~RSUApp() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    cancelAndDelete(serviceMaintEvt);
    findHost()->unsubscribe(mobilityStateChangedSignal, this);
}

void RSUApp::startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription) {
    if (sendWSAEvt->isScheduled()) {
        error("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, type_CCH);
    scheduleAt(wsaTime, sendWSAEvt);

}

void RSUApp::stopService() {
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void RSUApp::sendDown(cMessage* msg) {

    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void RSUApp::sendDelayedDown(cMessage* msg, simtime_t delay) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void RSUApp::checkAndTrackPacket(cMessage* msg) {
    if (isParked && !communicateWhileParked) error("Attempted to transmit a message while parked, but this is forbidden by current configuration");

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(msg)) {
        DBG_APP << "sending down a BSM" << std::endl;

        if(netMetricsBSM.getTxPackets() == 0){
            netMetricsBSM.setTimeTxFirst( simTime() );
            netMetricsBSM.setTimeTxLast( simTime() );
        }
        else{
            netMetricsBSM.setTimeTxLast( simTime() );
        }

        netMetricsBSM.setTxBytes( netMetricsBSM.getTxBytes() + bsm->getByteLength());
        netMetricsBSM.setTxPackets( netMetricsBSM.getTxPackets() + 1 );
    }
    else if (dynamic_cast<WaveServiceAdvertisment*>(msg)) {
        DBG_APP << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (EntertainmentMessageA* entMsgA = dynamic_cast<EntertainmentMessageA*>(msg)) {

        std::cout << " ************ RSU, SERVICE MSG A, ID RECEIVER:  "<< entMsgA->getRcvAddress() << endl;
        std::cout << " ************ RSU, SERVICE MSG A, TIMESTAMP:  "<< entMsgA->getTimestamp() << endl;

        std::map<int,NetMetrics*>::iterator itStatsEntMsg = netMetricsEntA.begin();
        itStatsEntMsg = netMetricsEntA.find(entMsgA->getRcvAddress());
        if (itStatsEntMsg != netMetricsEntA.end()) {
            if(itStatsEntMsg->second->getTxPackets() == 0){
                itStatsEntMsg->second->setTimeTxFirst( simTime() );
                itStatsEntMsg->second->setTimeTxLast( simTime() );
            }
            else{
                itStatsEntMsg->second->setTimeTxLast( simTime() );
            }

            itStatsEntMsg->second->setTxBytes( itStatsEntMsg->second->getTxBytes() + entMsgA->getByteLength() );
            itStatsEntMsg->second->setTxPackets( itStatsEntMsg->second->getTxPackets() + 1);
        }
    }
    else if (EntertainmentMessageB* entMsgB = dynamic_cast<EntertainmentMessageB*>(msg)) {

        std::cout << " ************ RSU, SERVICE MSG B, ID RECEIVER:  "<< entMsgB->getRcvAddress() << endl;
        std::cout << " ************ RSU, SERVICE MSG B, TIMESTAMP:  "<< entMsgB->getTimestamp() << endl;

        std::map<int,NetMetrics*>::iterator itStatsEntMsg = netMetricsEntB.begin();
        itStatsEntMsg = netMetricsEntB.find(entMsgB->getRcvAddress());
        if (itStatsEntMsg != netMetricsEntB.end()) {
            if(itStatsEntMsg->second->getTxPackets() == 0){
                itStatsEntMsg->second->setTimeTxFirst( simTime() );
                itStatsEntMsg->second->setTimeTxLast( simTime() );
            }
            else{
                itStatsEntMsg->second->setTimeTxLast( simTime() );
            }

            itStatsEntMsg->second->setTxBytes( itStatsEntMsg->second->getTxBytes() + entMsgB->getByteLength() );
            itStatsEntMsg->second->setTxPackets(itStatsEntMsg->second->getTxPackets() + 1 );
        }

    }
    else if (dynamic_cast<WaveShortMessage*>(msg)) {
            DBG_APP << "sending down a wsm" << std::endl;
            generatedWSMs++;
    }
}
