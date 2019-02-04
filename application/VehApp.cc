

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

#include "VehApp.h"

const simsignalwrap_t VehApp::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);
const simsignalwrap_t VehApp::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(VehApp);

void VehApp::initialize(int stage) {
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

        sendEntMsg = par("sendEntMsg").boolValue();

        entMsgADataLengthBits = par("entMsgADataLengthBits").longValue();
        entMsgAUserPriority = par("entMsgAUserPriority").longValue();
        entMsgAInterval = par("entMsgAInterval").doubleValue();

        entMsgBDataLengthBits = par("entMsgBDataLengthBits").longValue();
        entMsgBUserPriority = par("entMsgBUserPriority").longValue();
        entMsgBInterval = par("entMsgBInterval").doubleValue();


        isParked = false;


        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);
        sendEntMsgAEvt = new cMessage("ent msg A evt", SEND_ENT_A_EVT);
        sendEntMsgBEvt = new cMessage("ent msg B evt", SEND_ENT_B_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

        //XXX stats for entertainment msgs
        generatedEntMsgA = 0;
        receivedEntMsgA = 0;
        generatedEntMsgB = 0;
        receivedEntMsgB = 0;

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
                scheduleAt(firstBeacon, sendBeaconEvt);
            }

        }

        //XXX Initialize Requesting of services
        InitializeEntService();
        //findHost()->getDisplayString().updateWith("r=300,green");
    }


}

simtime_t VehApp::computeAsynchronousSendingTime(simtime_t interval, t_channel chan) {

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

void VehApp::populateWSM(WaveShortMessage* wsm, int rcvId, int serial) {

    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(-1); //XXX changed by me. before, the default value is rcvId
    wsm->setSerial(serial);
    wsm->setBitLength(headerLength);
    //XXX Set Receiver Address Added By Pedro
    // I am not able to use recipient address because of recent implementation of unicast
    wsm->setRcvAddress(rcvId);


    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm) ) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(currentOfferedServiceId); //XXX Modified By Me to inform the PSID of the service
        bsm->setChannelNumber(Channels::CCH);
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
        //XXX Added by Pedro
        bsm->setServiceState(serviceState);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(Channels::CCH);
        //XXX Modified By Pedro
        wsa->setTargetChannel(currentServiceChannel);
        wsa->setPsid(currentOfferedServiceId);
        //XXX https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7428792 (Standard aproved for PSIDs)
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else if (EntertainmentMessageA* entMsgA = dynamic_cast<EntertainmentMessageA*>(wsm)) {

    }
    else if (EntertainmentMessageB* entMsgB = dynamic_cast<EntertainmentMessageB*>(wsm)) {

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

void VehApp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void VehApp::handlePositionUpdate(cObject* obj) {
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getCurrentPosition();
    curSpeed = mobility->getCurrentSpeed();
}

void VehApp::handleParkingUpdate(cObject* obj) {
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

void VehApp::handleLowerMsg(cMessage* msg) {
    //XXX variables to compute network metrics
    simtime_t jitter;
    simtime_t delay;

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    ASSERT(wsm);

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm)) {
        if(netMetricsBSM.receivedPackets == 0) {
            netMetricsBSM.timeRxFirst = simTime();
        }
        else{
            netMetricsBSM.timeRxLast = simTime();
        }

        netMetricsBSM.receivedPackets++;
        netMetricsBSM.rxBytesSum += bsm->getByteLength();
        delay = simTime() - bsm->getTimestamp();
        netMetricsBSM.delaySum += delay;
        jitter = netMetricsBSM.lastDelay - delay;
        netMetricsBSM.lastDelay = delay;

        if (jitter > SimTime(0)) {
            netMetricsBSM.jitterSum += jitter;
        }
        else{
            netMetricsBSM.jitterSum -= jitter;
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

            if(netMetricsEntA.receivedPackets == 0) {
                netMetricsEntA.timeRxFirst = simTime();
            }
            else{
                netMetricsEntA.timeRxLast = simTime();
            }

            netMetricsEntA.receivedPackets++;
            netMetricsEntA.rxBytesSum += entMsgA->getByteLength();
            delay = simTime() - entMsgA->getTimestamp();
            netMetricsEntA.delaySum +=  delay;
            jitter = netMetricsEntA.lastDelay - delay;
            netMetricsEntA.lastDelay = delay;

            if (jitter > SimTime(0)) {
                netMetricsEntA.jitterSum += jitter;
            }
            else{
                netMetricsEntA.jitterSum -= jitter;
            }
            onEntMsgA(entMsgA);
        }
        else{
            netMetricsEntA.receivedNeighborPackets++;//SCH packets thats not are addressed to me
        }
    }
    else if (EntertainmentMessageB* entMsgB = dynamic_cast<EntertainmentMessageB*>(wsm)) {
        //XXX Only handle msgs destinated to me
        if(entMsgB->getRcvAddress() == myId){
            if(netMetricsEntB.receivedPackets == 0) {
                netMetricsEntB.timeRxFirst = simTime();
            }
            else{
                netMetricsEntB.timeRxLast = simTime();
            }

            netMetricsEntB.receivedPackets++;
            netMetricsEntB.rxBytesSum += entMsgB->getByteLength();
            delay = simTime() - entMsgB->getTimestamp();
            netMetricsEntB.delaySum += delay;
            jitter = netMetricsEntB.lastDelay - delay;
            netMetricsEntB.lastDelay = delay;

            if (jitter > SimTime(0)) {
                netMetricsEntB.jitterSum += jitter;
            }
            else{
                netMetricsEntB.jitterSum -= jitter;
            }
            onEntMsgB(entMsgB);
        }
        else{
            netMetricsEntB.receivedNeighborPackets++; // SCH packets that not are adressed to me
        }
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete(msg);
}

void VehApp::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {

        BasicSafetyMessage* bsm = new BasicSafetyMessage();

        populateWSM(bsm, getSimulation()->getModuleByPath("rsu[0]")->getId());
        sendDown(bsm);

        std::cout << "My Address " << bsm->getSenderAddress() << endl;
        std::cout << "Service State Variable" << serviceState << endl;

        ManageEntServiceState();


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
        //only receiving for now
        break;
    }
    case SEND_ENT_B_EVT: {
        //only receiving for now
        break;
    }
    default: {
        if (msg)
            DBG_APP << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
}

void VehApp::onBSM(BasicSafetyMessage* bsm){

}

void VehApp::onEntMsgA(EntertainmentMessageA* entMsgA){

}

void VehApp::onEntMsgB(EntertainmentMessageB* entMsgB){

}

void VehApp::InitializeEntService(){

    currentServiceChannel = Channels::SCH1;
    mac->changeServiceChannel(currentServiceChannel);

    if( ( RNGCONTEXT bernoulli(0.5) ) ){
        currentOfferedServiceId = WavePsid::Entertainment_A;
        currentServiceDescription = "Entertainment Application A";
    }
    else{
        currentOfferedServiceId = WavePsid::Entertainment_B;
        currentServiceDescription = "Entertainment Application B";
    }

    serviceState = WaveEntServiceState::REQUESTING; // set service state to requesting

}

void VehApp::ManageEntServiceState(){
    if(serviceState == WaveEntServiceState::REQUESTING){
        serviceState = WaveEntServiceState::RECEIVING;
    }
}


void VehApp::finish() {
    std::string text;

    recordScalar("generatedWSMs",generatedWSMs);
    recordScalar("receivedWSMs",receivedWSMs);

    //XXX I modified this for best stats of BSM the another ones are untouched for now
    text = "generatedBSM_"+std::to_string(myId);
    recordScalar(text.c_str(), netMetricsBSM.generatedPackets);
    text = "receivedBSM_"+std::to_string(myId);
    recordScalar(text.c_str(), netMetricsBSM.receivedPackets);
    text = "delaySumBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.delaySum);
    text = "jitterSumBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.jitterSum);
    text = "rxBytesSumBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.rxBytesSum);
    text = "txBytesSumBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.txBytesSum);
    text = "timeRxFirstBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.timeRxFirst);
    text = "timeRxLastBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.timeRxLast);
    text = "timeTxFirstBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.timeTxFirst);
    text = "timeTxLastBSM_"+std::to_string(myId);
    recordScalar(text.c_str(),netMetricsBSM.timeTxLast);

    recordScalar("generatedWSAs",generatedWSAs);
    recordScalar("receivedWSAs",receivedWSAs);

    recordScalar("generatedWSAs",generatedWSAs);
    recordScalar("receivedWSAs",receivedWSAs);

    // XXX stats for entertainment msgs
    if (currentOfferedServiceId == WavePsid::Entertainment_A){

        text = "generatedEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.generatedPackets);
        text = "receivedEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.receivedPackets);
        text = "delaySumEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.delaySum);
        text = "jitterSumEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.jitterSum);
        text = "rxBytesSumEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.rxBytesSum);
        text = "txBytesSumEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.txBytesSum);
        text = "timeRxFirstEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.timeRxFirst);
        text = "timeRxLastEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.timeRxLast);
        text = "timeTxFirstEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.timeTxFirst);
        text = "timeTxLastEntMsg_"+std::to_string(Entertainment_A)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntA.timeTxLast);

    }
    else{
        text = "generatedEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.generatedPackets);
        text = "receivedEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.receivedPackets);
        text = "delaySumEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.delaySum);
        text = "jitterSumEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.jitterSum);
        text = "rxBytesSumEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.rxBytesSum);
        text = "txBytesSumEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.txBytesSum);
        text = "timeRxFirstEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.timeRxFirst);
        text = "timeRxLastEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.timeRxLast);
        text = "timeTxFirstEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.timeTxFirst);
        text = "timeTxLastEntMsg_"+std::to_string(Entertainment_B)+"_"+std::to_string(myId);
        recordScalar(text.c_str(),netMetricsEntB.timeTxLast);
    }

    //FIXME record the another net statistics here!
}




VehApp::~VehApp() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    cancelAndDelete(sendEntMsgAEvt);
    cancelAndDelete(sendEntMsgBEvt);
    findHost()->unsubscribe(mobilityStateChangedSignal, this);
}

void VehApp::startService(Channels::ChannelNumber channel, int serviceId, std::string serviceDescription) {
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

void VehApp::stopService() {
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void VehApp::sendDown(cMessage* msg) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void VehApp::sendDelayedDown(cMessage* msg, simtime_t delay) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void VehApp::checkAndTrackPacket(cMessage* msg) {
    if (isParked && !communicateWhileParked) error("Attempted to transmit a message while parked, but this is forbidden by current configuration");

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(msg)) {
        DBG_APP << "sending down a BSM" << std::endl;

        if(netMetricsBSM.generatedPackets == 0){
            netMetricsBSM.timeTxFirst = simTime();
            netMetricsBSM.timeTxLast = simTime();
        }
        else{
            netMetricsBSM.timeTxLast = simTime();
        }
        netMetricsBSM.txBytesSum += bsm->getByteLength();

        netMetricsBSM.generatedPackets++;

    }
    else if (dynamic_cast<WaveServiceAdvertisment*>(msg)) {
        DBG_APP << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (EntertainmentMessageA* entMsgA = dynamic_cast<EntertainmentMessageA*>(msg)) {
        DBG_APP << "sending down a esm A" << std::endl;

        if(generatedEntMsgA == 0){
            netMetricsEntA.timeTxFirst = simTime();
            netMetricsEntA.timeTxLast = simTime();
        }
        else{
            netMetricsEntA.timeTxLast = simTime();
        }
        netMetricsEntA.txBytesSum += bsm->getByteLength();

        netMetricsEntA.generatedPackets++;
    }
    else if (dynamic_cast<EntertainmentMessageB*>(msg)) {
        DBG_APP << "sending down a esm B" << std::endl;

        if(netMetricsEntB.generatedPackets == 0){
            netMetricsEntB.timeTxFirst = simTime();
            netMetricsEntB.timeTxLast = simTime();
        }
        else{
            netMetricsEntB.timeTxLast = simTime();
        }
        netMetricsEntB.txBytesSum += bsm->getByteLength();

        netMetricsEntB.generatedPackets++;
    }
    else if (dynamic_cast<WaveShortMessage*>(msg)) {
        DBG_APP << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }
}
