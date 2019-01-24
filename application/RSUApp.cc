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

        entMsgADataLengthBits = par("entMsgADataLengthBits").longValue();
        entMsgAUserPriority = par("entMsgAUserPriority").longValue();
        entMsgAInterval = par("entMsgAInterval").doubleValue();

        entMsgBDataLengthBits = par("entMsgBDataLengthBits").longValue();
        entMsgBUserPriority = par("entMsgBUserPriority").longValue();
        entMsgBInterval = par("entMsgBInterval").doubleValue();

        serviceMaintInterval = SimTime(1.0, SIMTIME_S); //timer to maintenance of the services interval


        isParked = false;


        findHost()->subscribe(mobilityStateChangedSignal, this);
        findHost()->subscribe(parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);
        sendEntMsgAEvt = new cMessage("ent msg A evt", SEND_ENT_A_EVT); // event of send video type A
        sendEntMsgBEvt = new cMessage("ent msg B evt", SEND_ENT_B_EVT); // event of send video type B
        serviceMaintEvt = new cMessage("service maintenance", SERVICE_MAINTENANCE_EVT); //maintenance of the services

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

        //XXX Added By Pedro Stats for entertainment msgs
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

//            if (sendEntMsg) {
//                mac->changeServiceChannel(Channels::SCH1);
//                scheduleAt(computeAsynchronousSendingTime(entMsgAInterval, type_SCH), sendEntMsgAEvt);
//                scheduleAt(computeAsynchronousSendingTime(entMsgBInterval, type_SCH), sendEntMsgBEvt);
//                //FIXME verificar a questao de cooordenar a fatia de tempo transmitindo no canal
//            }


        }

        //XXX Schedule Maintenance of service Interval
        scheduleAt(simTime()+serviceMaintInterval, serviceMaintEvt);
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
    wsm->setRecipientAddress(rcvId);
    wsm->setSerial(serial);
    wsm->setBitLength(headerLength);


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
        //the video frame size is setup in the call of this method
    }
    else if (EntertainmentMessageB* entMsgB = dynamic_cast<EntertainmentMessageB*>(wsm)) {
        /*entMsgB->setChannelNumber(Channels::SCH1);
        entMsgB->addBitLength(1000);
        //FIXME need to be modified bby the video frame size
        entMsgB->setUserPriority(4);*/
        //para alterar o canal de envio
        //mac->changeServiceChannel(wsa->getTargetChannel());
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

    if (BasicSafetyMessage* bsm = dynamic_cast<BasicSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (WaveServiceAdvertisment* wsa = dynamic_cast<WaveServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else if (EntertainmentMessageA* entMsgA = dynamic_cast<EntertainmentMessageA*>(wsm)) {
        receivedEntMsgA++;
        onEntMsgA(entMsgA);
    }
    else if (EntertainmentMessageB* entMsgB = dynamic_cast<EntertainmentMessageB*>(wsm)) {
        receivedEntMsgB++;
        onEntMsgB(entMsgB);
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

        int vehID = std::stoi(msg->getName()); // convert vehicle id stored in message event
        uint32_t videoFrameSize, restFragmentSize;
        int i, numFragments;

        std::map<int,std::fstream>::iterator itVSM = videoStreamMap.begin();
        itVSM = videoStreamMap.find(vehID);
        if (itVSM != videoStreamMap.end()) {

            itVSM->second >> videoFrameSize;

            numFragments = videoFrameSize / maximumTransUnit;
            restFragmentSize = videoFrameSize % maximumTransUnit;

            if (numFragments > 0) {
                for (i = 0; i < numFragments; i++) {
                    EntertainmentMessageA* entMsgA = new EntertainmentMessageA();
                    populateWSM(entMsgA);
                    entMsgA->addByteLength(1500);
                    sendDown(entMsgA);
                }
            }
            if (restFragmentSize > 0) {
                EntertainmentMessageA* entMsgA = new EntertainmentMessageA();
                populateWSM(entMsgA);
                entMsgA->addByteLength(restFragmentSize);
                sendDown(entMsgA);
            }

        }
        else{
            error ("Got a SelfMessage ENT_A of an vehicle of unknown ID");
        }
        scheduleAt(simTime() + entMsgAInterval, sendEntMsgAEvt);
        break;
    }
    case SEND_ENT_B_EVT: {
        EntertainmentMessageA* entMsgB = new EntertainmentMessageA();
        populateWSM(entMsgB);
        sendDown(entMsgB);
        scheduleAt(simTime() + entMsgBInterval, sendEntMsgBEvt);
        break;
    }
    case SERVICE_MAINTENANCE_EVT: {
        TimeOutEntService();
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
        InitializeEntService(bsm);
    }
    else{
        if(bsm->getServiceState() == WaveEntServiceState::RECEIVING){
            MaintenanceEntService(bsm);
        }
        else{
            error("Just got a BSM of a unknown service state.");
        }
    }
}

/*
 * This method will initialize pointer to files for stream video messages
 * as well schedule self messages for maintain the frequency of the video
 * stream (specific for each vehicles). Because each vehicle has their own
 * stream of video to receive with different times of access.
 */

void RSUApp::InitializeEntService(BasicSafetyMessage* bsm){

    std::string msgText;
    //Initialize Maps data structures
    //based on PSIDs of services
    //Entertainment_A = 40,
    //Entertainment_B = 41

    if ( bsm->getPsid() == WavePsid::Entertainment_A ) {

        videoStreamMap.emplace(
            bsm->getSenderAddress(),
            std::fstream("../../video_dataset/Terse_Parkplatz_10.dat.txt",std::fstream::in));

        //inserting elemnt with a hint after insert it returns a iterator to the new inserted element
        //see hint version at: http://www.cplusplus.com/reference/map/map/insert/
        //so we can use the second element to create a timer for this vehicle video stream

        std::map<int,cMessage*>::iterator it = timersVideoStreamMap.begin();
        it = timersVideoStreamMap.insert (it, std::pair<int,cMessage*>(bsm->getSenderAddress(),
                new cMessage(std::to_string(bsm->getSenderAddress()).c_str(), SEND_ENT_A_EVT) ));
        mac->changeServiceChannel(Channels::SCH1);

        scheduleAt(simTime() + computeAsynchronousSendingTime(entMsgAInterval, type_SCH), it->second);


    }
    else {
        if( bsm->getPsid() == WavePsid::Entertainment_B ) {
            videoStreamMap.emplace(
                        bsm->getSenderAddress(),
                        std::fstream("../../video_dataset/Terse_Simpsons.dat.txt",std::fstream::in));

            //inserting elemnt with a hint after insert it returns a iterator to the new inserted element
            //see hint version at: http://www.cplusplus.com/reference/map/map/insert/
            //so we can use the second element to create a timer for this vehicle video stream

            std::map<int,cMessage*>::iterator it = timersVideoStreamMap.begin();
            it = timersVideoStreamMap.insert (it, std::pair<int,cMessage*>( bsm->getSenderAddress(),
                    new cMessage(std::to_string(bsm->getSenderAddress()).c_str(), SEND_ENT_B_EVT) ));
            mac->changeServiceChannel(Channels::SCH1);

            scheduleAt(simTime() + computeAsynchronousSendingTime(entMsgBInterval, type_SCH), it->second);
        }
        else{
            error("Got a BSM of unknown/unregistered PSID.");
        }
    }

    //XXX Added By Pedro Opening Files for Entertainment Messages (Video Traces)
    //Cada arquivo possui as informações:
    //Frame No. | Time[ms] |  Length [byte]
    //videoMedQ->open("../../video_dataset/Terse_Parkplatz_10.dat.txt",std::fstream::in);
    //videoHighQ->open("../../video_dataset/Terse_Simpsons.dat.txt",std::fstream::in);
}

void RSUApp::MaintenanceEntService(BasicSafetyMessage* bsm){
    lastBeaconVideoStream.insert( std::pair<int,simtime_t>( bsm->getSenderAddress(),bsm->getTimestamp() ) );
}


/*
 * This method evaluates all last times of received beacons of
 * each vehicle which RSU is streaming over
 * If its is too old (ex. 4 sec.) this stream need to be closed
 * FIXME The future version need to begin a handover at this point
 * because vehicle will get in coverage of the subsequent RSU.
 */
void RSUApp::TimeOutEntService(){

    std::map<int,simtime_t>::iterator itr = lastBeaconVideoStream.begin();
    while (itr != lastBeaconVideoStream.end()) {
        if ( (simTime().dbl() - itr->second.dbl()) >= 4.0) {

            std::map<int,std::fstream>::iterator itVSM = videoStreamMap.begin();
            itVSM = videoStreamMap.find(itr->first);
            if (itVSM != videoStreamMap.end()) {
                itVSM->second.close(); //fecha o arquivo de stream de video
                videoStreamMap.erase (itVSM); //remove do map
            }

            std::map<int,cMessage*>::iterator itTimerVSM = timersVideoStreamMap.begin();
            itTimerVSM = timersVideoStreamMap.find(itr->first);
            if (itTimerVSM != timersVideoStreamMap.end()) {
                //we need to verify if its message is currently scheduled (should be)
                //in true case, it needs to be canceled and deleted
                if(itTimerVSM->second->isScheduled()){
                    cancelAndDelete(itTimerVSM->second);
                }
                timersVideoStreamMap.erase (itTimerVSM); //remove do map
            }

            itr = lastBeaconVideoStream.erase(itr);
        }
        else {
           ++itr;
        }
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
    recordScalar("generatedWSMs",generatedWSMs);
    recordScalar("receivedWSMs",receivedWSMs);

    recordScalar("generatedBSMs",generatedBSMs);
    recordScalar("receivedBSMs",receivedBSMs);

    recordScalar("generatedWSAs",generatedWSAs);
    recordScalar("receivedWSAs",receivedWSAs);

    recordScalar("generatedWSAs",generatedWSAs);
    recordScalar("receivedWSAs",receivedWSAs);

    // XXX stats for entertainment msgs
    recordScalar("generatedEntMsgA",generatedEntMsgA);
    recordScalar("receivedEntMsgA",receivedEntMsgA);

    recordScalar("generatedEntMsgB",generatedEntMsgB);
    recordScalar("receivedEntMsgB",receivedEntMsgB);

    //XXX Closing Services involved data files
    videoMedQ->close();
    videoHighQ->close();

}

RSUApp::~RSUApp() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    cancelAndDelete(sendEntMsgAEvt);
    cancelAndDelete(sendEntMsgBEvt);
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

    if (dynamic_cast<BasicSafetyMessage*>(msg)) {
        DBG_APP << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<WaveServiceAdvertisment*>(msg)) {
        DBG_APP << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<EntertainmentMessageA*>(msg)) {
        DBG_APP << "sending down a esm A" << std::endl;
        generatedEntMsgA++;
    }
    else if (dynamic_cast<EntertainmentMessageB*>(msg)) {
        DBG_APP << "sending down a esm B" << std::endl;
        generatedEntMsgB++;
    }
    else if (dynamic_cast<WaveShortMessage*>(msg)) {
            DBG_APP << "sending down a wsm" << std::endl;
            generatedWSMs++;
    }
}
