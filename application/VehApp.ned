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

package com.liborio.slicing.application;
import org.car2x.veins.base.modules.IBaseApplLayer;
//
// TODO auto-generated module
//

simple VehApp like IBaseApplLayer
{
    parameters:
        bool debug = default(false); //whether to print debug output

        int headerLength = default(88bit) @unit(bit); //header length of the application

        bool sendBeacons = default(true); //tell the applayer to periodically send beacons
        int beaconLengthBits = default(256bit) @unit(bit); //the length of a beacon packet
        int beaconUserPriority = default(7); //the user priority (UP) of the beacon messages
        double beaconInterval = default(1s) @unit(s); //the intervall between 2 beacon messages

        int dataLengthBits = default(1024bit) @unit(bit); //the length of a data packet
        bool dataOnSch = default(false); //tells the applayer whether to use a service channel for datapackets or the control channel
        int dataUserPriority = default(7); //the default user priority (UP) for data packets

        bool avoidBeaconSynchronization = default(true); //don't start beaconing directly after node was created but delay to avoid artifical synchronization

        bool sendWSA = default(false);
        int wsaLengthBits = default(250bit) @unit(bit);
        double wsaInterval =  default(1s) @unit(s);

        bool communicateWhileParked = default(false); //whether the car should communicate when it is parked in SUMO
        
        // Attributes added by Pedro
        int maximumTransUnit = default(1500);
        //send Entertainment Messages
        bool sendEntMsg = default(false);
        //Entertainment Message A
        int  entMsgADataLengthBytes = default(1500 byte) @unit(byte); //This isnot being used for now 
        int  entMsgAUserPriority = default(4); //De acordo com a UP definida no padrão 802.11p para video
        double entMsgAInterval = default(0.4s) @unit(s); //enviados a cada 0.4 seg
        //Entertainment Message B
        int  entMsgBDataLengthBytes = default(1500 byte) @unit(byte); //This is not being used for now
        int  entMsgBUserPriority = default(4); //De acordo com a UP definida no padrão 802.11p para video
        double entMsgBInterval = default(0.4s) @unit(s); //enviados a cada 0.4 seg
        
        //InitializeService Selfmessage Interval
        double initServiceInterval = default(1s) @unit(s); //verificado a cada 1seg 
        
        @class(VehApp);
		@display("i=block/app2");

    gates:
        input lowerLayerIn; // from mac layer
        output lowerLayerOut; // to mac layer
        input lowerControlIn;
        output lowerControlOut;
    
}

