#ifndef SRC_COMMUNICATIONSSYSTEM_H_
#define SRC_COMMUNICATIONSSYSTEM_H_

#include <iostream>
#include <thread>
#include <sys/dispatch.h>
#include "Msg_structs.h"

// CommunicationsSystem class
class CommunicationsSystem {
public:
	CommunicationsSystem(); // Constructor
	~CommunicationsSystem(); // Destructor
	int chid; // Channel ID
private:
    void HandleCommunications(); // Method to handle communications with the aircraft
    void messageAircraft(const Message& msg); // Method to send a message to the aircraft

    // Method to send a message to the aircraft and check if the aircraft replied with a full msg_plane_info payload
    bool messageAircraftAugmented(const Message& msg, Message_inter_process& outReply); 
    
    std::thread Communications_System; // Thread to handle the communications system
};

#endif /* SRC_COMMUNICATIONSSYSTEM_H_ */
