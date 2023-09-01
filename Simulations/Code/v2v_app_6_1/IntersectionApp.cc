#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "veins/base/utils/Coord.h"
#include "IntersectionApp.h"

Define_Module(IntersectionApp);

std::map<std::pair<std::string, std::string>, double> IntersectionApp::carDistances;
std::map<std::string, Coord> IntersectionApp::carPositions;
std::map<std::string, Coord> IntersectionApp::rsuPositions;
std::map<std::pair<std::string, std::string>, std::pair<int, std::string>> IntersectionApp::messageExchangeCount;
std::priority_queue<CarMessage*, std::vector<CarMessage*>,
        IntersectionApp::ComparePriority> IntersectionApp::msgQueue;
std::set<std::string> IntersectionApp::interestingIds = { "car1", "bus0", "bus1", "bus2", "bus3", "bus4", "bus5", "bus6", "bus7", "bus8"};
std::vector<RSUInteraction> IntersectionApp::rsuInteractions;

void IntersectionApp::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        lastSent = simTime();
        //By default, you are not allowed to go through intersection
        canGo = false;
        //Figuring out if you are an RSU
        std::string path = getFullPath();
        if (path.find("rsu") != std::string::npos) {
            isRSU = true;
        } else {
            isRSU = false;
        }
        //Starts channels
        if (dataOnSch) {
            startService(Channels::SCH2, 42, "Traffic Information Service");
        }

        rsuInteractions.reserve(2000000);
    }
}

void IntersectionApp::finish() {
    exportMessageExchangeCounts();
    exportRSUInteractionLog();
}

IntersectionApp::~IntersectionApp() {
    for (auto it = RSUData.begin(); it != RSUData.end(); ++it) {
        delete (*it);
    }
    RSUData.clear();
}
void IntersectionApp::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    //Cars send messages every 1 second
//    if (simTime() - lastSent >= 1 && !isRSU) {
    if (!isRSU) {
        std::string carId = mobility->getExternalId();
        Coord carPos = mobility->getPositionAt(simTime());

        carPositions[carId] = carPos;  // Update the position of the current car

//         Loop over the positions of all cars and calculate the distances
        for (const auto& pair : carPositions) {
            std::string otherCarId = pair.first;
            Coord otherCarPos = pair.second;

            if (otherCarId == carId) {
                continue; // Skip self
            }

            double distance = calculateDistance(carPos, otherCarPos);
            carDistances[ { carId, otherCarId }] = distance;

            if (distance <= 40) {
                CarMessage* cm = new CarMessage();
                populateCMBrodcast(cm);

                //If there is currently data on the channel, the car cannot send the message right then
                if (dataOnSch) {
                    //schedule message to self to send later
                    scheduleAt(computeAsynchronousSendingTime(1, type_SCH), cm);
                }
                //Send on CCH, because channel switching is disabled
                else {
                    //Sends with specified delay in message
                    simtime_t delayTimeCars = par("delayTimeCars");
                    sendDelayedDown(cm, delayTimeCars);
                }

                if (interestingIds.count(carId) > 0) {
                    cm->setPriority(1);  // Set the priority of the message
                    cm->setRsuId(otherCarId.c_str());
                    cm->setCarId(carId.c_str());
                    cm->setSendTime(simTime());
                    populateCMBrodcast(cm);

                    // Log this message exchange
                    // When a message is exchanged, do:
                    auto& record = messageExchangeCount[ { carId, otherCarId }];
                    record.first += 1; // increment message count

                    // Append the current simTime to the string.
                    if (!record.second.empty()) {
                        record.second += ",";
                    }
                    record.second += std::to_string((int) simTime().dbl());
                } else {
                    cm->setPriority(5);  // Set the priority of the message
                    cm->setRsuId(otherCarId.c_str());
                    cm->setCarId(carId.c_str());
                    cm->setSendTime(simTime());
                    populateCMBrodcast(cm);
                }

            }

        }

    } else {
        for (int i = 0; i < 13; i++) {
            std::string rsuPath = "rsu[" + std::to_string(i) + "].mobility";
            BaseMobility* mobilityModule = check_and_cast<BaseMobility*>(
                    getSimulation()->getModuleByPath(rsuPath.c_str()));
            if (mobilityModule) {
                Coord pos = mobilityModule->getCurrentPosition();
                rsuPositions[rsuPath] = pos;
            } else {
                // Handle the case where the module was not found.
                EV_ERROR << "Could not find module at path: " << rsuPath
                                << "\n";
            }
        }
    }

}

double IntersectionApp::calculateDistance(const Coord& pos1,
        const Coord& pos2) {
    return pos1.distance(pos2);
}

void IntersectionApp::handleSelfMsg(cMessage* msg) {
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
        sendDown(cm->dup());
        delete (cm);
    } else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void IntersectionApp::handleLowerMsg(cMessage* msg) {
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
      if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
          onCarMessage(cm);
      }
      delete (msg);
}

void IntersectionApp::populateCMBrodcast(CarMessage* cm) {
    cm->setHelloMsg("Hello World");
    cm->setSenderAddress(myId);
    cm->setRecipientAddress(-1);  // -1 indicates broadcast
    cm->setWsmData(cm->getHelloMsg());
    cm->setWsmLength(strlen(cm->getHelloMsg()));
}

void IntersectionApp::onCarMessage(CarMessage* cm) {
    EV << "Received CarMessage from RSU " << cm->getSenderAddress() << ": " << cm->getHelloMsg() << "\n";

    // Extract sender and receiver RSU IDs
    std::string senderRsuId = cm->getRsuId();
    std::string receiverRsuId = cm->getCarId();

    // Check if either the sender or receiver RSU ID is interesting
    if (interestingIds.count(senderRsuId) > 0 && interestingIds.count(receiverRsuId) > 0) {
        RSUInteraction interaction;
        interaction.setSenderRsuId(senderRsuId.c_str());
        interaction.setReceiverRsuId(receiverRsuId.c_str());
        interaction.setSendTime(cm->getSendTime());
        interaction.setReceiveTime(simTime());

        rsuInteractions.push_back(interaction);
    }
}


void IntersectionApp::exportRSUInteractionLog() {
    std::ofstream file("RSU_Interactions.csv");
    file << "Sender RSU ID,Receiver RSU ID,Send Time,Receive Time\n";
    for (const auto& interaction : rsuInteractions) {
        file << interaction.getSenderRsuId() << ","
             << interaction.getReceiverRsuId() << ","
             << interaction.getSendTime() << ","
             << interaction.getReceiveTime() << "\n";
    }
    file.close();
}

void IntersectionApp::exportMessageExchangeCounts() {
    std::ofstream outputFile("message_exchange_2.csv");
    outputFile << "CarId,RSUId,MessageCount,Timestamp\n";

    for (const auto& pair : messageExchangeCount) {
        std::string carId = pair.first.first;
        std::string rsuId = pair.first.second;
        int count = pair.second.first;
        std::string time = pair.second.second;

        outputFile << carId << "," << rsuId << "," << count << "," << time
                << "\n";
    }
    outputFile.close();
}
