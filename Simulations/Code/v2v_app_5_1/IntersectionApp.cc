#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "veins/base/utils/Coord.h"
#include "IntersectionApp.h"

Define_Module(IntersectionApp);

std::map<std::string, Coord> IntersectionApp::carPositions;
std::map<std::string, Coord> IntersectionApp::rsuPositions;
std::map<std::pair<std::string, std::string>, std::pair<int, std::string>> IntersectionApp::messageExchangeCount;
std::priority_queue<CarMessage*, std::vector<CarMessage*>, IntersectionApp::ComparePriority> IntersectionApp::msgQueue;
std::set<std::string> IntersectionApp::interestingIds = { "car1" };
std::map<std::string, std::vector<int>> IntersectionApp::rsuAvailability;
std::map<std::string, std::vector<Service*>> IntersectionApp::rsuServices;
std::vector<InteractionData> IntersectionApp::interactions;
std::set<std::string> IntersectionApp::loggedInteractions;
std::vector<RSUInteraction> IntersectionApp::rsuInteractions;
bool checkOnce = true;

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
    }
}

void IntersectionApp::finish() {
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
    if (!isRSU) {
            // do nothing
    } else {
        if (checkOnce) {
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
            checkOnce = false;
        } else {
            std::string currentRsuId = getFullPath().substr(getFullPath().find("rsu[") + 4);
            currentRsuId = currentRsuId.substr(0, currentRsuId.find(']'));
            for (const auto& receiverRsu : rsuPositions) {
                if (receiverRsu.first != currentRsuId) {
                    CarMessage* cm = new CarMessage();
                    cm->setPriority(1);
                    cm->setRsuId(receiverRsu.first.c_str());
                    cm->setSenderRsuId(currentRsuId.c_str());
                    cm->setSendTime(simTime());
                    populateCMBrodcast(cm);
                    sendDown(cm);

                    // Log this message exchange
                    // When a message is exchanged, do:
                    std::string carId = std::string(cm->getCarId());
                    std::string rsuId = std::string(cm->getRsuId());
                    auto& record = messageExchangeCount[ { carId, rsuId }];
                    record.first += 1; // increment message count
                    // Append the current simTime to the string.
                    if (!record.second.empty()) {
                        record.second += ",";
                    }
                    record.second += std::to_string((int) simTime().dbl());
                }
            }

        }
    }

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

    // Extract the receiver RSU ID from the current module's path
    std::string receiverRsuId = getFullPath().substr(getFullPath().find("rsu[") + 4);
    receiverRsuId = receiverRsuId.substr(0, receiverRsuId.find(']'));

    RSUInteraction interaction;
    interaction.setSenderRsuId(cm->getSenderRsuId()); // No need to convert to std::string
    interaction.setReceiverRsuId(receiverRsuId.c_str()); // Convert to const char*
    interaction.setSendTime(cm->getSendTime());
    interaction.setReceiveTime(simTime());

    rsuInteractions.push_back(interaction);
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

