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
std::set<std::string> IntersectionApp::interestingIds = { "car1" };
std::map<simtime_t, double> IntersectionApp::car1Speeds;
std::map<simtime_t, double> IntersectionApp::bus0Speeds;
std::map<simtime_t, double> IntersectionApp::bus1Speeds;
std::map<simtime_t, double> IntersectionApp::bus2Speeds;
std::map<simtime_t, double> IntersectionApp::bus3Speeds;
std::map<simtime_t, double> IntersectionApp::bus4Speeds;
std::map<simtime_t, double> IntersectionApp::bus5Speeds;
std::map<simtime_t, double> IntersectionApp::bus6Speeds;
std::map<simtime_t, double> IntersectionApp::bus7Speeds;
std::map<simtime_t, double> IntersectionApp::bus8Speeds;

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
    exportMessageExchangeCounts();
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
                    populateCMBrodcast(cm);
                }

            }

            // If the current car is "car1", store its speed
            std::string carId = mobility->getExternalId();
            if (carId == "car1") {
                double speed = mobility->getSpeed();
                car1Speeds[simTime()] = speed;
            }else if(carId=="bus0"){
                double speed = mobility->getSpeed();
                bus0Speeds[simTime()] = speed;
            }else if(carId=="bus1"){
                double speed = mobility->getSpeed();
                bus1Speeds[simTime()] = speed;
            }else if(carId=="bus2"){
                double speed = mobility->getSpeed();
                bus2Speeds[simTime()] = speed;
            }else if(carId=="bus3"){
                double speed = mobility->getSpeed();
                bus3Speeds[simTime()] = speed;
            }else if(carId=="bus4"){
                double speed = mobility->getSpeed();
                bus4Speeds[simTime()] = speed;
            }else if(carId=="bus5"){
                double speed = mobility->getSpeed();
                bus5Speeds[simTime()] = speed;
            }else if(carId=="bus6"){
                double speed = mobility->getSpeed();
                bus6Speeds[simTime()] = speed;
            }else if(carId=="bus7"){
                double speed = mobility->getSpeed();
                bus7Speeds[simTime()] = speed;
            }else if(carId=="bus8"){
                double speed = mobility->getSpeed();
                bus8Speeds[simTime()] = speed;
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
        msgQueue.push(cm);
        if (!msgQueue.empty()) {
            CarMessage* cm = msgQueue.top(); // Get the highest-priority message
            msgQueue.pop();  // Remove the message from the queue
            onCarMessage(cm);
        }
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
    EV << "Received CarMessage from car " << cm->getSenderAddress() << ": "
              << cm->getHelloMsg() << "\n";
}

void IntersectionApp::exportMessageExchangeCounts() {
    std::ofstream outputFile("message_exchange.csv");
    outputFile << "CarId,RSUId,MessageCount,Timestamp\n";

    for (const auto& pair : messageExchangeCount) {
        std::string carId = pair.first.first;
        std::string rsuId = pair.first.second;
        int count = pair.second.first;
        std::string time = pair.second.second;

        outputFile << carId << "," << rsuId << "," << count << "," << time
                << "\n";
    }

    // Log the message queue
    outputFile << "\nMessage Queue:\n";
    while (!msgQueue.empty()) {
        CarMessage* cm = msgQueue.top();
        std::string carId = std::string(cm->getCarId());
        std::string rsuId = std::string(cm->getRsuId());
        int priority = cm->getPriority();
        outputFile << "CarId: " << carId << ", RSUId: " << rsuId
                << ", Priority: " << priority << "\n";
        msgQueue.pop();
    }
    outputFile.close();

    // Log the speed of car1
    std::ofstream speedFile("car1_speed.csv");
    speedFile << "Timestamp,Speed\n";
    for (const auto& pair : car1Speeds) {
        speedFile << pair.first << "," << pair.second << "\n";
    }
    speedFile.close();

    // Log the speed of car1
    std::ofstream speedFile1("bus0_speed.csv");
    speedFile1 << "Timestamp,Speed\n";
    for (const auto& pair : bus0Speeds) {
        speedFile1 << pair.first << "," << pair.second << "\n";
    }
    speedFile1.close();

    // Log the speed of car1
    std::ofstream speedFile2("bus1_speed.csv");
    speedFile2 << "Timestamp,Speed\n";
    for (const auto& pair : bus1Speeds) {
        speedFile2 << pair.first << "," << pair.second << "\n";
    }
    speedFile2.close();

    // Log the speed of car1
    std::ofstream speedFile3("bus2_speed.csv");
    speedFile3 << "Timestamp,Speed\n";
    for (const auto& pair : bus2Speeds) {
        speedFile3 << pair.first << "," << pair.second << "\n";
    }
    speedFile3.close();

    // Log the speed of car1
    std::ofstream speedFile4("bus3_speed.csv");
    speedFile4 << "Timestamp,Speed\n";
    for (const auto& pair : bus3Speeds) {
        speedFile4 << pair.first << "," << pair.second << "\n";
    }
    speedFile4.close();

    // Log the speed of car1
    std::ofstream speedFile5("bus4_speed.csv");
    speedFile5 << "Timestamp,Speed\n";
    for (const auto& pair : bus4Speeds) {
        speedFile5 << pair.first << "," << pair.second << "\n";
    }
    speedFile5.close();

    // Log the speed of car1
    std::ofstream speedFile6("bus5_speed.csv");
    speedFile6 << "Timestamp,Speed\n";
    for (const auto& pair : bus5Speeds) {
        speedFile6 << pair.first << "," << pair.second << "\n";
    }
    speedFile6.close();

    // Log the speed of car1
    std::ofstream speedFile7("bus6_speed.csv");
    speedFile7 << "Timestamp,Speed\n";
    for (const auto& pair : bus6Speeds) {
        speedFile7 << pair.first << "," << pair.second << "\n";
    }
    speedFile7.close();

    // Log the speed of car1
    std::ofstream speedFile8("bus7_speed.csv");
    speedFile8 << "Timestamp,Speed\n";
    for (const auto& pair : bus7Speeds) {
        speedFile8 << pair.first << "," << pair.second << "\n";
    }
    speedFile8.close();

    // Log the speed of car1
    std::ofstream speedFile9("bus8_speed.csv");
    speedFile9 << "Timestamp,Speed\n";
    for (const auto& pair : bus8Speeds) {
        speedFile9 << pair.first << "," << pair.second << "\n";
    }
    speedFile9.close();
}
