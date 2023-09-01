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

        // Populate rsuAvailability from CSV file
        rsuAvailability = readCSV("pred.csv");

        // Step 1: Createing services
        std::vector<std::string> services = { "service1", "service2", "service3", "service4", "service5" };

        std::set<std::string> usedRsus;

        for (const auto& service : services) {
            std::string earliestRsuId;
            double earliestTime = std::numeric_limits<double>::max();

            for (const auto& pair : rsuAvailability) {
                std::string rsuId = pair.first;
                if (usedRsus.count(rsuId) > 0)
                    continue;  // Skip this RSU if it's already used

                std::vector<int> timestamps = pair.second;

                for (auto timestamp : timestamps) {
                    if (timestamp < earliestTime) {
                        earliestTime = timestamp;
                        earliestRsuId = rsuId;
                    }
                }
            }

            if (!earliestRsuId.empty()) {
                double startTime = earliestTime;
                double endTime = startTime + 3;  // Service lasts for 3 seconds
                placeService(earliestRsuId, service, "Service Data", startTime, endTime);  // Place service on RSU

                // Add the RSU to the "used" list
                usedRsus.insert(earliestRsuId);

                // Remove the timestamp from the RSU's availability
                rsuAvailability[earliestRsuId].erase( std::remove(rsuAvailability[earliestRsuId].begin(), rsuAvailability[earliestRsuId].end(),
                        earliestTime), rsuAvailability[earliestRsuId].end());
            } else {
                EV_ERROR << "Could not place service: " << service << "\n";
            }
        }

    }
}

void IntersectionApp::placeService(std::string rsuId, std::string serviceName,
        std::string serviceData, double startTime, double endTime) {
    // Create service
    Service* service = new Service("ServiceName");

    // set fields
    service->setServiceName(serviceName.c_str());
    service->setServiceData(serviceData.c_str());
    service->setStartTime(startTime);
    service->setEndTime(endTime);

    // Add service to the RSU
    rsuServices[rsuId].push_back(service); // Assuming that rsuServices is a map of RSU ids to their assigned services
}

void IntersectionApp::finish() {
    exportMessageExchangeCounts();
    exportInteractionLog();
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
        std::string carId = mobility->getExternalId();
        Coord carPos = mobility->getPositionAt(simTime());

        carPositions[carId] = carPos;  // Update the position of the current car

        for (const auto& pair : rsuPositions) { // Assuming rsuPositions is a map of RSU ids to their positions
            std::string rsuId = pair.first;
            Coord rsuPos = pair.second;

            double distance = calculateDistance(carPos, rsuPos);
            if (distance <= 30) { // Assuming RSU_COMM_RANGE is the communication range of an RSU
                CarMessage* cm = new CarMessage();
                if (dataOnSch) {
                    scheduleAt(computeAsynchronousSendingTime(1, type_SCH), cm);
                } else {
                    simtime_t delayTimeCars = par("delayTimeCars");
                    sendDelayedDown(cm, delayTimeCars);
                }

                if (interestingIds.count(carId) > 0) {
                    cm->setPriority(1);  // Set the priority of the message
                    cm->setRsuId(rsuId.c_str());
                    cm->setCarId(carId.c_str());
                    cm->setSendTime(simTime());
                    populateCMBrodcast(cm);

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
                } else {
                    cm->setPriority(5);  // Set the priority of the message
                    cm->setRsuId(rsuId.c_str());
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
        // Check if the CarId is in the interestingIds list
        if (interestingIds.count(cm->getCarId()) > 0) {
            // If the RSU has a service
            if (rsuServices.count(cm->getRsuId()) > 0) {
                for (auto* service : rsuServices[cm->getRsuId()]) {
                    std::string serviceName = service->getServiceName();
                    // Check if current time is within service time
                    if (isServiceTime(service, simTime().dbl())) {
                        std::string message = "Providing service: " + serviceName;
                        cm->setWsmData(message.c_str());
                        cm->setWsmLength(strlen(message.c_str()));
                    }
                }
            }
            cm->setResponseTime(simTime());
            sendDown(cm->dup());
        }
        delete (cm);
    } else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}

void IntersectionApp::handleUpperMsg(cMessage* msg) {
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
        // Calculate RTT
        simtime_t rtt = simTime() - cm->getSendTime();
        EV << "RTT for car " << cm->getCarId() << " is " << rtt << "\n";

        // Find relevant interaction and update RTT
        for (auto& interaction : interactions) {
            if (interaction.getCarId() == std::string(cm->getCarId())
                    && interaction.getRsuId() == std::string(cm->getRsuId())) {
                interaction.setRTT(rtt.dbl());
            }
        }
    }
    delete (msg);
}

void IntersectionApp::handleLowerMsg(cMessage* msg) {
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    if (CarMessage* cm = dynamic_cast<CarMessage*>(wsm)) {
        simtime_t latency = simTime() - cm->getSendTime(); // Calculate latency
        // Check if the CarId is in the interestingIds list
        if (interestingIds.count(cm->getCarId()) > 0) {
            // If the RSU has a service
            if (rsuServices.count(cm->getRsuId()) > 0) {
                for (auto* service : rsuServices[cm->getRsuId()]) {
                    std::string serviceName = service->getServiceName();
                    // Check if current time is within service time
                    if (isServiceTime(service, simTime().dbl())) {
                        std::string message = "Providing service: " + serviceName;
                        cm->setWsmData(message.c_str());
                        cm->setWsmLength(strlen(message.c_str()));
                        // Log this interaction
                        logInteraction(cm->getCarId(), cm->getRsuId(),
                                serviceName, simTime().dbl(), latency);
                    }
                }
            }
            msgQueue.push(cm);
        }
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
    EV << "Received CarMessage from car " << cm->getSenderAddress() << ": " << cm->getHelloMsg() << "\n";
}

void IntersectionApp::exportMessageExchangeCounts() {
    std::ofstream outputFile("message_exchange.csv");
    outputFile << "CarId,RSUId,MessageCount,Timestamp\n";

    for (const auto& pair : messageExchangeCount) {
        std::string carId = pair.first.first;
        std::string rsuId = pair.first.second;
        int count = pair.second.first;
        std::string time = pair.second.second;

        outputFile << carId << "," << rsuId << "," << count << "," << time << "\n";
    }

    // Log the message queue
    outputFile << "\nMessage Queue:\n";
    while (!msgQueue.empty()) {
        CarMessage* cm = msgQueue.top();
        std::string carId = std::string(cm->getCarId());
        std::string rsuId = std::string(cm->getRsuId());
        int priority = cm->getPriority();
        outputFile << "CarId: " << carId << ", RSUId: " << rsuId << ", Priority: " << priority << "\n";
        msgQueue.pop();
    }
    outputFile.close();
}

void IntersectionApp::exportInteractionLog() {
    std::ofstream file("interactions.csv");
    file << "CarID,RSUID,ServiceName,InteractionTime\n";
    for (const auto &interaction : interactions) {
        file << interaction.getCarId() << "," << interaction.getRsuId() << ","
                << interaction.getServiceName() << ","
                << interaction.getInteractionTime() << ","
                << interaction.getLatency() << ","
                << interaction.getRTT() << "\n";
    }
    file.close();
}

std::map<std::string, std::vector<int>> IntersectionApp::readCSV(
        const std::string& filename) {
    // Open the file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file " + filename);
    }

    // Map to store RSU availability
    std::map<std::string, std::vector<int>> rsuAvailability;

    // Process each line
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);

        std::string carId, rsuId, messageCount;
        std::getline(ss, carId, ',');
        std::getline(ss, rsuId, ',');
        std::getline(ss, messageCount, ',');

        int timestamp;
        while (ss >> timestamp) {
            // Ignore the comma
            if (ss.peek() == ',')
                ss.ignore();

            // Add the timestamp to the rsuId's vector
            rsuAvailability[rsuId].push_back(timestamp);
        }

        // Sort the timestamps
        std::sort(rsuAvailability[rsuId].begin(), rsuAvailability[rsuId].end());
    }

    return rsuAvailability;
}

// Check if current time is within service time
bool IntersectionApp::isServiceTime(Service* service, double currentTime) {
    return currentTime >= service->getStartTime()
            && currentTime <= service->getEndTime();
}

// Log interaction details in a CSV file
void IntersectionApp::logInteraction(std::string carId, std::string rsuId,
        const std::string &serviceName, double interactionTime, simtime_t latency) {

    // Create a unique identifier for this interaction
    std::string interactionId = carId + "," + rsuId + "," + serviceName + ","
            + std::to_string(interactionTime) + "," + std::to_string(latency.dbl());

    // If this interaction hasn't been logged before, log it
    if (loggedInteractions.count(interactionId) == 0) {
        loggedInteractions.insert(interactionId);

        InteractionData data;
        data.setCarId(carId.c_str());
        data.setRsuId(rsuId.c_str());
        data.setLatency(latency);
        data.setServiceName(serviceName.c_str());
        data.setInteractionTime(interactionTime);
        interactions.push_back(data);
    }
}
