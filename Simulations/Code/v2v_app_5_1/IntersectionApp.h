#ifndef INTERSECTIONAPP_H_
#define INTERSECTIONAPP_H_

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "IntersectMessage_m.h"
#include "RSUMessage_m.h"
#include "CarMessage_m.h"
#include "Service_m.h"
#include "InteractionData_m.h"
#include "RSUInteraction_m.h"
//for debugging
#include <iostream>

class IntersectionApp: public BaseWaveApplLayer {
public:
    virtual void initialize(int stage);
    virtual void finish();
protected:
    ~IntersectionApp();
    //RSU
    simtime_t delayTimeRSU = 0;
    simtime_t lastSentRSU; //last time RSU sent msg
    std::vector<IntersectMessage*> RSUData;

    //Vehicles
    simtime_t delayTimeCars = 0;
    simtime_t lastSent; // the last time car sent a message
    bool canGo; //if the car can go through intersection
    virtual void handlePositionUpdate(cObject* obj);

    //General
    bool isRSU; //if it is an RSU or a car
    virtual void handleSelfMsg(cMessage* msg);
    virtual void handleLowerMsg(cMessage *msg);



    //adding new
    virtual void populateCMBrodcast(CarMessage* cm);
    virtual void onCarMessage(CarMessage* cm);
    double calculateDistance(const Coord& pos1, const Coord& pos2);
    static std::map<std::pair<std::string, std::string>, double> carDistances;
    static std::map<std::string, Coord> carPositions;
    static std::map<std::string, Coord> rsuPositions;
    static std::map<std::pair<std::string, std::string>,
            std::pair<int, std::string>> messageExchangeCount;
    static std::set<std::string> interestingIds;

    struct ComparePriority {
        bool operator()(const CarMessage* cm1, const CarMessage* cm2) const {
            return cm1->getPriority() < cm2->getPriority();
        }
    };

    static std::priority_queue<CarMessage*, std::vector<CarMessage*>,
            ComparePriority> msgQueue;

    static std::map<std::string, std::vector<int>> rsuAvailability;

    static std::map<std::string, std::vector<Service*>> rsuServices;

    std::map<std::string, std::vector<int>> readCSV(const std::string& filename);

    bool isServiceTime(Service* service, double currentTime);
    static void logInteraction(std::string carId, std::string rsuId, const std::string &serviceName, double interactionTime, simtime_t latency);

    static std::vector<InteractionData> interactions;

    static std::set<std::string> loggedInteractions;

    // Key: pair of RSU IDs, Value: pair of send and receive times
    std::map<std::pair<std::string, std::string>, std::pair<simtime_t, simtime_t>> rsuMessageTimes;

    static std::vector<RSUInteraction> rsuInteractions;

    virtual void exportRSUInteractionLog();


};

#endif /* TUTORIALAPPL_H_ */
