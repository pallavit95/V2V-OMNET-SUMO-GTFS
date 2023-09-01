# V2V-OMNET-SUMO-GTFS
 
Service placement is a an optimization problem, which can be applied to any problem where two or metrics are supposed to be the variables that are to be optimized. 

In that way Service Placement Problem is a wrapper. It and can be wrapped around any problem statement like VM placement, static RSU in smart city, dynamic RSU in smart city, Multi-tenant architectural platform optimizations etc)
Service placement when considered in IoT and especially smart city domains for placing services, optimizes the energy consumed (one of the variables, most popular) and QoS (like latency, response time) parameter when interacting with the devices. This optimization problem when is generally done in systems that consider static RSUs. 

For dynamic RSUs the dynamicity in a smart city environment can be considered as that of moving vehicles. From that perspective, a V2V scenario and model that assumes the movements of the microscopic traffic model can be applied, where one vehicle can be considered a consumer and another the producer. The producer has OBUs installed on them, to make their range of interaction stronger. There are multiple configurations like master-slave, clustering of moving nodes etc which have been considered in literature and previous research work. 

There’s none to the best of my knowledge that considers the past data of public vehicles and considers a system where their future location is predicted and hence an interaction map or interaction time windows of these moving vehicles are considered. The following is a system that takes GTFS data of public buses in Dublin, and based on the predictions of the model, generates and interaction map. Since interaction map has to be generated using a formula or a set of rules, that are is unexplored and the interaction maps are generated based on the simulations instead. These simulations are then varied based on – how off – the prediction can be and its impact evaluated. 

The definition of a system is extremely critical before wrapping it around with a Service Placement Optimization problem.

The following defines a step-by-step process for the setup of all the tools and ide-s required for generating V2V simulations.

•	Omnet++ v5.3
•	SUMO 0.32
•	VEINS 4.7
•	INET 3.6.8
1)	In windows, download omnet++ v5.3 from https://omnetpp.org/download/old , the current version is 6.0.1, but these modules are not perfectly backward compatible or wholistically upgraded, so there are issues if the exact versions are not used together.
2)	Follow the basic setup guideline from Omnet++ installation guide. This is an exact process, and is not mentioned here to avoid discripencies.
3)	SUMO and older versions of SUMO can be downloaded and installed from here - https://sumo.dlr.de/docs/Downloads.php
4)	Veins 4.7 can be downloaded from here, and doesn’t have to be installed – just imported to omnet++ directory. https://veins.car2x.org/download/
5)	INET 3.6.8 would be downloaded automatically by the omnet ide on startup and imported to the workspace.

After the setup the project can be imported. There are 4 projects overall and they are slightly different from each other in terms of v2v setup in the following way:
V2v_app_5 – implements interaction of moving vehicle with static RSUS and records the message exchange between car-rsu throughout the simulation. 
V2v_app_5_1 – static RSU-RSU RTT is recorded by re-using the components of the above project.
V2v_app_6 – implements interaction of moving vehicle with other moving vehicle and records the message exchange between car-car throughout the simulation.
V2V_app_6_1 – car-car RTT is recorded by re-using the components of the above project. 

While an overview of OMNET++ and its use in different applications is available across multiple sources, a concise view of its most important files and properties that are used to create a simulation are missing to the best of my knowledge. Thus a brief view of the files is provided below along with information about the different properties.
The overall critical files are interacting with each other in the the following way:
a) The simulation starts and reads the .ini file to determine the setup.
b) Using the specified network in the .ini file, OMNeT++ constructs the network using the appropriate .ned file(s).
c) Once the network structure is in place, the simulation runs the behavior defined in the .cc files.
d) The simulation proceeds, with nodes (like vehicles and RSUs) interacting based on the logic in the .cc files, until the simulation ends (based on criteria like a time limit).

To visualize this, the .ini file serves as the blueprint, the .ned files as the architectural design, and the .cc files as the construction details of a building. In this analogy, The simulation then lives in this building, following the rules set in the code
 
Codebase Overview:

NED File Description

IntersectionApp
- Extends BaseWaveApplLayer, inheriting its capabilities for vehicular applications.
- @class(IntersectionApp); is a decorator indicating the associated C++ class.
- @display("i=block/app2"); sets the graphical display characteristics in the simulation environment.

IntersectionScenario
- Extends Scenario, which is a custom base scenario class.
- rsu[13]: RSU declares 13 Road-Side Units (RSUs) in the scenario.
- @display("bgb=1000,1000;bgl=3"); sets the background dimensions and grid line characteristics.

CarMessage extends WaveShortMessage, inheriting its features and adding more fields - helloMsg: A string containing a "Hello World" message, priority: An integer representing the priority of the message, rsuId: The ID of the RSU involved in the message exchange, carId: The ID of the car sending the message.

Header File (C++)

Inherited Class
- BaseWaveApplLayer: This is the base class for WAVE (Wireless Access in Vehicular Environments) application layers. It provides methods to handle lower and self-messages, as well as position updates.

Data Members
- RSU related
  - delayTimeRSU: The time delay before an RSU can send another message.
  - lastSentRSU: The last time an RSU sent a message.
  - RSUData: A vector storing IntersectMessage objects, probably containing data related to intersections.
- Vehicle related
  - delayTimeCars: The time delay before a car can send another message.
  - lastSent: The last time a car sent a message.
  - canGo: A flag to indicate whether the car can go through an intersection or not.
- General
  - isRSU: A boolean flag indicating if the current node is an RSU.
Static Data Members
- carDistances, carPositions, rsuPositions: Maps that store distances between cars, and the positions of cars and RSUs, respectively.
- messageExchangeCount: Stores the count and timestamps of messages exchanged between nodes.
- interestingIds: A set of IDs that the application is particularly interested in.
- msgQueue: A priority queue that sorts incoming CarMessage objects by priority.
- car1Speeds,bus0Speeds, etc.: Maps that store the speeds of specific vehicles at different simulation times.

Methods
- Initialize and Finish
  - initialize(int stage): Sets initial parameters and flags.
  - finish(): Called at the end of the simulation to perform any required cleanup or data export.
- Position Updates
  - handlePositionUpdate(cObject* obj): Called when the position of a car or RSU changes.
- Message Handling
  - handleSelfMsg(cMessage* msg): Handles self-sent messages.
  - handleLowerMsg(cMessage *msg): Handles messages received from lower layers.
  - populateCMBrodcast(CarMessage* cm): Populates a CarMessage before it's sent.
  - onCarMessage(CarMessage* cm): Custom handling for received CarMessage objects.
- Utility Functions
  - calculateDistance(const Coord& pos1, const Coord& pos2): Calculates the distance between two coordinates.
  - exportMessageExchangeCounts(): Exports message exchange data to a CSV file.

Note:
WaveShortMessage is a predefined class for WAVE (Wireless Access in Vehicular Environments) short messages in the Veins framework. It's designed to encapsulate the data and attributes associated with short messages sent between vehicles or between vehicles and infrastructure. It typically includes attributes like sender and receiver addresses, data payload, etc.

BaseWaveApplLayer :This is the base class for WAVE application layers in the Veins framework. The class defines the basic functionality required for applications that need to send and receive WAVE short messages. It provides essential methods for handling messages, including methods to deal with messages from the network layer (handleLowerMsg) and self-messages (handleSelfMsg), as well as methods for updating positions (handlePositionUpdate).

Implementation Overview:
1.	Initialization (initialize)
During the initialization phase, the simulation identifies whether the node is a Road-Side Unit (RSU) or a vehicle.

2.	Position Update (handlePositionUpdate)
This method is triggered every time a vehicle moves. It updates the position of the car in the carPositions map. It also calculates the distance to every other car and updates this in the carDistances map.

3.	Message Sending
If the distance between two cars is less than or equal to 40 units, a new CarMessage is created. The priority of the message is set based on the vehicle ID. If the channel is busy (dataOnSch), the message is scheduled for later sending. Otherwise, it's sent immediately.

4.	Message Receiving (handleLowerMsg)
Received messages are placed in a priority queue (msgQueue). The message with the highest priority is then processed by the onCarMessage function.

5.	Self Messages (handleSelfMsg)
This function is responsible for managing messages that are scheduled for later. When the message is received (from "self"), it is sent down to the lower layers for actual transmission.

6.	Message Logging (onCarMessage)
Logs the received CarMessage. The messageExchangeCount map is updated with the new message data.

7.	Simulation End (finish)
At the end of the simulation, all logged data, such as message counts and vehicle speeds, are exported to CSV files.

INI File Description
General Settings
cmdenv-express-mode, cmdenv-autoflush, etc.: These settings configure how the simulation runs and how it interacts with the user.
network = IntersectionScenario: Specifies that the network configuration is based on the IntersectionScenario NED file.
sim-time-limit = 1000s: The simulation will run for 1000 seconds.
Simulation Parameters
*.scalar-recording = true and *.vector-recording = true: Enable the recording of scalar and vector data, useful for post-simulation analysis.
*.playgroundSizeX, *.playgroundSizeY, *.playgroundSizeZ: Set the dimensions of the simulation playground.
Annotation Parameters
*.annotations.draw = true: Enables drawing of annotations, probably for debugging or visualization.
TraCIScenarioManager Parameters
*.manager.updateInterval = 1s: The TraCI (Traffic Control Interface) updates every second, which aligns with the 1-second update rate in the code.
RSU Settings
*.rsu[*].mobility.x/y/z: These set the positions of the Road-Side Units (RSUs) in the simulated environment.
*.rsu[*].applType = "IntersectionApp": Specifies that the application type running on the RSUs is IntersectionApp, aligning with the C++ class.
11p Specific Parameters
*.connectionManager.sendDirect = true: Enables direct sending of messages.
*.connectionManager.maxInterfDist = 40m: Sets the maximum interference distance to 40m, aligning with the 40-unit distance check in the code.
WaveAppLayer
*.node[*].applType = "IntersectionApp": Specifies that the nodes (vehicles) will run the IntersectionApp application.
*.node[*].appl.sendBeacons = false: Vehicles won't send beacons by default, focusing on data messages.
Mobility
*.node[*].veinsmobilityType.debug = true: Enables debugging for mobility, useful for tracking position updates.
Config Sections
[Config Default]: The default configuration.
[Config WithBeaconing]: Enables beaconing for RSUs and nodes.
[Config WithChannelSwitching]: Enables channel switching, allowing for the use of service channels in addition to the control channel.
Connections to Code
RSU and Node Positions: The .ini file directly sets the positions of RSUs, which would be read into rsuPositions in the code.
Update Intervals: The update interval in the .ini file (*.manager.updateInterval = 1s) corresponds to how often handlePositionUpdate would likely be called in the code.
Application Types: The application type specified for both RSUs and nodes is IntersectionApp, tying back to the main class in the C++ code.
Max Interference Distance: The maximum interference distance (*.connectionManager.maxInterfDist = 40m) aligns with the distance check (if (distance <= 40)) in the handlePositionUpdate method in the code.

To start the simulation, right-click on the omnet.ini and the run the simulation in WithChannelSwitching mode. Sumo-gui also needs to be started in the background, and can be started with this command 
python "path to veins-4.7.1\sumo-launchd.py" -vv -c sumo-gui
Ending the simulation generates the csv files in the project structure itself, and they can then be used to do analysis.


End.
