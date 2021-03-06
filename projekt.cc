#include "ns3/command-line.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/qos-txop.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/csma-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/gnuplot.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/aodv-helper.h"
#include <math.h>

using namespace ns3;

void runSim(double);
void fillGnuplotData(std::vector<int> meassurements[]);
void fillGnuplotData(std::vector<double> meassurements[]);
void fillGnuplotData(std::vector<int> meassurements[], std::vector<double> xValues);
void fillGnuplotData(std::vector<double> meassurements[], std::vector<double> xValues);
void aggregatePacketCount(std::vector<double> packetArrivalTimes, std::vector<int> meassurementsArray[], uint64_t index);

// global variables / simulation settings
bool logRobotCallback = false;
bool doNetanim = false;
int makeGraph = 0;
int simTime = 30;
Gnuplot2dDataset data;
Gnuplot2dDataset errorBars;

// Application packets meassurments
int packetsReceived = 0;
std::vector<double> arrivalTimes = {};
std::vector<int> packetsPerSec[10];

// all packets meassurements
int allPacketsRecieved = 0;
std::vector<double> allPacketsArrivalTimes = {};
std::vector<int> allPacketsMeassurements[10];

// graph 9
std::vector<double> bitRates = {};

// position allocators accessible from callbacks
bool returningHome = false;
Ptr<RandomRectanglePositionAllocator> waypointAllocator;
Ptr<RandomRectanglePositionAllocator> homeAllocator;

void packetReceivedCallback(Ptr< const Packet > packet, const Address &address) {
    packetsReceived++;
    arrivalTimes.push_back(Simulator::Now().GetSeconds());
}

void returnHomeCallback(Ptr< const MobilityModel> mobModel) {
    Vector pos = mobModel->GetPosition();

    if (logRobotCallback)
        std::cout << "[" << Simulator::Now().GetSeconds() << "s] " << pos.x << "; " << pos.y << std::endl;

    if (!returningHome) {
        // is the robot out of bounds?
        if (pos.x < 0.0 || pos.x > 100.0 || pos.y < 0.0 || pos.y > 80.0) {
            returningHome = true;
            Config::Set("/NodeList/21/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/PositionAllocator", PointerValue(homeAllocator));

            if (logRobotCallback)
                std::cout << "[" << Simulator::Now().GetSeconds() << "s] " << "robot has left AP reach and will return home." << std::endl;
        }
    } else {
        // has the robot returned home?
        if (pos.x > 49.5 && pos.x < 50.5 && pos.y > 49.5 && pos.y < 50.5) {
            returningHome = false;
            Config::Set("/NodeList/21/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/PositionAllocator", PointerValue(waypointAllocator));

            if (logRobotCallback)
                std::cout << "[" << Simulator::Now().GetSeconds() << "s] " << "robot has returned home and will begin roaming again." << std::endl;
        }
    }
}

void macRecievePacketCallback(Ptr< const Packet> packet) {
    ++allPacketsRecieved;
    allPacketsArrivalTimes.push_back(Simulator::Now().GetSeconds());
}

static void changeRobotSpeed() {
    Config::Set("NodeList/21/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/Speed", StringValue("ns3::ConstantRandomVariable[Constant=40]"));
}

static void changePingFrequency() {
    Config::Set("NodeList/21/ApplicationList/0/$ns3::OnOffApplication/OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.5]"));
}

static void doSimulation(bool olsrRouting, uint64_t dataRatekb, double simulationTime) {
    // Server Node
    NodeContainer serverNodes;
    serverNodes.Create(1);
    Ptr<Node> server = serverNodes.Get(0);

    // AP Nodes
    NodeContainer apNodes;
    apNodes.Create(20);

    // UAV Node
    NodeContainer robotNodes;
    robotNodes.Create(1);
    Ptr<Node> robot = robotNodes.Get(0);

    // helper containers to install nodes more easily
    NodeContainer ethernetNodes;
    ethernetNodes.Add(server);
    ethernetNodes.Add(apNodes);

    NodeContainer wifiNodes;
    wifiNodes.Add(apNodes);
    wifiNodes.Add(robot);

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Construct the wifi network                                            //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////
    // Create the wifi net devices and install them into the nodes in our container
    WifiHelper wifi;
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
            "DataMode", StringValue("OfdmRate54Mbps"));
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
            "MaxRange", DoubleValue(15.0));
    wifiPhy.SetChannel(wifiChannel.Create());
    NetDeviceContainer wifiDevices = wifi.Install(wifiPhy, mac, wifiNodes);

    // Add the IPv4 protocol stack to the nodes in our container
    InternetStackHelper internet;
    if (olsrRouting) {
        OlsrHelper olsr;
        internet.SetRoutingHelper(olsr);
    } else {
        AodvHelper aodv;
        internet.SetRoutingHelper(aodv);
    }
    internet.Install(wifiNodes);

    // Assign IPv4 addresses to the device drivers (actually to the associated IPv4 interfaces) we just created.
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase("192.168.0.0", "255.255.255.0");
    ipAddrs.Assign(wifiDevices);

    // APs Mobility
    MobilityHelper apMobility;
    apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(10.0),
            "MinY", DoubleValue(10.0),
            "DeltaX", DoubleValue(20.0),
            "DeltaY", DoubleValue(20.0),
            "GridWidth", UintegerValue(5),
            "LayoutType", StringValue("RowFirst"));
    apMobility.Install(apNodes);

    // server Mobility
    MobilityHelper serverMobility;
    serverMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    serverMobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(200.0),
            "MinY", DoubleValue(50.0));
    serverMobility.Install(server);

    // robot Mobility
    Ptr<UniformRandomVariable> allocatorRandVar = CreateObject<UniformRandomVariable>();
    allocatorRandVar->SetAttribute("Min", DoubleValue(-30.0));
    allocatorRandVar->SetAttribute("Max", DoubleValue(130.0));

    waypointAllocator = CreateObject<RandomRectanglePositionAllocator>();
    waypointAllocator->SetX(allocatorRandVar);
    waypointAllocator->SetY(allocatorRandVar);

    Ptr<ConstantRandomVariable> homeRandVar = CreateObject<ConstantRandomVariable>();
    homeRandVar->SetAttribute("Constant", DoubleValue(50.0));

    homeAllocator = CreateObject<RandomRectanglePositionAllocator>();
    homeAllocator->SetX(homeRandVar);
    homeAllocator->SetY(homeRandVar);

    MobilityHelper robotMobility;
    robotMobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=20.0]"),
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
            "PositionAllocator", PointerValue(waypointAllocator));

    robotMobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(50.0),
            "MinY", DoubleValue(50.0));
    robotMobility.Install(robot);
    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Construct the LAN                                                     //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    // Reset the address base-- all of the CSMA networks will be in the "172.16 address space
    ipAddrs.SetBase("172.16.0.0", "255.255.255.0");

    // Create the CSMA net devices and install them into the nodes in our collection.
    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate",
            DataRateValue(DataRate(dataRatekb * 1000)));
    csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
    NetDeviceContainer lanDevices = csma.Install(ethernetNodes);

    // Add the IPv4 protocol stack to the new LAN nodes (only the server is new!)
    internet.Install(serverNodes);
    // Assign IPv4 addresses to the device drivers (actually to the associated IPv4 interfaces) we just created.
    ipAddrs.Assign(lanDevices);

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Application configuration                                             //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    // Create the OnOff application to send UDP datagrams of size
    // 210 bytes at a rate of 10 Kb/s, between two nodes
    // Data is sent from the robot to the server

    uint16_t port = 9; // Discard port (RFC 863)

    // Let's fetch the IP address of the last node, which is on Ipv4Interface 1
    Ipv4Address remoteAddr = server->GetObject<Ipv4> ()->GetAddress(1, 0).GetLocal();

    OnOffHelper onoff("ns3::UdpSocketFactory",
            Address(InetSocketAddress(remoteAddr, port)));

    ApplicationContainer apps = onoff.Install(robot);
    apps.Start(Seconds(3));
    apps.Stop(Seconds(simulationTime - 1));

    // Create a packet sink to receive these packets
    PacketSinkHelper sink("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), port));
    apps = sink.Install(server);
    apps.Start(Seconds(3));

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Callback configuration                                                //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    Config::ConnectWithoutContext("/NodeList/21/$ns3::MobilityModel/CourseChange", MakeCallback(&returnHomeCallback));
    if (makeGraph >= 1 && makeGraph <= 8)
        Config::ConnectWithoutContext("/NodeList/0/ApplicationList/0/$ns3::PacketSink/Rx", MakeCallback(&packetReceivedCallback));
    if (makeGraph >= 5 && makeGraph <= 9)
        Config::ConnectWithoutContext("/NodeList/0/DeviceList/0/$ns3::CsmaNetDevice/MacRx", MakeCallback(&macRecievePacketCallback));

    Simulator::Schedule(Seconds(5.0), &changeRobotSpeed);
    Simulator::Schedule(Seconds(15.0), &changePingFrequency);

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // NetAnim                                                               //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    if (doNetanim) {
        AnimationInterface anim("netanim.xml");

        // APs
        for (int i = 0; i < apNodes.GetN(); ++i) {
            anim.UpdateNodeColor(apNodes.Get(i), 0, 0, 0);
            anim.UpdateNodeDescription(apNodes.Get(i), "");
        }
        // server
        anim.UpdateNodeColor(serverNodes.Get(0), 0, 255, 0);
        anim.UpdateNodeDescription(serverNodes.Get(0), "Server");
        // robot
        anim.UpdateNodeColor(robotNodes.Get(0), 255, 0, 0);
        anim.UpdateNodeDescription(robotNodes.Get(0), "Robot");

        anim.EnablePacketMetadata();

        runSim(simulationTime);
    } else {
        runSim(simulationTime);
    }
}

void runSim(double simulationTime) {
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();
    Simulator::Destroy();
}

int main(int argc, char *argv[]) {
    // Simulation defaults are typically set before command line arguments are parsed.
    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

    // CommandLine arguments
	double st = 30.0;
    CommandLine cmd;
    cmd.AddValue("anim", "Generate NetAnim file", doNetanim);
    cmd.AddValue("simulTime", "Total simulation time", st);
    cmd.AddValue("robotCallbackLogging", "Enable logging of robot callback", logRobotCallback);
    cmd.AddValue("graph", "[0-9], which graph should be generated; 0 for none", makeGraph);
    cmd.Parse(argc, argv);
	simTime = (int) st;

    // prvotne nastavenia v hl.funkcii
    Gnuplot graf("graf" + std::to_string(makeGraph) + ".svg");
    if (makeGraph) {
        graf.SetTerminal("svg");
        switch (makeGraph) {
            case 1:
                graf.SetTitle("Graf zavislosti mnozstva prijatych datovych paketov od casu");
                graf.SetLegend("Cas [s]", "Mnozstvo prijatych paketov");
                data.SetTitle("prijate pakety (OLSR 5Mbit)");
                break;
            case 2:
                graf.SetTitle("Graf zavislosti mnozstva prijatych datovych paketov od casu");
                graf.SetLegend("Cas [s]", "Mnozstvo prijatych paketov");
                data.SetTitle("prijate pakety (OLSR 5kbit)");
                break;
            case 3:
                graf.SetTitle("Graf zavislosti mnozstva prijatych datovych paketov od casu");
                graf.SetLegend("Cas [s]", "Mnozstvo prijatych paketov");
                data.SetTitle("prijate pakety (AODV 5Mbit)");
                break;
            case 4:
                graf.SetTitle("Graf zavislosti mnozstva prijatych datovych paketov od casu");
                graf.SetLegend("Cas [s]", "Mnozstvo prijatych paketov");
                data.SetTitle("prijate pakety (AODV 5kbit)");
                break;
            case 5:
                graf.SetTitle("Graf zavislosti podielu prijatych datovych paketov ku vsetkym paketom v case");
                graf.SetLegend("Cas [s]", "podiel datove pakety ku vsetkym paketom");
                data.SetTitle("goodput (OLSR 5Mbit)");
                break;
            case 6:
                graf.SetTitle("Graf zavislosti podielu prijatych datovych paketov ku vsetkym paketom v case");
                graf.SetLegend("Cas [s]", "podiel datove pakety ku vsetkym paketom");
                data.SetTitle("goodput (OLSR 5kbit)");
                break;
            case 7:
                graf.SetTitle("Graf zavislosti podielu prijatych datovych paketov ku vsetkym paketom v case");
                graf.SetLegend("Cas [s]", "podiel datove pakety ku vsetkym paketom");
                data.SetTitle("goodput (AODV 5Mbit)");
                break;
            case 8:
                graf.SetTitle("Graf zavislosti podielu prijatych datovych paketov ku vsetkym paketom v case");
                graf.SetLegend("Cas [s]", "podiel datove pakety ku vsetkym paketom");
                data.SetTitle("goodput (AODV 5kbit)");
                break;
            case 9:
                graf.SetTitle("Graf zavislosti poctu prijatych paketov od rychlosti ethernetovej linky");
                graf.SetLegend("Rychlost [bit/s]", "pocet prijatych paketov za celu simulaciu");
                data.SetTitle("pocet paketov (AODV)");
                break;
        }

        if (makeGraph >= 1 && makeGraph <= 8)
            graf.AppendExtra("set xrange[0:32]");
        if (makeGraph == 9) {
            graf.AppendExtra("set logscale x");
            graf.AppendExtra("set xrange[1000:5000000]");
        }

        data.SetStyle(Gnuplot2dDataset::LINES); // use LINES_POINTS if you want to have errorbars with the line in one dataset
        // Two lines because if the errorbars have the same color as the line it looks ugly
        errorBars.SetTitle("smerodajna odchylka");
        errorBars.SetStyle(Gnuplot2dDataset::POINTS);
        errorBars.SetErrorBars(Gnuplot2dDataset::Y);
    }

    // How many times will the simulation be run?
    uint64_t nRuns;
    if (makeGraph > 0 && makeGraph < 10)
        nRuns = 10;
    else if (makeGraph == 0)
        nRuns = 1;
    else {
        std::cerr << "makeGraph has to be from interval <0; 9>" << std::endl;
        return -1;
    }

    // Manage RNG seeds
    RngSeedManager seedManager;
    seedManager.SetRun(nRuns);

    // Simulation parameters
    uint64_t dataRatekb;
    bool olsrRouting;
    switch (makeGraph) {
        case 1:
        case 5:
        default:
            dataRatekb = 5000; // in kilo bits
            olsrRouting = true; // false = AODV
            break;
        case 2:
        case 6:
            dataRatekb = 5;
            olsrRouting = true;
            break;
        case 3:
        case 7:
            dataRatekb = 5000;
            olsrRouting = false;
            break;
        case 4:
        case 8:
            dataRatekb = 5;
            olsrRouting = false;
            break;
        case 9:
            olsrRouting = false;
            break;
    }

    int outerRuns = 1;
    if (makeGraph == 9) {
        outerRuns = 8;
        bitRates.clear();
    }

    // Perform simulations
    for (int outer = 0; outer < outerRuns; ++outer) {
        if (makeGraph == 9) {
            dataRatekb = pow(10.0, 0.5 * outer); // evenly spaces speeds (on log scale) from ~1kbit to ~5Mbit
            bitRates.push_back(dataRatekb*1000);
        }

        for (uint64_t i = 0; i < nRuns; i++) {
            if (makeGraph >= 1 && makeGraph <= 8) {
                packetsReceived = 0;
                arrivalTimes.clear();
            }
            if (makeGraph >= 5 && makeGraph <= 9) {
                allPacketsRecieved = 0;
                allPacketsArrivalTimes.clear();
            }

            doSimulation(olsrRouting, dataRatekb, st);

            if (makeGraph >= 1 && makeGraph <= 8)
                aggregatePacketCount(arrivalTimes, packetsPerSec, i);
            if (makeGraph >= 5 && makeGraph <= 8)
                aggregatePacketCount(allPacketsArrivalTimes, allPacketsMeassurements, i);
            if (makeGraph == 9)
                allPacketsMeassurements[i].push_back(allPacketsArrivalTimes.size());
        }
    }

    // add the correct data to the graf
    if (makeGraph >= 1 && makeGraph <= 4)
        fillGnuplotData(packetsPerSec);

    if (makeGraph >= 5 && makeGraph <= 8) {
        std::vector<double> quotient[10];
        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < allPacketsMeassurements[i].size(); ++j) {
                if (j < packetsPerSec[i].size() && allPacketsMeassurements[i][j] != 0) {
                    quotient[i].push_back(packetsPerSec[i][j] / (double) allPacketsMeassurements[i][j]);
                } else {
                    quotient[i].push_back(0);
                }
            }
        }
        fillGnuplotData(quotient);
    }

    if (makeGraph == 9)
        fillGnuplotData(allPacketsMeassurements, bitRates);

    if (makeGraph) {
        // zaverecne spustenie
        graf.AddDataset(errorBars);
        graf.AddDataset(data);
        std::ofstream plotFile("graf" + std::to_string(makeGraph) + ".plt");
        graf.GenerateOutput(plotFile);
        plotFile.close();
        std::string pltName = "gnuplot graf" + std::to_string(makeGraph) + ".plt";
        if (system(pltName.c_str()));
    }
    return 0;
}

void aggregatePacketCount(std::vector<double> packetArrivalTimes, std::vector<int> meassurementsArray[], uint64_t index) {
    int j = 0;
    do{
        for (int k = 0; k < simTime; k++) {
            if (j == 0)
                meassurementsArray[index].push_back(0);
            if(packetArrivalTimes.size() > 0){
                if (packetArrivalTimes[j] >= k && packetArrivalTimes[j] < k + 1) {
                    meassurementsArray[index][k]++;
                    if (j != 0)
                        break;
                }
            }
        }
        j++;
    } while(j < packetArrivalTimes.size());
}

void fillGnuplotData(std::vector<int> meassurements[]) {
    // convert the integers to doubles and call the other function
    std::vector<double> doubles[10];
    for (int i = 0; i < 10; ++i) {
        doubles[i] = std::vector<double>(meassurements[i].begin(), meassurements[i].end());
    }
    fillGnuplotData(doubles);
}

void fillGnuplotData(std::vector<double> meassurements[]) {
    std::vector<double> xVals;
    for (int i = 0; i < meassurements[0].size(); ++i) {
        xVals.push_back((double) i);
    }
    fillGnuplotData(meassurements, xVals);
}

void fillGnuplotData(std::vector<int> meassurements[], std::vector<double> xValues) {
    std::vector<double> doubles[10];
    for (int i = 0; i < 10; ++i) {
        doubles[i] = std::vector<double>(meassurements[i].begin(), meassurements[i].end());
    }
    fillGnuplotData(doubles, xValues);
}

void fillGnuplotData(std::vector<double> meassurements[], std::vector<double> xValues) {
    for (int i = 0; i < meassurements[0].size(); ++i) {
        double average = 0.0;
        for (int j = 0; j < 10; ++j) {
            average += meassurements[j].at(i);
        }
        average /= 10;

        double deviation = 0.0;
        for (int j = 0; j < 10; ++j) {
            double k = meassurements[j].at(i) - average;
            deviation += k*k;
        }
        deviation /= 10;
        deviation = sqrt(deviation);

        data.Add(xValues[i], average);
        errorBars.Add(xValues[i], average, deviation);
    }
}