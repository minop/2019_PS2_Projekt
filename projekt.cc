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

using namespace ns3;

void runSim(double);

// global variables / simulation settings
bool logRobotCallback = false;
bool doNetanim = true; // TODO: change to false
int makeGraph = 0;
double simTime = 30.0;
Gnuplot2dDataset data;

// position allocators accessible from callbacks
bool returningHome = false;
Ptr<RandomRectanglePositionAllocator> waypointAllocator;
Ptr<RandomRectanglePositionAllocator> homeAllocator;

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
    // TODO: count the packets somewhere
}

static void changeRobotSpeed() {
    Config::Set("NodeList/21/$ns3::MobilityModel/$ns3::RandomWaypointMobilityModel/Speed", StringValue("ns3::ConstantRandomVariable[Constant=40]"));
}

static void changePingFrequency() {
    Config::Set("NodeList/21/ApplicationList/0/$ns3::OnOffApplication/OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.5]"));
}

static void doSimulation(bool olsrRouting, uint64_t dataRatekb) {
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
    if(olsrRouting) {
        OlsrHelper olsr;
        internet.SetRoutingHelper(olsr);
    }
    else {
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
            DataRateValue(DataRate(dataRatekb*1000)));
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
    apps.Stop(Seconds(simTime - 1));

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
    // TODO: should only be registered if some graphs are being made (to speed thing up)
    Config::ConnectWithoutContext("/NodeList/0/DeviceList/0/$ns3::CsmaNetDevice/MacRx", MakeCallback(&macRecievePacketCallback));

    Simulator::Schedule(Seconds(5.0), &changeRobotSpeed);
    Simulator::Schedule(Seconds(15.0), &changeRobotSpeed);

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

        runSim(simTime);
    } else {
        runSim(simTime);
    }
}

void runSim(double simTime) {
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
}

int main(int argc, char *argv[]) {
    // Simulation defaults are typically set before command line arguments are parsed.
    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

    // CommandLine arguments
    CommandLine cmd;
    cmd.AddValue("doNetanim", "Generate NetAnim file", doNetanim);
    cmd.AddValue("simTime", "Total simulation time", simTime);
    cmd.AddValue("robotCallbackLogging", "Enable logging of robot callback", logRobotCallback);
    cmd.AddValue("graph", "[0-9], which graph should be generated; 0 for none", makeGraph);
    cmd.Parse(argc, argv);

    // prvotne nastavenia v hl.funkcii
    Gnuplot graf("graf1.svg");
    graf.SetTerminal("svg");
    graf.SetTitle("Graf zavislosti mnozstva prijatych datovych paketov od casu");
    graf.SetLegend("Cas [s]","Mnozstvo prijatych paketov");
    graf.AppendExtra("set xrange[0:32]");
    //data.SetTitle ("strata udajov");
    data.SetStyle (Gnuplot2dDataset::LINES);
    //data.SetErrorBars(Gnuplot2dDataset::Y);
    
    // How many times will the simulation be run?
    uint64_t nRuns;
    if (makeGraph > 0 && makeGraph < 10)
        nRuns = 10;
    else if(makeGraph == 0)
        nRuns = 1;
    else {
        std::cerr << "makeGraph has to be from interval <0; 9>" << std::endl;
        return -1;
    }

    // Manage RNG seeds
    RngSeedManager seedManager;
    seedManager.SetRun(nRuns);

    // Default simulation parameters
    uint64_t dataRatekb = 5000; // in kilo bits
    bool olsrRouting = false;    // false = AODV
    
    // Perform simulations
    for (uint64_t i = 0; i < nRuns; i++) {
        doSimulation(olsrRouting, dataRatekb);
    }
    
    if(makeGraph){
        //zaverecne spustenie
        graf.AddDataset (data);
        std::ofstream plotFile ("graf" + std::to_string(makeGraph) + ".plt");
        graf.GenerateOutput (plotFile);
        plotFile.close ();
        if(system("gnuplot graf.plt"));
    }
    return 0;
}