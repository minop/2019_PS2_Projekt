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

using namespace ns3;

void runSim(double);

static void changeRobotSpeed(){
    Config::Set("NodeList/21/$ns3::MobilityModel/$ns3::RandomWalk2dMobilityModel/Speed",StringValue("ns3::UniformRandomVariable[Min=1000.0|Max=5000.0]"));
    std::cout << "robot speed chaged!";
}

int main(int argc, char *argv[]) {
    // Local variables / Simulation properties
    uint32_t backboneNodes = 10;
    uint32_t infraNodes = 2;
    uint32_t lanNodes = 2;
    uint32_t stopTime = 20;
    bool useCourseChangeCallback = false;

    // Simulation defaults are typically set before command line arguments are parsed.
    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

    CommandLine cmd;
    cmd.AddValue("backboneNodes", "number of backbone nodes", backboneNodes);
    cmd.AddValue("infraNodes", "number of leaf nodes", infraNodes);
    cmd.AddValue("lanNodes", "number of LAN nodes", lanNodes);
    cmd.AddValue("stopTime", "simulation stop time (seconds)", stopTime);
    cmd.AddValue("useCourseChangeCallback", "whether to enable course change tracing", useCourseChangeCallback);

    // The system global variables and the local values added to the argument system can be overridden by command line arguments by using this call.
    cmd.Parse(argc, argv);
    
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
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    NetDeviceContainer wifiDevices = wifi.Install(wifiPhy, mac, wifiNodes);

    // TODO: maybe different routing? (AODV?)
    // We enable OLSR (which will be consulted at a higher priority than the global routing) on the backbone ad hoc nodes
    OlsrHelper olsr;
    // Add the IPv4 protocol stack to the nodes in our container
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr); // has effect on the next Install ()
    internet.Install(wifiNodes);

    // Assign IPv4 addresses to the device drivers (actually to the associated IPv4 interfaces) we just created.
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase("192.168.0.0", "255.255.255.0");
    ipAddrs.Assign(wifiDevices);

    // APs Mobility
    MobilityHelper apMobility;
    // TODO: final physical layout of the nodes
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
    // TODO: randomWaypointModel
    MobilityHelper robotMobility;
    robotMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
            "Bounds", RectangleValue(Rectangle(0, 100, 0, 100)));
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
            DataRateValue(DataRate(5000000)));
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
    apps.Stop(Seconds(stopTime - 1));

    // Create a packet sink to receive these packets
    PacketSinkHelper sink("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), port));
    apps = sink.Install(server);
    apps.Start(Seconds(3));

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Tracing configuration                                                 //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    // TODO: remove? change?
    // Let's set up some ns-2-like ascii traces, using another helper class
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("mixed-wireless.tr");
    wifiPhy.EnableAsciiAll(stream);
    csma.EnableAsciiAll(stream);
    internet.EnableAsciiIpv4All(stream);

    // Csma captures in non-promiscuous mode
    csma.EnablePcapAll("mixed-wireless", false);
    // pcap captures on the backbone wifi devices
    wifiPhy.EnablePcap("mixed-wireless", wifiDevices, false);
    // pcap trace on the application data sink
    wifiPhy.EnablePcap("mixed-wireless", server->GetId(), 0);

    // TODO: proper callbacks
    if (useCourseChangeCallback == true) {
        //Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback(&CourseChangeCallback));
    }
    
    Simulator::Schedule (Seconds (5.0), &changeRobotSpeed);

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // NetAnim                                                               //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////
    
    AnimationInterface anim("netanim.xml");

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Run simulation                                                        //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    //NS_LOG_INFO("Run Simulation.");
    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();
    Simulator::Destroy();

    /*
    bool doNetanim = false;
    double simTime = 10.0;

    // CommandLine arguments
    CommandLine cmd;
    cmd.AddValue("doNetanim", "Should NetAnim file be generated", doNetanim);
    cmd.AddValue("simTime", "Total simulation time", simTime);
    cmd.Parse(argc, argv);

    // Netanim
    if (doNetanim) {
        AnimationInterface anim("netanim.xml");
        
        // APs
        for (int i = 0; i < apNodes.GetN(); ++i) {
            anim.UpdateNodeColor(apNodes.Get(i), 0, 0, 0);
            anim.UpdateNodeDescription(apNodes.Get(i), "");
        }
        // server
        anim.UpdateNodeColor(server, 0, 255, 0);
        anim.UpdateNodeDescription(server, "Server");
        // robot
        anim.UpdateNodeColor(robot, 255, 0, 0);
        anim.UpdateNodeDescription(robot, "Robot");
        
        anim.EnablePacketMetadata();
        
        runSim(simTime);
    }
    else {
        runSim(simTime);
    }
     */
    return 0;
}

void runSim(double simTime) {
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
}