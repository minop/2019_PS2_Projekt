#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/netanim-module.h"
#include "ns3/olsr-helper.h"

using namespace ns3;

void runSim(double);

int
main(int argc, char *argv[]) {
    bool doNetanim = false;
    double simTime = 10.0;

    // CommandLine arguments
    CommandLine cmd;
    cmd.AddValue("doNetanim", "Should NetAnim file be generated", doNetanim);
    cmd.AddValue("simTime", "Total simulation time", simTime);
    cmd.Parse(argc, argv);


    // Server Node
    NodeContainer snc;
    snc.Create(1);
    Ptr<Node> server = snc.Get(0);

    // AP Nodes
    NodeContainer apNodes;
    apNodes.Create(20);

    // UAV Node
    NodeContainer rnc;
    rnc.Create(1);
    Ptr<Node> robot = rnc.Get(0);

    // helper containers to install nodes more easily
    NodeContainer ethernetNodes;
    ethernetNodes.Add(server);
    ethernetNodes.Add(apNodes);

    NodeContainer wifiNodes;
    wifiNodes.Add(apNodes);
    wifiNodes.Add(robot);

    // Setup physical layer
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    wifiPhy.Set ("RxGain", DoubleValue (-10) );
    wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel (wifiChannel.Create ());
    
    // Set mac layer  
    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");
    mac.SetType("ns3::StaWifiMac",
            "Ssid", SsidValue(ssid),
            "ActiveProbing", BooleanValue(false));

    // Wifi connection from robot to APs
    // TODO: wifi attributes
    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");
    NetDeviceContainer wifiDevices;
    wifiDevices = wifi.Install(wifiPhy, mac, wifiNodes);
    
    // Ethernet connection from APs to server
    CsmaHelper ethernet;
    // TODO: ethernet attributes
    ethernet.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    ethernet.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

    NetDeviceContainer ethernetDevices;
    ethernetDevices = ethernet.Install(ethernetNodes);

    // AP Mobility
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
    // TODO: some mobility
    MobilityHelper robotMobility;
    robotMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
            "Bounds", RectangleValue(Rectangle(0, 100, 0, 100)));
    robotMobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(50.0),
            "MinY", DoubleValue(50.0));
    robotMobility.Install(robot);

    // Enable OLSR
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;

    Ipv4ListRoutingHelper list;
    list.Add (staticRouting, 0);
    list.Add (olsr, 10);

    // InternetStack
    InternetStackHelper stack;
    stack.Install(server);
    stack.Install(apNodes);
    stack.Install(robot);


    // Addresses
    Ipv4AddressHelper address;

    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ethernetInterfaces;
    ethernetInterfaces = address.Assign(ethernetDevices);

    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer wifiInterfaces;
    wifiInterfaces = address.Assign(wifiDevices);


    // Application
    uint16_t port = 9;
    OnOffHelper onoff ("ns3::UdpSocketFactory",
                     Address (InetSocketAddress(wifiInterfaces.GetAddress(12), port)));

    ApplicationContainer apps = onoff.Install(robot);
    apps.Start (Seconds (3));
    apps.Stop (Seconds (simTime - 1));

    // Create a packet sink to receive these packets
    PacketSinkHelper sink ("ns3::UdpSocketFactory",
                         InetSocketAddress (Ipv4Address::GetAny(), port));
    apps = sink.Install (server);
    apps.Start (Seconds (3));
    
    /*
    UdpServerHelper serverHelp;

    NodeContainer testServerNode = NodeContainer(apNodes.Get(12));
    
    ApplicationContainer serverApps = serverHelp.Install(testServerNode);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(20.0));

    UdpClientHelper client((Address)wifiInterfaces.GetAddress(12));
    client.SetAttribute("MaxPackets", UintegerValue(1));
    client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    client.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = client.Install(rnc);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(10.0));
    */

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
    return 0;
}

void runSim(double simTime) {
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
}