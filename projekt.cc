#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"

using namespace ns3;

int
main(int argc, char *argv[]) {
    /*
  bool verbose = true;
  uint32_t nCsma = 3;
  uint32_t nWifi = 3;
  bool tracing = false;

  CommandLine cmd;
  cmd.AddValue ("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing); 

  cmd.Parse (argc,argv);
     */

    // Server Node
    Node sn = Node();
    Ptr<Node> server = Ptr<Node>(&sn);
    
    // AP Nodes
    NodeContainer apNodes;
    apNodes.Create(20);
    
    // UAV Node
    Node rn = Node();
    Ptr<Node> robot = Ptr<Node>(&rn);
    
    // helper containers to install nodes more easily
    NodeContainer ethernetNodes;
    ethernetNodes.Add(server);
    ethernetNodes.Add(apNodes);
    
    NodeContainer wifiNodes;
    wifiNodes.Add(apNodes);
    wifiNodes.Add(robot);

    
    // Ethernet connection from APs to server
    CsmaHelper ethernet;
    // TODO: ethernet attributes
    ethernet.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    ethernet.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

    NetDeviceContainer ethernetDevices;
    ethernetDevices = ethernet.Install(ethernetNodes);

    // Wifi connection from robot to APs
    // TODO: wifi attributes
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");
    mac.SetType("ns3::StaWifiMac",
            "Ssid", SsidValue(ssid),
            "ActiveProbing", BooleanValue(false));

    NetDeviceContainer wifiDevices;
    wifiDevices = wifi.Install(phy, mac, wifiNodes);

    mac.SetType("ns3::ApWifiMac",
            "Ssid", SsidValue(ssid));

    
    // AP Mobility
    MobilityHelper apMobility;
    // TODO: final physical layout of the nodes
    apMobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(10.0),
            "MinY", DoubleValue(10.0),
            "DeltaX", DoubleValue(20.0),
            "DeltaY", DoubleValue(20.0),
            "GridWidth", UintegerValue(5),
            "LayoutType", StringValue("RowFirst"));

    apMobility.Install(apNodes);

    apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMobility.Install(apNodes);
    
    // server Mobility
    MobilityHelper serverMobility;
    serverMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    serverMobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(300.0),
            "MinY", DoubleValue(75.0));
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
    // TODO:
    /*
    UdpEchoServerHelper echoServer(9);

    ApplicationContainer serverApps = echoServer.Install(csmaNodes.Get(nCsma));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    UdpEchoClientHelper echoClient(csmaInterfaces.GetAddress(nCsma), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps =
            echoClient.Install(wifiStaNodes.Get(nWifi - 1));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(10.0));
    */
    
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    Simulator::Stop(Seconds(10.0));

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
