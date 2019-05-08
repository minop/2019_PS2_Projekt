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
#include "ns3/animation-interface.h"

using namespace ns3;

void runSim(double);

int
main(int argc, char *argv[]) {
    //
    // First, we declare and initialize a few local variables that control some
    // simulation parameters.
    //
    uint32_t backboneNodes = 10;
    uint32_t infraNodes = 2;
    uint32_t lanNodes = 2;
    uint32_t stopTime = 20;
    bool useCourseChangeCallback = false;

    //
    // Simulation defaults are typically set next, before command line
    // arguments are parsed.
    //
    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

    //
    // For convenience, we add the local variables to the command line argument
    // system so that they can be overridden with flags such as
    // "--backboneNodes=20"
    //
    CommandLine cmd;
    cmd.AddValue("backboneNodes", "number of backbone nodes", backboneNodes);
    cmd.AddValue("infraNodes", "number of leaf nodes", infraNodes);
    cmd.AddValue("lanNodes", "number of LAN nodes", lanNodes);
    cmd.AddValue("stopTime", "simulation stop time (seconds)", stopTime);
    cmd.AddValue("useCourseChangeCallback", "whether to enable course change tracing", useCourseChangeCallback);

    //
    // The system global variables and the local values added to the argument
    // system can be overridden by command line arguments by using this call.
    //
    cmd.Parse(argc, argv);

    if (stopTime < 10) {
        std::cout << "Use a simulation stop time >= 10 seconds" << std::endl;
        exit(1);
    }
    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Construct the backbone                                                //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    //
    // Create a container to manage the nodes of the adhoc (backbone) network.
    // Later we'll create the rest of the nodes we'll need.
    //
    NodeContainer backbone;
    backbone.Create(backboneNodes);
    //
    // Create the backbone wifi net devices and install them into the nodes in
    // our container
    //
    WifiHelper wifi;
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
            "DataMode", StringValue("OfdmRate54Mbps"));
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    NetDeviceContainer backboneDevices = wifi.Install(wifiPhy, mac, backbone);

    // We enable OLSR (which will be consulted at a higher priority than
    // the global routing) on the backbone ad hoc nodes
    //NS_LOG_INFO("Enabling OLSR routing on all backbone nodes");
    OlsrHelper olsr;
    //
    // Add the IPv4 protocol stack to the nodes in our container
    //
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr); // has effect on the next Install ()
    internet.Install(backbone);

    //
    // Assign IPv4 addresses to the device drivers (actually to the associated
    // IPv4 interfaces) we just created.
    //
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase("192.168.0.0", "255.255.255.0");
    ipAddrs.Assign(backboneDevices);

    //
    // The ad-hoc network nodes need a mobility model so we aggregate one to
    // each of the nodes we just finished building.
    //
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
            "MinX", DoubleValue(20.0),
            "MinY", DoubleValue(20.0),
            "DeltaX", DoubleValue(20.0),
            "DeltaY", DoubleValue(20.0),
            "GridWidth", UintegerValue(5),
            "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
            "Bounds", RectangleValue(Rectangle(-500, 500, -500, 500)),
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=2]"),
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
    mobility.Install(backbone);

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Construct the LANs                                                    //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    // Reset the address base-- all of the CSMA networks will be in
    // the "172.16 address space
    ipAddrs.SetBase("172.16.0.0", "255.255.255.0");


    for (uint32_t i = 0; i < backboneNodes; ++i) {
        //NS_LOG_INFO("Configuring local area network for backbone node " << i);
        //
        // Create a container to manage the nodes of the LAN.  We need
        // two containers here; one with all of the new nodes, and one
        // with all of the nodes including new and existing nodes
        //
        NodeContainer newLanNodes;
        newLanNodes.Create(lanNodes - 1);
        // Now, create the container with all nodes on this link
        NodeContainer lan(backbone.Get(i), newLanNodes);
        //
        // Create the CSMA net devices and install them into the nodes in our
        // collection.
        //
        CsmaHelper csma;
        csma.SetChannelAttribute("DataRate",
                DataRateValue(DataRate(5000000)));
        csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));
        NetDeviceContainer lanDevices = csma.Install(lan);
        //
        // Add the IPv4 protocol stack to the new LAN nodes
        //
        internet.Install(newLanNodes);
        //
        // Assign IPv4 addresses to the device drivers (actually to the
        // associated IPv4 interfaces) we just created.
        //
        ipAddrs.Assign(lanDevices);
        //
        // Assign a new network prefix for the next LAN, according to the
        // network mask initialized above
        //
        ipAddrs.NewNetwork();
        //
        // The new LAN nodes need a mobility model so we aggregate one
        // to each of the nodes we just finished building.
        //
        MobilityHelper mobilityLan;
        Ptr<ListPositionAllocator> subnetAlloc =
                CreateObject<ListPositionAllocator> ();
        for (uint32_t j = 0; j < newLanNodes.GetN(); ++j) {
            subnetAlloc->Add(Vector(0.0, j * 10 + 10, 0.0));
        }
        mobilityLan.PushReferenceMobilityModel(backbone.Get(i));
        mobilityLan.SetPositionAllocator(subnetAlloc);
        mobilityLan.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobilityLan.Install(newLanNodes);
    }

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Construct the mobile networks                                         //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    // Reset the address base-- all of the 802.11 networks will be in
    // the "10.0" address space
    ipAddrs.SetBase("10.0.0.0", "255.255.255.0");

    for (uint32_t i = 0; i < backboneNodes; ++i) {
        //NS_LOG_INFO("Configuring wireless network for backbone node " << i);
        //
        // Create a container to manage the nodes of the LAN.  We need
        // two containers here; one with all of the new nodes, and one
        // with all of the nodes including new and existing nodes
        //
        NodeContainer stas;
        stas.Create(infraNodes - 1);
        // Now, create the container with all nodes on this link
        NodeContainer infra(backbone.Get(i), stas);
        //
        // Create an infrastructure network
        //
        WifiHelper wifiInfra;
        WifiMacHelper macInfra;
        wifiPhy.SetChannel(wifiChannel.Create());
        // Create unique ssids for these networks
        std::string ssidString("wifi-infra");
        std::stringstream ss;
        ss << i;
        ssidString += ss.str();
        Ssid ssid = Ssid(ssidString);
        wifiInfra.SetRemoteStationManager("ns3::ArfWifiManager");
        // setup stas
        macInfra.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid));
        NetDeviceContainer staDevices = wifiInfra.Install(wifiPhy, macInfra, stas);
        // setup ap.
        macInfra.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid),
                "BeaconInterval", TimeValue(Seconds(2.5)));
        NetDeviceContainer apDevices = wifiInfra.Install(wifiPhy, macInfra, backbone.Get(i));
        // Collect all of these new devices
        NetDeviceContainer infraDevices(apDevices, staDevices);

        // Add the IPv4 protocol stack to the nodes in our container
        //
        internet.Install(stas);
        //
        // Assign IPv4 addresses to the device drivers (actually to the associated
        // IPv4 interfaces) we just created.
        //
        ipAddrs.Assign(infraDevices);
        //
        // Assign a new network prefix for each mobile network, according to
        // the network mask initialized above
        //
        ipAddrs.NewNetwork();
        //
        // The new wireless nodes need a mobility model so we aggregate one
        // to each of the nodes we just finished building.
        //
        Ptr<ListPositionAllocator> subnetAlloc =
                CreateObject<ListPositionAllocator> ();
        for (uint32_t j = 0; j < infra.GetN(); ++j) {
            subnetAlloc->Add(Vector(0.0, j, 0.0));
        }
        mobility.PushReferenceMobilityModel(backbone.Get(i));
        mobility.SetPositionAllocator(subnetAlloc);
        mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                "Bounds", RectangleValue(Rectangle(-10, 10, -10, 10)),
                "Speed", StringValue("ns3::ConstantRandomVariable[Constant=3]"),
                "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.4]"));
        mobility.Install(stas);
    }

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Application configuration                                             //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    // Create the OnOff application to send UDP datagrams of size
    // 210 bytes at a rate of 10 Kb/s, between two nodes
    // We'll send data from the first wired LAN node on the first wired LAN
    // to the last wireless STA on the last infrastructure net, thereby
    // causing packets to traverse CSMA to adhoc to infrastructure links

    //NS_LOG_INFO("Create Applications.");
    uint16_t port = 9; // Discard port (RFC 863)

    // Let's make sure that the user does not define too few nodes
    // to make this example work.  We need lanNodes > 1  and infraNodes > 1
    NS_ASSERT(lanNodes > 1 && infraNodes > 1);
    // We want the source to be the first node created outside of the backbone
    // Conveniently, the variable "backboneNodes" holds this node index value
    Ptr<Node> appSource = NodeList::GetNode(backboneNodes);
    // We want the sink to be the last node created in the topology.
    uint32_t lastNodeIndex = backboneNodes + backboneNodes * (lanNodes - 1) + backboneNodes * (infraNodes - 1) - 1;
    Ptr<Node> appSink = NodeList::GetNode(lastNodeIndex);
    // Let's fetch the IP address of the last node, which is on Ipv4Interface 1
    Ipv4Address remoteAddr = appSink->GetObject<Ipv4> ()->GetAddress(1, 0).GetLocal();

    OnOffHelper onoff("ns3::UdpSocketFactory",
            Address(InetSocketAddress(remoteAddr, port)));

    ApplicationContainer apps = onoff.Install(appSource);
    apps.Start(Seconds(3));
    apps.Stop(Seconds(stopTime - 1));

    // Create a packet sink to receive these packets
    PacketSinkHelper sink("ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), port));
    apps = sink.Install(appSink);
    apps.Start(Seconds(3));

    ///////////////////////////////////////////////////////////////////////////
    //                                                                       //
    // Tracing configuration                                                 //
    //                                                                       //
    ///////////////////////////////////////////////////////////////////////////

    //NS_LOG_INFO("Configure Tracing.");
    CsmaHelper csma;

    //
    // Let's set up some ns-2-like ascii traces, using another helper class
    //
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("mixed-wireless.tr");
    wifiPhy.EnableAsciiAll(stream);
    csma.EnableAsciiAll(stream);
    internet.EnableAsciiIpv4All(stream);

    // Csma captures in non-promiscuous mode
    csma.EnablePcapAll("mixed-wireless", false);
    // pcap captures on the backbone wifi devices
    wifiPhy.EnablePcap("mixed-wireless", backboneDevices, false);
    // pcap trace on the application data sink
    wifiPhy.EnablePcap("mixed-wireless", appSink->GetId(), 0);

    if (useCourseChangeCallback == true) {
        //Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback(&CourseChangeCallback));
    }

    AnimationInterface anim("mixed-wireless.xml");

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