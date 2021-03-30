#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/config-store-module.h"

// Default Network Topology
//
//   Wifi 10.1.3.0
//                 AP-R1 (eduroam)
//  *    *    *    *
//  |    |    |    |    10.1.1.0    R2
// n5   n6   n7   n0 -------------- n1   n2   n3   n4
//                   point-to-point  |    |    |    |
//                                   ================
//                                     LAN 10.1.2.0

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("cvicenie2o13");

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktCount, Time pktInterval ){
  if (pktCount > 0) {
      std::ostringstream sprava;
      sprava << "Ahoj stano "<< pktCount << '\0';
      Ptr<Packet> p = Create<Packet> ( (uint8_t*)sprava.str().c_str(), (uint16_t) sprava.str().length() + 1 );
      socket->Send (p);

      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktCount-1, pktInterval);
  }else{ socket->Close (); }
}

int main (int argc, char *argv[]) {

  uint32_t nCsma = 3;
  uint32_t nWifi = 3;

  CommandLine cmd (__FILE__);
  cmd.Parse (argc,argv);

  // logging
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);


  /************************************ POINT TO POINT - smerovace n0, n1 ********************************/

  NodeContainer p2pNodes; p2pNodes.Create (2); //2 smerovace

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer pDevices; pDevices = pointToPoint.Install (p2pNodes);

  /************************************ CSMA - LAN n1, n2, n3, n4 ****************************************/

  NodeContainer csmaNodes;
  csmaNodes.Add (p2pNodes.Get (1));
  csmaNodes.Create (nCsma);

  CsmaHelper csmaHelper;
  csmaHelper.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csmaHelper.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

  // CSMA zariadenie
  NetDeviceContainer csmaDevices; csmaDevices = csmaHelper.Install (csmaNodes);

  /****************** WI-FI - mobilne zariadenia: n5, n6, n7; AP:n0 **************************************/

  NodeContainer wifiNodes; wifiNodes.Create (nWifi);
  NodeContainer accessPointNode = p2pNodes.Get (0);

  // channel a fyzicka vrstva
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy; phy.SetChannel (channel.Create ());

  WifiHelper wifiHelper; wifiHelper.SetRemoteStationManager ("ns3::AarfWifiManager");

  // mac adresa
  Ssid ssid = Ssid ("eduroam");
  WifiMacHelper mac;
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));

  // mobilne wifi zariadenia
  NetDeviceContainer wifiMobileDevices;
  wifiMobileDevices = wifiHelper.Install (phy, mac, wifiNodes);
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));

  // access point zariadenia
  NetDeviceContainer accessPointDevices;
  accessPointDevices = wifiHelper.Install (phy, mac, accessPointNode);

  /************************************ Mobility model - pohyb uzlov ****************************************/

  MobilityHelper mobilityHelper;
  mobilityHelper.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
                      "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=40.0] "),
                      "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=40.0] "),
                      "Z", StringValue("ns3::ConstantRandomVariable[Constant=1.0] "));

  // wifi uzly = random pohyb
  mobilityHelper.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
  mobilityHelper.Install (wifiNodes);

  // access point a csma uzly = staticke
  mobilityHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityHelper.Install (accessPointNode);
  mobilityHelper.Install (csmaNodes);

  /************************************ IP adresy - sietova vrstva ****************************************/

  InternetStackHelper stack; stack.InstallAll();
  Ipv4AddressHelper addressHelper;

  // point to point
  addressHelper.SetBase ("10.1.1.0", Ipv4Mask("/30"));
  addressHelper.Assign (pDevices);
  // csma
  addressHelper.SetBase ("10.1.2.0", "255.255.255.0");
  addressHelper.Assign (csmaDevices);
  // wifi
  addressHelper.SetBase ("10.1.3.0", "255.255.255.0");
  addressHelper.Assign (wifiMobileDevices);
  addressHelper.Assign (accessPointDevices);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /************************************ Klient, server, socket - L4-L7 ****************************************/


  Ptr<Node> sourceNode = wifiNodes.Get(0); Names::Add ("src", sourceNode); // n5
  Ptr<Node> destinationNode = csmaNodes.Get(1); Names::Add ("dst", destinationNode); // n2

  auto csmaIp_1 = csmaNodes.Get(1)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(); // n2
  auto csmaIp_2 = csmaNodes.Get(2)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(); // n3
  NS_LOG_UNCOND("N2: "+csmaIp_1);
  NS_LOG_UNCOND("N3: "+csmaIp_2);

  /****** server = n2 ******/

  UdpEchoServerHelper echoServer (9);
  ApplicationContainer serverApps = echoServer.Install (destinationNode);
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (20.0));


  /****** klient = n3 ******/

  UdpEchoClientHelper echoClient (csmaIp_1 , 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (sourceNode);
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (20.0));

  /****** socket = n7 ******/

  Ptr<Socket> socket = Socket::CreateSocket (wifiNodes.Get(2), TypeId::LookupByName ("ns3::UdpSocketFactory"));
  socket->Connect (InetSocketAddress (csmaIp_2, 80)); // IP n3

  PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (csmaIp_2, 80)); // IP n3
  ApplicationContainer sinkApp = sinkHelper.Install (csmaNodes.Get(2)); // n4
  sinkApp.Start(Seconds(2.0));
  sinkApp.Stop(Seconds(10.0));

  Simulator::Stop (Seconds (10.0));
  Simulator::Schedule (Seconds(4.0), &GenerateTraffic, socket, 10, Seconds(0.3)); // po 4 sekundach sa vygeneruju pakety

  // pcap
  pointToPoint.EnablePcapAll ("pcap/cv02-p2p");
  phy.EnablePcap ("pcap/cv02-phy", accessPointDevices.Get (0));
  csmaHelper.EnablePcap ("pcap/cv02-csma", csmaDevices.Get (0), true);

  // animacia
  AnimationInterface a("cv02.xml");
  a.EnablePacketMetadata();
  a.UpdateNodeColor(destinationNode, 0, 0, 255);
  a.UpdateNodeDescription(destinationNode, "DST");
  a.UpdateNodeColor(sourceNode, 0, 255, 0);
  a.UpdateNodeDescription(sourceNode, "SRC");
//  auto obr = a.AddResource("/home/student/Documents/share/router.png");
//  a.UpdateNodeImage(p2pNodes.Get(0)->GetId(), obr);

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
