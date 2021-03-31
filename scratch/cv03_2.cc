#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/wifi-module.h"
#include "ns3/flow-monitor-module.h"


using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("cvicenie 3.2");


int main (int argc, char *argv[]) {
  uint32_t mCsma = 3;
  uint32_t nWifi = 3;
  uint32_t dataSize = 1024;

  CommandLine cmd;
  cmd.AddValue ("m", "Pocet serverov", mCsma);
  cmd.AddValue ("n", "Pocet wiif stanic.", nWifi);
  cmd.AddValue ("dSize", "Velkost udajov.", dataSize);
  cmd.Parse (argc,argv);

   /*********************** Mobility model - pohyb uzlov ***********************/

  NodeContainer wifiNodes, p2pNodes,nA,nB,serverNodes;
  p2pNodes.Create(2);       //AP+R1


  auto router1 = p2pNodes.Get(0);      //Names::Add ("R1", router1); // router 1
  auto accessPoint = p2pNodes.Get(1);  //Names::Add ("AP", accessPoint); // access point

  nA.Add(router1);        //router 1
  nA.Create(2);           //router 2 + PCa

  auto router2 = nA.Get(1); //Names::Add ("R2", router2);
  auto PCa = nA.Get(2);     //Names::Add ("PCa", PCa);

  nB.Add(router1);              // router 1
  nB.Add(router2);              // router 2
  nB.Create(1);                 // PCb
  auto PCb = nB.Get(2);     //Names::Add ("PCb", nB.Get(2));

  serverNodes.Add(router2);              // router 2
  serverNodes.Create(mCsma);             // router 2 + servery
  Names::Add ("PCs", serverNodes.Get(mCsma));


  wifiNodes.Create (nWifi);

  /******************************* Point to point *********************************/

  PointToPointHelper p2pHelper;
  p2pHelper.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  p2pHelper.SetChannelAttribute ("Delay", StringValue ("5ms"));
  NetDeviceContainer p2pDevices;
  p2pDevices = p2pHelper.Install (p2pNodes);

  /************************************* CSMA *************************************/

  CsmaHelper csmaHelper;
  NetDeviceContainer pcaDevices, pcbDevices, serverDevices;

  // PCa
  csmaHelper.SetChannelAttribute ("DataRate", StringValue ("1000Mbps"));
  csmaHelper.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (5)));
  pcaDevices = csmaHelper.Install (nA);

  //  PCb
  csmaHelper.SetChannelAttribute ("DataRate", StringValue ("1000Mbps"));
  csmaHelper.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (300)));
  pcbDevices = csmaHelper.Install (nB);

  // PCs
  csmaHelper.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csmaHelper.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (700)));
  serverDevices = csmaHelper.Install (serverNodes);

  /************************************ WI-FI *************************************/

  // wi-fi channel
  YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();
  // fyzicka vrstva
  YansWifiPhyHelper phy; phy.SetChannel (wChannel.Create ());


  WifiHelper wifiHelper; wifiHelper.SetRemoteStationManager ("ns3::AarfWifiManager");

  Ssid ssid = Ssid ("eduroam");

  // mac
  WifiMacHelper mac;
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));
  NetDeviceContainer wifiMobileDevices;
  wifiMobileDevices = wifiHelper.Install (phy, mac, wifiNodes);
  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer nicWap;
  nicWap = wifiHelper.Install (phy, mac, accessPoint);


  /*********************** Mobility model - pohyb uzlov ***********************/

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5.0),
                                 "DeltaY", DoubleValue (10.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));

  // mobilne zariadenia sa random hybu
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
  mobility.Install (wifiNodes);

  // ostatne su staticke
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (accessPoint);
  mobility.Install (serverNodes);
  mobility.Install (PCa);
  mobility.Install (PCb);
  mobility.Install (router1);

  /***************************** L3 - sietova vrstva, IP *********************************/

  // stack
  InternetStackHelper stack; stack.InstallAll();

  Ipv4AddressHelper address;

  // servery
  address.SetBase ("10.2.1.0", "255.255.255.0");
  Ipv4InterfaceContainer networkSContainer = address.Assign (serverDevices);

  // PCa
  address.SetBase ("10.2.2.0", Ipv4Mask ("/24"));
  Ipv4InterfaceContainer networkAContainer = address.Assign (pcaDevices);
  networkAContainer.SetMetric(1,3);//interface,metric
  networkAContainer.SetMetric(2,4);
  networkAContainer.SetMetric(0,5);

  // PCb
  address.SetBase ("10.2.3.0", Ipv4Mask (0xffffff00));
  Ipv4InterfaceContainer networkBContainer = address.Assign (pcbDevices);
  networkBContainer.SetMetric(0,1);//interface,metric
  networkBContainer.SetMetric(1,2);
  networkBContainer.SetMetric(2,3);

  // point to point
  address.SetBase ("10.2.4.0", Ipv4Mask ("/30"));
  address.Assign (p2pDevices);

  // wi-fi
  address.SetBase ("10.1.0.0", Ipv4Mask("255.255.0.0"));
  Ipv4InterfaceContainer networkWContainer = address.Assign (wifiMobileDevices);
  address.Assign (wifiMobileDevices);
  address.Assign (nicWap);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  /***************************** L7 - aplikacna vrstva *********************************/

  /***** server *****/

  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install ( "PCs") ;
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (11.0));

  /***** klient *****/

  UdpEchoClientHelper echoClient (networkSContainer.GetAddress (mCsma), 9);//mCsma=nS.GetN()-1
  echoClient.SetAttribute ("MaxPackets", UintegerValue (5));
  echoClient.SetAttribute ("Interval", TimeValue (MilliSeconds (1000)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (dataSize));

  ApplicationContainer clientApps = echoClient.Install (wifiNodes.Get(0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (11.0));

  Simulator::Stop (Seconds (11.0));

  /******************************* animacia ***********************************/

  AnimationInterface anim ("animations/cv03_2.xml");

  // access point
  auto antennaImg = anim.AddResource("/home/student/Documents/share/antenna.png");
  anim.UpdateNodeDescription (accessPoint, "AP");
  anim.UpdateNodeImage(accessPoint->GetId(),antennaImg);
  anim.UpdateNodeSize(accessPoint->GetId(),2,2);

  // mobilne zariadenia
  auto runnerImg = anim.AddResource("/home/student/Documents/share/runner.png");
  for (uint32_t i = 0; i < wifiNodes.GetN(); ++i){
    anim.UpdateNodeDescription (wifiNodes.Get(i), "Run"+std::to_string(i));
    anim.UpdateNodeImage(wifiNodes.Get(i)->GetId(),runnerImg);
    anim.UpdateNodeSize(wifiNodes.Get(i)->GetId(),2,2);
  }

  // servery
  auto serverImg = anim.AddResource("/home/student/Documents/share/server.png");
  for (uint32_t i = 1; i < serverNodes.GetN(); ++i){
    anim.UpdateNodeDescription (serverNodes.Get(i), "Ser"+std::to_string(i));
    anim.UpdateNodeImage(serverNodes.Get(i)->GetId(),serverImg);
    anim.UpdateNodeSize(serverNodes.Get(i)->GetId(),2,2);
  }

  // routre
  auto routerImg = anim.AddResource("/home/student/Documents/share/router.png");
  for (uint32_t i = 0; i < 2; ++i) {
      auto router = nA.Get(i);
      anim.UpdateNodeDescription (router, "R"+std::to_string(i+1));
      anim.UpdateNodeImage(router->GetId(),routerImg);
      anim.UpdateNodeSize(router->GetId(),2,2);
  }

  anim.UpdateNodeColor(PCa,0,200,0); // zeleny
  anim.UpdateNodeDescription (PCa, "PCa");

  anim.UpdateNodeColor(PCb,0,0,200); // modry
  anim.UpdateNodeDescription (PCb, "PCb");

  // flow monitor
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  Simulator::Run ();

  // stratene pakety
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    std::cout << "Flow " << i->first - 2 << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
    std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
    std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
    std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
    std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
    std::cout << "  DelaySum: " << i->second.delaySum.GetMilliSeconds()  << " ms\n";
    }
  Simulator::Destroy ();
  return 0;
}
