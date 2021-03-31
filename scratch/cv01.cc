// ctrl + shift + o
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"

// Default Network Topology
//
//       10.1.1.0
// n0 -------------- n1
//    point-to-point
//
 
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("FirstScriptExample");


void f(){
  NS_LOG_UNCOND("dsfsdf\n");
}


int main (int argc, char *argv[]) {
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  // enable logging - info
  Time::SetResolution (Time::NS);
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO); // ctrl + space
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  // vytvorenie 2 nodov
  NodeContainer nodes;
  nodes.Create (2);

  // vyber technologie = Point to Point channel
  PointToPointHelper pointToPoint;
  // properties channelu
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  // install point to point channel na devices
  NetDeviceContainer devices;
  devices = pointToPoint.Install (nodes);

  // internet protocols, rules
  InternetStackHelper stack;
  stack.Install (nodes);

  // priradenie base IP adresy pre devices
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0"); // parametre: IP, subnet mask
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  // vytvori sa server, ktory bezi na porte 9
  UdpEchoServerHelper echoServer (9);
  // node s id=1 je server
  ApplicationContainer serverApps = echoServer.Install (nodes.Get (1));
  // kedy sa spusti a stopne
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));


  // vytvori sa klient na port 9
  UdpEchoClientHelper echoClient (interfaces.GetAddress (1), 9);
  // atributy klienta
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));
  // node s id=0 je klient
  ApplicationContainer clientApps = echoClient.Install (nodes.Get (0));
  // kedy sa spusti a stopne
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));


  // po 4. sekunde sa spusti funkcia f
  Simulator::Schedule(Seconds(4.0), f);

  // ulozenie konfiguracie do xml
//  Config::SetDefault("ns3::ConfigStore::Filename", StringValue("out.xml"));
//  Config::SetDefault("ns3::ConfigStore::FileFormat", StringValue("RawText"));
//  Config::SetDefault("ns3::ConfigStore::Mode", StringValue("Save"));
//  ConfigStore cs;
//  cs.ConfigureAttributes();
//  cs.ConfigureDefaults();

  // custom konfig cez okno
//   GtkConfigStore csgtk;
//   csgtk.ConfigureAttributes();
//   csgtk.ConfigureDefaults();

  // spustenie simulacie
  Simulator::Run ();
  // skoncenie simulacie
  Simulator::Destroy ();

  return 0;
}
