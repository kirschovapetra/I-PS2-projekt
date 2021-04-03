#include <bits/stdint-uintn.h>
#include <ns3/application-container.h>
#include <ns3/callback.h>
#include <ns3/command-line.h>
#include <ns3/config.h>
#include <ns3/flow-classifier.h>
#include <ns3/flow-monitor.h>
#include <ns3/flow-monitor-helper.h>
#include <ns3/gnuplot.h>
#include <ns3/inet-socket-address.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-address.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-flow-classifier.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/log.h>
#include <ns3/log-macros-disabled.h>
#include <ns3/mobility-helper.h>
#include <ns3/mobility-model.h>
#include <ns3/net-device-container.h>
#include <ns3/node.h>
#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/on-off-helper.h>
#include <ns3/packet.h>
#include <ns3/position-allocator.h>
#include <ns3/ptr.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/udp-echo-helper.h>
#include <ns3/uinteger.h>
#include <ns3/vector.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/wifi-phy.h>
#include <ns3/wifi-standards.h>
#include <ns3/wifi-tx-vector.h>
#include <ns3/yans-wifi-helper.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

//
//  *      * ---> * <--- *
//  |      |      |      |
// n3     n0     n1     n2
//

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("cvicenie 4.1");


/****************************************** Callback funkcie ********************************************/


void CourseChanged(Ptr<const MobilityModel> model) {
  Vector position = model->GetPosition ();
  NS_LOG_UNCOND (" x = " << position.x << ", y = " << position.y);
}

void CourseChanged_ConfigConnect(int i, std::string context, Ptr<const MobilityModel> model) {
  NS_LOG_UNCOND("i=" << i <<
                " " << Simulator::Now().GetMilliSeconds() <<
                " context=" << context <<
                " x=" << model->GetPosition().x <<
                " y=" << model->GetPosition().y <<
                " z=" << model->GetPosition().z);
}



// Traceovanie WifiPhy
//  MonitorSnifferRx: Trace source simulating a wifi device in monitor mode sniffing all received frames
//  https://www.nsnam.org/doxygen/classns3_1_1_wifi_phy.html#a3fada182d61244bd6124b8d8c9249134
void MonitorSniffRx(/*premenne*/ /*kontext*/ std::string context,
        Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu,
        SignalNoiseDbm signalNoise, uint16_t staId){

  auto ref_a = Config::LookupMatches("/NodeList/0/");
  auto a = ref_a.Get(0)->GetObject<MobilityModel>();

  auto ref_b = Config::LookupMatches("/NodeList/1/");
  auto b = ref_b.Get(0)->GetObject<MobilityModel>();

  NS_LOG_UNCOND(Simulator::Now().GetMilliSeconds() << "ms | " <<
                "Freq: " << channelFreqMhz << " MHz | " <<
                "Noise: "  << signalNoise.noise << " dBm | " <<
                "Signal: " << signalNoise.signal << "dBm | " <<
                "Vzdialenost a-b: " << a->GetDistanceFrom(b));
}

double experiment (bool enableCtsRts, std::string wifiManager, std::string animationFile, double width) {

  // 0. Enable or disable CTS/RTS
  UintegerValue ctsThr = (enableCtsRts ? UintegerValue (100) : UintegerValue (2200));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);

  // 4 uzly
  NodeContainer nodes;
  nodes.Create (4);

  // suradnice uzlov
  Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();
  for (uint8_t i = 0; i < nodes.GetN()-1; ++i) {
      positions->Add(Vector(0.0, width*i,0.0));
  }

  positions->Add(Vector(50.,0.,0.)); // pozicia posledneho nodu

  MobilityHelper mobilityHelper;
  mobilityHelper.SetPositionAllocator(positions);

  // constant
  mobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityHelper.Install(nodes.Get(0));
  mobilityHelper.Install(nodes.Get(1));
  mobilityHelper.Install(nodes.Get(2));

  // random walk
  mobilityHelper.SetMobilityModel("ns3::RandomWalk2dMobilityModel");
  mobilityHelper.Install(nodes.Get(3));


  /******************************* nastavenia wi-fi *********************************/

  // channel
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel"); // model, ktory vytvara stratu, neda sa traceovat https://www.nsnam.org/doxygen/classns3_1_1_friis_propagation_loss_model.html#details"

  // fyzicka vrstva
  YansWifiPhyHelper wifiPhy; // da sa traceovat, premenne: https://www.nsnam.org/doxygen/_trace_source_list.html
  wifiPhy.SetChannel (wifiChannel.Create());

  // mac
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac"); // ad-hoc MAC

  // wifi nastavenia + zariadenia
  WifiHelper wifiHelper;
  wifiHelper.SetStandard (WIFI_STANDARD_80211b);
  wifiHelper.SetRemoteStationManager ("ns3::" + wifiManager + "WifiManager");
  NetDeviceContainer devices = wifiHelper.Install (wifiPhy, wifiMac, nodes);


  /**********************************callback ************************************/


  auto node3mobilityModel = nodes.Get(3)->GetObject<MobilityModel>();

  // without context
//  node3mobilityModel->TraceConnectWithoutContext("CourseChange", MakeCallback(&CourseChanged));

  //config connect (* = vsetky nodes)
//  Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeBoundCallback(&CourseChanged_ConfigConnect,7));

  // MonitorSnifferRxCallback config path = https://www.nsnam.org/doxygen/classns3_1_1_wifi_phy.html#details
  Config::Connect("/NodeList/0|[2-3]/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));


  /******************************* IP adresy *********************************/

  // stack
  InternetStackHelper internet;  internet.Install (nodes);

  // priradenie IP
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.0.0.0");
  ipv4.Assign (devices);

  /******************************* L7 - aplikacna vrstva *********************************/

  OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address ("10.0.0.2"), 12345));
  onOffHelper.SetAttribute ("PacketSize", UintegerValue (1400));
  onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));


  /******* CBR ********/

  ApplicationContainer cbrApps;
  // flow 1:  node 0 -> node 1
  onOffHelper.SetAttribute ("DataRate", StringValue ("3000000bps"));
  onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.000000)));
  cbrApps.Add (onOffHelper.Install (nodes.Get (0)));
  // flow 2:  node 2 -> node 1
  onOffHelper.SetAttribute ("DataRate", StringValue ("3001100bps"));
  onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.001)));
  cbrApps.Add (onOffHelper.Install (nodes.Get (2)));


  /******* Klient ********/

  UdpEchoClientHelper echoClientHelper (Ipv4Address ("10.0.0.2"), 9);
  echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
  echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));

  /******* Ping ********/

  ApplicationContainer pingApps;
  // flow 1:  node 0 -> node 1
  echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001)));
  pingApps.Add (echoClientHelper.Install (nodes.Get (0)));
  // flow 2:  node 2 -> node 1
  echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.006)));
  pingApps.Add (echoClientHelper.Install (nodes.Get (2)));

  // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

//  // animacia cez netanim
//  if (animationFile != "") {
//    AnimationInterface anim(animationFile);
//    anim.EnablePacketMetadata();
//  }



  // Run simulation for 10 seconds
  Simulator::Stop (Seconds (10));
  Simulator::Run ();

  // vypis z flow monitoru
  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();


  double throughtput_sum = 0;

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i) {

      if (i->first > 2) {
//          Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
//          std::cout << "Flow " << i->first - 2 << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
//          std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
//          std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
//          std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000  << " Mbps\n";
//          std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
//          std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
//          std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000  << " Mbps\n";
//          std::cout << "  Celkove oneskorenie: " << std::to_string((i->second.delaySum).GetSeconds()) << " sec\n";

          throughtput_sum += i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000;
      }
    }

  Simulator::Destroy ();

  return throughtput_sum/2;  // vrati priemerny throughtput
}


void visualize(std::string wifiManager, Gnuplot2dDataset data) {
  Gnuplot graf("graphs/cv03_1.svg");
  graf.SetTerminal("svg");
  graf.SetTitle("priemerna priepustnost A2B");
  graf.SetLegend("Vzdialenost anten (m)","Priepustnost (Mbit/s)");
  std::ostringstream str;
  str << "WifiManager " << wifiManager << " ";
  data.SetTitle (str.str() );
  data.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  graf.AddDataset (data);
  std::ofstream plotFile ("graf.plt");
  graf.GenerateOutput (plotFile);
  plotFile.close ();
  if(system("gnuplot graf.plt"));
}


int main (int argc, char **argv) {
  std::string wifiManager ("Arf");
  CommandLine cmd (__FILE__);
  cmd.AddValue ("wifiManager", "Set wifi rate manager (Aarf, Aarfcd, Amrr, Arf, Cara, Ideal, Minstrel, Onoe, Rraa)", wifiManager);
  cmd.Parse (argc, argv);

  std::cout << "Hidden station experiment with RTS/CTS disabled:\n" << std::flush;
  double meranie1 = experiment (false, wifiManager, "animations/cv03_disabled.xml", 100);
//  std::cout << "------------------------------------------------\n";
//  std::cout << "Hidden station experiment with RTS/CTS enabled:\n";
//
//  double meranie2 = experiment (true, wifiManager, "animations/cv03_enabled.xml", 100);
//
//  std::cout << "mer1: " << meranie1 << "\nmer2: " << meranie2 << "\n";

  // viackrat spustene
//  Gnuplot2dDataset data;
//  for (double x = 10; x <= 150; x +=20){
//      std::cout << "Experiment "+ std::to_string(x) << "\n";
//      data.Add(x, experiment(true, wifiManager, "", x));
//  }

//  visualize(wifiManager, data);

  return 0;
}
