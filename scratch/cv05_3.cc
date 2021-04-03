#include <iostream>
#include <cmath>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/yans-wifi-helper.h"

#include "ns3/gnuplot.h"
#include "ns3/stats-module.h"
#include "ns3/animation-interface.h"

using namespace ns3;
 NS_LOG_COMPONENT_DEFINE ("aodv");

class AodvExample {
public:
  AodvExample ();
  bool Configure (int argc, char **argv);
  void Run ();
  void Report (std::ostream & os);
  Ptr<Node> getMobilityModel(uint32_t );
  void addData(double );

private:

  // parameters
  /// Number of nodes
  uint32_t size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;

  uint32_t sumRx;
  uint32_t sumTx;
  uint32_t tmp;
  Gnuplot pG;
  Gnuplot2dDataset data,data2,data3;
  double  oldVel, oldTim;

  // network
  /// nodes used in the example
  NodeContainer nodes;
  /// devices used in the example
  NetDeviceContainer devices;
  /// interfaces used in the example
  Ipv4InterfaceContainer interfaces;

private:
  /// Create the nodes
  void CreateNodes ();
  /// Create the devices
  void CreateDevices ();
  /// Create the network
  void InstallInternetStack ();
  /// Create the simulation applications
  void InstallApplications ();
  //callBacks
  void vyslanieBalika(Ptr<const Packet> );
  void TxStart(Ptr<const Packet>,double );
  void RxEnd(Ptr<const Packet> );
  void MojaFunkcia (Time);
};


int main (int argc, char **argv)
{
  AodvExample test;
  //NS_LOG_ERROR ("sqlite support not compiled in.");
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  test.Report (std::cout);
  return 0;
}

//-----------------------------------------------------------------------------
AodvExample::AodvExample () :
  size (10)
  ,step (50)
  ,totalTime (100)
  ,pcap (false)
  ,printRoutes (false)
  ,sumRx(0)
  ,sumTx(0)
  ,tmp(2799)//2.8s =2800ms
  ,pG("graphs/cv05_3.svg")

{
    pG.SetTerminal("svg enhanced size 800 800 background rgb 'white' fname 'Verdana,20'");
    pG.SetTitle("AODV example");
    pG.SetLegend("Cas simulacie (s)","pomer, pingTime(s)");
    pG.SetExtra("set y2label \"Akceleracia(m/s^2)\"");
    pG.AppendExtra("set ytics nomirror");
	pG.AppendExtra("set y2tics nomirror");
    data.SetTitle ("pomer");
    data.SetStyle (Gnuplot2dDataset::LINES_POINTS);
    data2.SetTitle ("ping");
    data2.SetStyle (Gnuplot2dDataset::LINES_POINTS);
    data3.SetTitle ("acceleration\" axes x1y2 with steps #");
    data3.SetStyle (Gnuplot2dDataset::STEPS);
}

bool AodvExample::Configure (int argc, char **argv){

  SeedManager::SetSeed (12345);
  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);
  cmd.AddValue ("tmp", "temp val", tmp);

  cmd.Parse (argc, argv);
  return true;
}

void AodvExample::Run (){
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";
  AnimationInterface a("animations/cv05_3.xml");
  a.EnablePacketMetadata();

  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();

  data.Add(Simulator::Now().GetSeconds(), (double)sumRx / (double)sumTx );
  pG.AddDataset (data);
  pG.AddDataset (data2);
  pG.AddDataset (data3);
  std::ofstream plotFile ("graf.plt");
  pG.GenerateOutput (plotFile);
  plotFile.close ();
  if(system("gnuplot graf.plt"));
  Simulator::Destroy ();
}

void AodvExample::Report (std::ostream &){
}

void AodvExample::CreateNodes (){
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i){
      std::ostringstream os;
      os << "node-" << i;
      Names::Add (os.str (), nodes.Get (i));
    }
  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (step),
                                 "DeltaY", DoubleValue (0),
                                 "GridWidth", UintegerValue (size),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel"
            ,"Mode", StringValue("Time")//1m|1s
			,"Time", TimeValue(Seconds(totalTime/4))
			,"Speed",StringValue( "ns3::ConstantRandomVariable[Constant=0.0]" )
			,"Bounds", RectangleValue(Rectangle(0,step*(size),0,step*(size)))
);
  mobility.Install (nodes);
  oldTim=0.0; oldVel=0.0;
}

void AodvExample::CreateDevices (){
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (1024) );
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("pcap/cv05_3_aodv"));
    }
}

void AodvExample::InstallInternetStack () {
  AodvHelper aodv;
  // you can configure AODV attributes here using aodv.Set(name, value)
  aodv.Set("EnableHello", BooleanValue(true));
  //aodv.Set("RreqRetries", UintegerValue(tmp));
  //aodv.Set("NodeTraversalTime", TimeValue(MilliSeconds(tmp)));
  aodv.Set("NetTraversalTime", TimeValue(MilliSeconds(tmp)));
  InternetStackHelper stack;
  stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (8), routingStream);
    }
}

void AodvExample::MojaFunkcia (Time v){
//    std::cout << v.GetMilliSeconds() << " v case " << Simulator::Now().GetSeconds() << "\n";
    data.Add(Simulator::Now().GetSeconds(), sumTx/ (double )sumRx  );
    data2.Add(Simulator::Now().GetSeconds(), v.GetSeconds());
    sumRx = sumTx = 0;
}

void AodvExample::TxStart(Ptr<const Packet> p,double power){
    sumTx += p->GetSize();
//        std::cout << " Tx"<<power <<" v case " << Simulator::Now().GetSeconds() << " a velk" << sumTx <<"\n";
}

void AodvExample::RxEnd(Ptr<const Packet> p){
    sumRx += p->GetSize();
//    std::cout << " Rx v case " << Simulator::Now().GetSeconds() << " a velk" << sumRx <<"\n";
}

void changeMobility(std::string text){
  boost::char_separator<char> sep(";");
  boost::tokenizer< boost::char_separator<char> > tokens(text, sep);

  BOOST_FOREACH (const std::string& t, tokens) {
  char sw = t.at(0);
  	switch(sw){
      case 'S':
  		Config::Set("NodeList/9/$ns3::MobilityModel/$ns3::RandomWalk2dMobilityModel/Speed",StringValue(t.substr(1)));break;
      case 'D':
  		Config::Set("NodeList/9/$ns3::MobilityModel/$ns3::RandomWalk2dMobilityModel/Direction",StringValue(t.substr(1)));break;
      case 'T':
  		Config::Set("NodeList/9/$ns3::MobilityModel/$ns3::RandomWalk2dMobilityModel/Time",StringValue(t.substr(1)));break;
   	  //default :       std::cout<<   "nieco ine\n";
    }
  }
}

Ptr<Node> AodvExample::getMobilityModel(uint32_t i){return nodes.Get(i);}

void AodvExample::addData(double newVel){
  // derivacia .. lim ... (newVEl -oldVel)/(newTim-oldTim)
	if(oldTim != Simulator::Now().GetSeconds() ){ data3.Add(Simulator::Now().GetSeconds(), (newVel - oldVel)/(Simulator::Now().GetSeconds()-oldTim));
		oldTim = Simulator::Now().GetSeconds();
		oldVel = newVel;}
//        std::cout << Simulator::Now().GetSeconds() << " " << oldVel << " " << newVel << "\n";
}

void pohyb(AodvExample* ae, Ptr< const MobilityModel > model){
//	std::cout <<Simulator::Now().GetSeconds() << "\t" << model->GetVelocity().GetLength() << "\t" << model->GetPosition() << "\n";

	Ptr<MobilityModel> model2 = ae->getMobilityModel(0)->GetObject<MobilityModel>();
	if(model->GetDistanceFrom(model2) < 40.){changeMobility("Sns3::ConstantRandomVariable[Constant=0.0];T2s");}
	ae->addData( model->GetVelocity().GetLength() );
}

void AodvExample::InstallApplications (){
  ns3::PacketMetadata::Enable ();
  V4PingHelper ping (interfaces.GetAddress (size - 1));
  ping.SetAttribute ("Verbose", BooleanValue (false));

  ApplicationContainer p;
  p.Add(ping.Install(nodes.Get (0)));
  p.Start (Seconds (0));
  p.Stop (Seconds (totalTime) - Seconds (0.001));

  // move node away
  Ptr<Node> node = nodes.Get (size/2);
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  Simulator::Schedule (Seconds (totalTime/4), &MobilityModel::SetPosition, mob, Vector (size/2*step, size/2*step, 0));
  Simulator::Schedule (Seconds (totalTime/4), &changeMobility, "Sns3::UniformRandomVariable[Min=10.0|Max=50.0];T500ms;Dns3::ConstantRandomVariable[Constant=-2.5]");
  Config::ConnectWithoutContext("NodeList/9/$ns3::MobilityModel/CourseChange",MakeBoundCallback(&pohyb, this));
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::V4Ping/Rtt",MakeCallback(&AodvExample::MojaFunkcia, this));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",MakeCallback(&AodvExample::TxStart, this));
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxEnd",MakeCallback(&AodvExample::RxEnd, this));
}
