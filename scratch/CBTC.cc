/*

5. Simulácia dopravy – CBTC (električka, vlak …)

CBTC systémy sú systémy na zefektívnenie dopravy. Dopravné prostriedky sa pohybujú za sebou (bez predbiehania) približujú/vzďaľujú sa od sebe prípadne sa môžu prekrývať. Dôležité je zadefinovať si trasy a zástavky.

Príklady správ:
rovnaká linka: Podľa poslatia správy (napr. PING) zistia že je nietko v blízkosti a spomalia/zabrzdia. - TOTO budeme robit
rôzna linka: Synchronizácia pri príjazdoch na zástavku/odchod

Efektivita môže byť definovaná ako skrátenie času presunu, alebo minimalizovanie státia, či minimalizovanie zrýchľovania/spomaľovania pohybu.

  * List position allocator: https://coe.northeastern.edu/Research/krclab/crens3-doc/classns3_1_1_list_position_allocator.html
  * Mapa elektriciek: https://dpba.blob.core.windows.net/media/Default/Obr%C3%A1zky/elektricky_201912.png
  * Waypoint mobility model: https://groups.google.com/g/ns-3-users/c/8r07uUD78Dk


HODNOTENIE:

✔ 1b: Vizualizácia Netanim, PyViz. Pomocou parametru povoľte/zakážte tvorby animácie, preto aby nevznikal/nezanikal súbor nanovo a tým nebrzdil simuláciu

2b Dva grafy
      pozn. Graf má obsahovať minimálne 10 bodov meraní s vyhodnotením, t.j. odchýlky merania;
      Simuláciu spustite viac krát, meňte pomocou SeedManager nastavenia generovania náhodnej premennej,
      aby ste získali priemer hodnôt merania a štandardnú odchýlku merania (yErrorbars) pre vynesené body grafu.

2b zhodnotenie grafov

2b Popis projektu (Hodnotí sa nápaditosť. Uprednostnite jednoduchosť a celistvosť)

2b Popis vhodného výberu ISO OSI (V úvodnom komentári popíšte dôvody výberu jednotlivých protokolov)

✔ 1b použitie časových udalostí (záznam o zmene)
    - Simulator::Schedule

✔ 2b použitie udalostí zmenu stavu (záznam o zmene);(atribútu modelu)
    - Course change

✔ 3b použitie vlastnej triedy v projekte (jeho zmena vlastnosti, využitie NS3 funkcionality udalostný systém)
    - bud zdedena/rozsirenie povodnej alebo totalne nova

✔ 2b v čase simulácie zmena v modeli L1 fyzické médium, pohyb, útlm …
    - zmeni sa spravanie uzlu - mobility model

3b v čase simulácie zmena v modelu L2-L5
   - zmenim atribut nejakeho smerovacieho protokolu - kolko paketov posiela atd
   - zmena intervalu Hello Paketov
   - na transportnej/aplikacnej/spojovej vrstve

 */

#include "CBTC.h"

#include <ns3/address.h>
#include <ns3/animation-interface.h>
#include <ns3/aodv-helper.h>
#include <ns3/assert.h>
#include <ns3/boolean.h>
#include <ns3/callback.h>
#include <ns3/command-line.h>
#include <ns3/config.h>
#include <ns3/double.h>
#include <ns3/dsdv-helper.h>
#include <ns3/dsr-helper.h>
#include <ns3/dsr-main-helper.h>
#include <ns3/event-id.h>
#include <ns3/fatal-error.h>
#include <ns3/inet-socket-address.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4.h>
#include <ns3/ipv4-address.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-global-routing-helper.h>
#include <ns3/ipv4-interface-address.h>
#include <ns3/log.h>
#include <ns3/log-macros-disabled.h>
#include <ns3/mobility-helper.h>
#include <ns3/names.h>
#include <ns3/node-list.h>
#include <ns3/object.h>
#include <ns3/olsr-helper.h>
#include <ns3/packet.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/position-allocator.h>
#include <ns3/simulator.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/type-id.h>
#include <ns3/waypoint.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/yans-wifi-helper.h>
#include <stddef.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>
#include <utility>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Tema 5: Simulacia dopravy – CBTC");

/************************** staticke premenne ******************************/

double CBTC::interval = 7.0;
double CBTC::delay = 2.0;
Time CBTC::stopLength = Seconds(2.0);
map<int, Time> CBTC::timings;
map<int, bool > CBTC::isStopped;
map<int, bool > CBTC::resetStop;
map<int, int> CBTC::tramPositions;

//   cesta: A (0) -> B (1) -> C (2) -> D (3) -> H (7) -> I (8) -> J (9) -> G (6) -> F (5) -> E (4) -> A (0)
vector<int> CBTC::path = {0,1,2,5,7,8,9,4,0};
vector<string> CBTC::stopNames = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"};
vector<Vector> CBTC::stopPositions = { Vector(0.0, 0.0, 0.0),        //A
                                       Vector(0.0, 200.0, 0.0),      //B

                                       Vector(100.0, 100.0, 0.0),    //C
                                       Vector(100.0, 200.0, 0.0),    //D

                                       Vector(200.0, 0.0, 0.0),      //E
                                       Vector(200.0, 100.0, 0.0),    //F

                                       Vector(300.0, 100.0, 0.0),    //G
                                       Vector(300.0, 200.0, 0.0),    //H
                                       Vector(400.0, 200.0, 0.0),    //I

                                       Vector(400.0, 100.0, 0.0) };  //J


/**********************************************************************************/


CBTC::CBTC(uint32_t tramsN = 3, uint32_t packetSize = 50, uint32_t totalTime = 1000, bool saveAnim = true,
           uint32_t protocol = 1, uint32_t packetInterval = 6.0, bool runtimeIntervalChange = true) {
  // CMD line argumenty
  this->tramsN = tramsN;
  this->packetSize = packetSize;
  this->totalTime = totalTime;
  this->saveAnim = saveAnim;
  this->protocol = protocol;
  this->packetInterval = packetInterval;
  this->runtimeIntervalChange = runtimeIntervalChange;
}


CBTC::~CBTC() {}

TypeId CBTC::GetTypeId () {
  static TypeId tid = TypeId ("CBTC")
    .SetParent<Object> ()
    .SetGroupName("CBTC")
    .AddConstructor<CBTC>()
    .AddTraceSource("packetIntervalTrace", "Trace-ovana zmena intervalu posielania paketov.",
                    MakeTraceSourceAccessor (&CBTC::packetIntervalTrace),
                    "ns3::TracedValueCallback::Int32");
  return tid;
}


/******************************** Trace callback ************************************/


void PacketIntervalChanged (int32_t oldValue, int32_t newValue) {
  NS_LOG_UNCOND("[INFO] Interval posielania paketov sa zmenil z " << oldValue << " na " << newValue << endl);
}

void RewritePacketInterval(Ptr<CBTC> thisPtr) { thisPtr->packetIntervalTrace = thisPtr->packetInterval; }


/*********************************** Config **************************************/

void CBTC::SetCommandLineArgs(int argc, char **argv) {
  /*
   * Vypis cmd line argumentov: ./waf --run "CBTC --PrintHelp"
   * Spustenie s argumentami:   ./waf --run "CBTC --ulozAnimaciu=true --velkostUdajov=100" atd
  */

  CommandLine cmd;

  cmd.AddValue ("saveAnim", "Ulozenie animacie", saveAnim);
  cmd.AddValue("packetSize", "Velkost paketu.", packetSize);
  cmd.AddValue("totalTime", "Trvanie simulacie.", totalTime);
  cmd.AddValue("tramN", "Pocet elektriciek.", tramsN);
  cmd.AddValue ("protocol", "Routovaci protokol: 1=OLSR;2=AODV;3=DSDV;4=DSR", protocol);
  cmd.AddValue ("packetInterval", "Interval posielania hello paketov", packetInterval);
  cmd.AddValue ("runtimeIntervalChange", "Zmena intervalu posielania paketov pocas behu simulacie true/false", runtimeIntervalChange);
  // dalsie parametre ...
  cmd.Parse(argc, argv);

  // logging

  NS_LOG_UNCOND(boolalpha << endl
                << "[INFO] --saveAnim=" << saveAnim
                << " --packetSize=" << packetSize
                << " --totalTime=" << totalTime
                << " --tramN=" << tramsN
                << " --protocol=" << protocol
                << " --packetInterval=" << packetInterval
                << " --runtimeIntervalChange=" << runtimeIntervalChange << endl);

}


void CBTC::CreateNodes() {

  // zastavky
  stopNodes.Create(stopsN);
  for (int i = 0; i < stopsN; i++)
      Names::Add (stopNames[i], stopNodes.Get(i));

  // elektricky
  tramNodes.Create(tramsN);
  for (int i = 0; i < tramsN; i++)
      Names::Add (to_string(i+1), tramNodes.Get(i));
}


void CBTC::StopsConstantPositionModel(MobilityHelper &mobility) {

  Ptr <ListPositionAllocator> stopPosAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: stopPositions) {
      stopPosAlloc -> Add(pozicia);
  }

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(stopPosAlloc);
  mobility.Install(stopNodes);
}


void CBTC::TramsWaypointModel(MobilityHelper &mobility) {

  mobility.SetMobilityModel("ns3::WaypointMobilityModel", "LazyNotify", BooleanValue(true));
  mobility.Install(tramNodes);

  // nastavenie pozicii do mobility modelu
  Ptr <ListPositionAllocator> tramPosAlloc = CreateObject<ListPositionAllocator>();
  for( int i = 0; i < tramsN; i++) {
      tramPosAlloc -> Add(Vector(0.0, 0.0, 0.0));
  }

  mobility.SetPositionAllocator(tramPosAlloc);


  // casovanie zastavky
  for (int i = 0; i < tramsN; i++) {
      timings.insert({i, Seconds(5) + Seconds(i * delay)});
      isStopped.insert({i, false});
      resetStop.insert({i, true});
  }


  for (int i = 0; i < tramsN; i++) {

//      cout << "-- ELEKTRICKA" << i << " --" << endl;
      tramPositions.insert(pair<int, int>(i, 1));

      Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel> (
          tramNodes.Get (i)->GetObject<MobilityModel> ());

      //0. zastavka
      model->AddWaypoint (Waypoint(Seconds(i * delay), stopPositions[path[0]]));

      //1. zastavka
      model->AddWaypoint (Waypoint(timings[i], stopPositions[path[1]]));

      // Schedulovanie zastavok pre jednotilve elektricky
      Simulator::Schedule (timings[i], &ScheduleNextStop, model, i);
  }

}


void CBTC::SetP2PDevices() {

  // spojenia medzi dvojicami zastavok
  vector<vector<string>> pairs = { {"A","B"}, {"A","E"}, {"B","C"}, {"C","D"}, {"C","F"}, {"D","F"}, {"D","H"}, {"J","E"},
                                   {"F","E"}, {"F","G"}, {"F","H"}, {"G","H"}, {"H","J"}, {"G","J"}, {"H","I"}, {"I","J"} };

  PointToPointHelper p2pHelper;
  for (auto &p: pairs)
    p2pHelper.Install(p[0], p[1]);

}


NetDeviceContainer CBTC::SetWifiDevices() {
     // wi-fi channel - elektricky
     YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();

     Config::SetDefault ("ns3::RangePropagationLossModel::MaxRange", DoubleValue (75));
     wChannel.AddPropagationLoss ("ns3::RangePropagationLossModel");

     YansWifiPhyHelper phy;
     phy.SetChannel (wChannel.Create ());

     WifiHelper wifi;
     wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager");

     // ad-hoc siet
     WifiMacHelper mac;
     mac.SetType ("ns3::AdhocWifiMac");

     return wifi.Install (phy, mac, tramNodes);
}


void CBTC::SetRouting(NetDeviceContainer nicElektricky) {

  InternetStackHelper internet;

  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;

  switch (protocol) {
    case 1:
      internet.SetRoutingHelper(olsr);
      break;
    case 2:
      internet.SetRoutingHelper(aodv);
      break;
    case 3:
      internet.SetRoutingHelper(dsdv);
      break;
    case 4:
      dsrMain.Install (dsr, tramNodes);
      break;
    default:
      NS_FATAL_ERROR ("Taky protokol neexistuje");
  }

  internet.Install(tramNodes);

  // elektricky: subnet 10.1.1.0 maska 255.255.255.0
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  interfaces = ipv4.Assign(nicElektricky);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}


void CBTC::SetApplications() {

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // vytvorenie socketov medzi elektrickami
  for (int i = 0, port = 80; i < tramNodes.GetN () - 1; i++, port--) {

      Ptr<Socket> recvSink = Socket::CreateSocket (tramNodes.Get (i + 1), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
      recvSink->Bind (local);
      recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

      Ptr<Socket> source = Socket::CreateSocket (tramNodes.Get (i), tid);
      InetSocketAddress remote = InetSocketAddress (interfaces.GetAddress(i + 1), port);
//      source->SetAllowBroadcast (true);
      source->Connect (remote);

      sockets.push_back(source);
  }

  // schedulovanie posielania paketov
  for (int i = 0; i < tramsN - 1; i++) {
      Simulator::ScheduleWithContext (sockets[i]->GetNode()->GetId(),
                                      Seconds(5.0 + i*2),
                                      &GenerateTraffic,
                                      sockets[i],
                                      packetSize,
                                      Seconds (packetIntervalTrace));
  }

}


/********************************** Pakety *************************************/


void CBTC::ReceivePacket (Ptr<Socket> socket) {


  Ptr<Node> socketNode = socket -> GetNode();
  Ptr<Packet> packet = socket->Recv();

  while (packet) {

      // elektricka zastavi
      Ptr<WaypointMobilityModel> waypointModel = DynamicCast<WaypointMobilityModel>(socketNode -> GetObject<MobilityModel>());
      StopTramMovement(waypointModel, socketNode -> GetId() - 10);

      // vypis paketu
      int packetSize = packet->GetSize();
      uint8_t *buffer = new uint8_t[packetSize];

      int size = packet->CopyData(buffer, packetSize);
      string packetContent = string(buffer, buffer + packetSize);
      packetContent.resize(packetContent.find('\0')); // odstrani sa padding z paketu

      Ptr<Ipv4> ipv4 = socketNode -> GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);

      NS_LOG_UNCOND("[INFO] " << Simulator::Now() << " | Receiver: " << iaddr.GetLocal() << " | "
                              << "Received ("<< size<< " B): " << packetContent << endl);

    }
}


void CBTC::GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, Time pktInterval ) {

  Ptr<Ipv4> ipv4 = socket -> GetNode() -> GetObject<Ipv4> ();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);

  ostringstream msg; msg << "Hello World from " << iaddr.GetLocal() << '\0';

  uint16_t messageLength = msg.str().length()+1;

  Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), messageLength);
  packet->AddPaddingAtEnd(pktSize - messageLength); // aby bol takej velkosti, ako pktSize
  socket->Send (packet);
  Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, Seconds(2.0));

}


/**************************** Pohyb elektriciek *********************************/


void CBTC::ScheduleNextStop (Ptr<WaypointMobilityModel> model, int id){

  srand (time(NULL));

  if (isStopped[id]){
      isStopped.find (id) -> second = false;
  }

  else {
    int nextStop = tramPositions.at (id) + 1;

    if (nextStop == path.size () - 1){
        tramPositions.find (id)->second = 0;
        nextStop = 0;
    } else {
        tramPositions.find (id)->second = nextStop;
    }

    auto now = Simulator::Now () ;


    // nastavenie nahodnej rychlosti
    Time randTime = Seconds(rand() % 9 + 5);
    timings.find(id) -> second = now + randTime + stopLength;

//    cout << "ELEKTRICKA #" << id << " next stop -> " << nextStop << endl;
    Vector nextStopPostition = stopPositions[path[nextStop]];
    Vector currentStopPostition = stopPositions[path[nextStop - 1]];

    // statie
    model->AddWaypoint (Waypoint (now + stopLength, currentStopPostition));
    // dalsia zastavka
    model->AddWaypoint (Waypoint (timings[id], nextStopPostition));
    // naschedulovanie dalsej zastavky
    Simulator::Schedule (randTime + stopLength, &ScheduleNextStop, model, id);
  }
}


void CBTC::SetStop(int id){
  cout << "STOP reset # " << id << endl;
  resetStop.find(id)-> second = true;
}

// zastavenie elektricky
void CBTC::StopTramMovement(Ptr<WaypointMobilityModel> waypointModel, int id) {


  auto now = Simulator::Now();

  if (resetStop[id]){

      cout << "-- elektricka " << id << " ZASTAV --" << endl;

      Vector pos = waypointModel->GetPosition();
      waypointModel->EndMobility();

      isStopped.find(id)-> second = true;
      resetStop.find(id)-> second = false;
      waypointModel->AddWaypoint(Waypoint(now + stopLength, pos));
      waypointModel->AddWaypoint(Waypoint(stopLength + timings[id], stopPositions[path[tramPositions.at (id)]]));


      Simulator::Schedule (stopLength + (timings[id] - now) + stopLength + Seconds(0.5), &SetStop, id);
      Simulator::Schedule (stopLength + (timings[id] - now), &ScheduleNextStop, waypointModel, id);

  }
}


/********************************** Spustenie ************************************/

void CBTC::Run(int argc = -1, char **argv = nullptr) {

  // CMD line
  if (argc > 0)
    SetCommandLineArgs(argc, argv);


//  packetIntervalTrace = 10;
  // schedulovanie zmeny intervalu paketov po 15tich sekundach

  if (runtimeIntervalChange) {
      // odchytavanie udalosti – zmeny packetIntervalTrace
      TraceConnectWithoutContext ("packetIntervalTrace", MakeCallback (&PacketIntervalChanged));
      // zmena intervalu posielania paketov
      Simulator::Schedule (Seconds(20.0), &RewritePacketInterval, this);
  }
  else {
      packetIntervalTrace = packetInterval;
  }

  // L1
  CreateNodes();

  MobilityHelper mobility;
  StopsConstantPositionModel(mobility);
  TramsWaypointModel(mobility);

  // L2
  SetP2PDevices();
  NetDeviceContainer nicElektricky = SetWifiDevices();

  // L3
  SetRouting(nicElektricky);

  // L4-L7
//  SetApplications();


  // animacia
  if (saveAnim) {

    AnimationInterface anim ("CBTC_animacia.xml");
    anim.EnablePacketMetadata();

    for (uint32_t i = 0; i < stopsN; ++i){
        anim.UpdateNodeColor(stopNodes.Get(i),0,0,200);
        anim.UpdateNodeDescription (stopNodes.Get(i), Names::FindName(stopNodes.Get(i)));
        anim.UpdateNodeSize(stopNodes.Get(i)->GetId(),10,10);
    }

    for (uint32_t i = 0; i < tramsN; ++i){
        anim.UpdateNodeColor(tramNodes.Get(i),0,200,0);
        anim.UpdateNodeDescription (tramNodes.Get(i), Names::FindName(tramNodes.Get(i)));
        anim.UpdateNodeSize(tramNodes.Get(i)->GetId(),10,10);
    }

    NS_LOG_UNCOND("[INFO] Animacia ulozena do CBTC_animacia.xml\n");
//    cout << "[INFO] Animacia ulozena do CBTC_animacia.xml\n";

    Simulator::Stop(Seconds(totalTime));
    Simulator::Run ();
    Simulator::Destroy ();

  } else {

      Simulator::Stop (Seconds(totalTime));
      Simulator::Run ();
      Simulator::Destroy ();
  }

}


/************************************ Main ***************************************/


int main(int argc, char **argv) {
  Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>();
  cbtcExperiment->Run(argc, argv);
  return 0;
}

//vector<string> GetElementsFromContext (string& context) {
//  vector <string> elements;
//  size_t pos1 = 0, pos2;
//  while (pos1 != context.npos)
//    {
//      pos1 = context.find ("/",pos1);
//      pos2 = context.find ("/",pos1 + 1);
//      elements.push_back (context.substr (pos1 + 1,pos2 - (pos1 + 1)));
//      pos1 = pos2;
//      pos2 = context.npos;
//    }
//  return elements;
//}
//
//
//Ptr <Node> GetNodeFromContext (string& context) {
//  vector <string> elements = GetElementsFromContext (context);
//  Ptr <Node> n = NodeList::GetNode (atoi (elements.at (1).c_str ()));
//  NS_ASSERT (n);
//  return n;
//}

