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

✔ 2b Popis vhodného výberu ISO OSI (V úvodnom komentári popíšte dôvody výberu jednotlivých protokolov)

✔ 1b použitie časových udalostí (záznam o zmene)
    - Simulator::Schedule

✔ 2b použitie udalostí zmenu stavu (záznam o zmene);(atribútu modelu)
    - Course change

✔ 3b použitie vlastnej triedy v projekte (jeho zmena vlastnosti, využitie NS3 funkcionality udalostný systém)
    - bud zdedena/rozsirenie povodnej alebo totalne nova

✔ 2b v čase simulácie zmena v modeli L1 fyzické médium, pohyb, útlm …
    - zmeni sa spravanie uzlu - mobility model

✔ 3b v čase simulácie zmena v modelu L2-L5
   - zmenim atribut nejakeho smerovacieho protokolu - kolko paketov posiela atd
   - zmena intervalu Hello Paketov
   - na transportnej/aplikacnej/spojovej vrstve

 */

#include "CBTC.h"

#include <ns3/address.h>
#include <ns3/animation-interface.h>
#include <ns3/aodv-helper.h>
#include <ns3/boolean.h>
#include <ns3/callback.h>
#include <ns3/command-line.h>
#include <ns3/config.h>
#include <ns3/double.h>
#include <ns3/event-id.h>
#include <ns3/fatal-error.h>
#include <ns3/flow-classifier.h>
#include <ns3/flow-monitor.h>
#include <ns3/flow-monitor-helper.h>
#include <ns3/gnuplot.h>
#include <ns3/inet-socket-address.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4.h>
#include <ns3/ipv4-address.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-flow-classifier.h>
#include <ns3/ipv4-global-routing-helper.h>
#include <ns3/ipv4-interface-address.h>
#include <ns3/log.h>
#include <ns3/log-macros-disabled.h>
#include <ns3/names.h>
#include <ns3/node.h>
#include <ns3/object.h>
#include <ns3/olsr-helper.h>
#include <ns3/packet.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/position-allocator.h>
#include <ns3/rng-seed-manager.h>
#include <ns3/simulator.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/type-id.h>
#include <ns3/waypoint.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/yans-wifi-helper.h>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Tema 5: Simulacia dopravy – CBTC");

/************************** staticke premenne ******************************/

//double CBTC::interval = 5.0;
double CBTC::delay = 3.0;
Time CBTC::stopLength = Seconds(2.0);
Time CBTC::pauseLength = Seconds(5.0);
int32_t CBTC::collisionCounter = 0;
int32_t CBTC::stopCounter = 0;
int32_t CBTC::sentPacketsCounter = 0;
int32_t CBTC::receivedPacketsCounter = 0;

map<int, Time> CBTC::timings;
map<int, bool > CBTC::isStopped;
map<int, bool > CBTC::resetStop;
map<int, int> CBTC::tramPositions;

//A(0) -> B(1) -> D(3) -> E(4) -> I(8) -> F(5) -> H(7) -> J(9) -> G(6) -> C(2) -> A(0)
vector<int> CBTC::path = {0,1,3, 4, 8, 5, 7, 9, 6, 2, 0};
vector<string> CBTC::stopNames = {"A","B", "C", "D", "E", "F", "G", "H", "I", "J"};

vector<Vector> CBTC::stopPositions = { Vector(0.0, 0.0, 0.0),        //A
                                       Vector(-50.0, 200.0, 0.0),        //B
                                       Vector(150.0, 0.0, 0.0),        //C
                                       Vector(100.0, 200.0, 0.0),        //D
                                       Vector(100.0, 400.0, 0.0),        //E
                                       Vector(200.0, 200.0, 0.0),        //F
                                       Vector(300.0, 0.0, 0.0),        //G
                                       Vector(300.0, 200.0, 0.0),        //H
                                       Vector(300.0, 400.0, 0.0),        //I
                                       Vector(500.0, 200.0, 0.0),        //J
};




CBTC::CBTC(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
           uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true, int32_t range = 100) {

  // CMD line argumenty
  this->tramsN = tramsN;
  this->packetSize = packetSize;
  this->totalTime = totalTime;
  this->saveAnim = saveAnim;
  this->protocol = protocol;
  this->packetInterval = packetInterval;
  this->runtimeIntervalChange = runtimeIntervalChange;
  this->range = range;
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


/********************************** Pakety *************************************/


void CBTC::ReceivePacket (Ptr<Socket> socket) {


  Ptr<Node> socketNode = socket -> GetNode();

  while (Ptr<Packet> packet = socket->Recv()) {

      // elektricka zastavi

      Ptr<WaypointMobilityModel> waypointModel = DynamicCast<WaypointMobilityModel>(socketNode -> GetObject<MobilityModel>());
      StopTramMovement(waypointModel, socketNode -> GetId() - 10);


      // vypis paketu
      int packSize = packet->GetSize();
      uint8_t *buffer = new uint8_t[packSize];

      int size = packet->CopyData(buffer, packSize);
      string packetContent = string(buffer, buffer + packSize);
      packetContent.resize(packetContent.find('\0')); // odstrani sa padding z paketu

      Ptr<Ipv4> ipv4 = socketNode -> GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);

      //NS_LOG_UNCOND("[INFO] " << Simulator::Now() << " | Receiver: " << iaddr.GetLocal() << " | "
      //                        << "Received ("<< size<< " B): " << packetContent);

      receivedPacketsCounter++;
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
  sentPacketsCounter++;

  Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktInterval);

}


/****************************** Pohyb elektriciek ********************************/


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
    Time randTime = Seconds(rand() % 7 + 5);
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


void CBTC::ResetStop(int id){
  cout << "STOP reset # " << id << endl;
  resetStop.find(id)-> second = true;
}

// zastavenie elektricky
void CBTC::StopTramMovement(Ptr<WaypointMobilityModel> waypointModel, int id) {


  Time now = Simulator::Now();

  if (resetStop[id] && (waypointModel->GetNextWaypoint().position != stopPositions[0])){
      stopCounter ++;

      cout << "-- elektricka " << id << " ZASTAV --" << stopCounter << endl;

      Vector pos = waypointModel->GetPosition();
      waypointModel->EndMobility();

      isStopped.find(id)-> second = true;
      resetStop.find(id)-> second = false;
      waypointModel->AddWaypoint(Waypoint(now + pauseLength, pos));
      waypointModel->AddWaypoint(Waypoint(pauseLength + timings[id], stopPositions[path[tramPositions.at (id)]]));


      Simulator::Schedule (pauseLength + (timings[id] - now) + stopLength + Seconds(0.5), &ResetStop, id);
      Simulator::Schedule (pauseLength + (timings[id] - now), &ScheduleNextStop, waypointModel, id);

  }
}


void CBTC::CheckDistances (NodeContainer tramNodes){

  for (int i = 0; i < tramNodes.GetN() - 1; i++){
      Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel> (
          tramNodes.Get (i)->GetObject<MobilityModel> ());
      Ptr<WaypointMobilityModel> nextModel = DynamicCast<WaypointMobilityModel> (
          tramNodes.Get (i + 1)->GetObject<MobilityModel> ());

      // vzdialenost 0 maju na zaciatku, pocas simulacie sa rata ako kolizia cislo > 0
      if(model->GetDistanceFrom (nextModel) > 0.0 && model->GetDistanceFrom (nextModel) < 5.0) {
          collisionCounter++;
          cout << "#" << i << " si moc blizko: " << model->GetDistanceFrom (nextModel)  << " counter " << collisionCounter << endl;

       }
  }

  Simulator::Schedule (Seconds (2), &CheckDistances, tramNodes);
}

/*********************************** Config **************************************/

void CBTC::SetCommandLineArgs(int argc, char **argv) {
  /*
   * Vypis cmd line argumentov: ./waf --run "CBTC --PrintHelp"
   * Spustenie s argumentami:   ./waf --run "CBTC --ulozAnimaciu=true --velkostUdajov=100" atd
  */

  CommandLine cmd;

  cmd.AddValue ("saveAnim", "Ulozenie animacie", saveAnim);
  cmd.AddValue ("packetSize", "Velkost paketu.", packetSize);
  cmd.AddValue ("totalTime", "Trvanie simulacie.", totalTime);
  cmd.AddValue ("tramN", "Pocet elektriciek.", tramsN);
  cmd.AddValue ("protocol", "Routovaci protokol: 1=OLSR;2=AODV", protocol);
  cmd.AddValue ("packetInterval", "Interval posielania hello paketov", packetInterval);
  cmd.AddValue ("runtimeIntervalChange", "Zmena intervalu posielania paketov pocas behu simulacie true/false", runtimeIntervalChange);
  cmd.AddValue ("range", "Rozsah signalu", range);
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
                << " --runtimeIntervalChange=" << runtimeIntervalChange
                << " --range=" << range << endl);

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

  Ptr <ListPositionAllocator> stopPositionsAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: stopPositions) {
      stopPositionsAlloc -> Add(pozicia);
  }

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(stopPositionsAlloc);
  mobility.Install(stopNodes);
}


void CBTC::TramsWaypointModel(MobilityHelper &mobility) {

  mobility.SetMobilityModel("ns3::WaypointMobilityModel", "LazyNotify", BooleanValue(true));
  mobility.Install(tramNodes);

  // nastavenie pozicii do mobility modelu
  Ptr <ListPositionAllocator> pozicieElektriciekAlloc = CreateObject<ListPositionAllocator>();
  for( int i = 0; i < tramsN; i++) {
      pozicieElektriciekAlloc -> Add(Vector(0.0, 0.0, 0.0));
  }

  mobility.SetPositionAllocator(pozicieElektriciekAlloc);


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

  vector<vector<string>> pairs = { {"A","B"}, {"A","C"}, {"B","D"}, {"C","G"}, {"D","E"}, {"D","F"}, {"F","G"},
                                   {"F","H"}, {"F","I"}, {"G","J"}, {"H","I"}, {"H","J"}, {"I","J"}, {"E","I"} };

  PointToPointHelper p2pHelper;
  for (auto &p: pairs)
    p2pHelper.Install(p[0], p[1]);

}


NetDeviceContainer CBTC::SetWifiDevices() {
     // wi-fi channel - elektricky
     YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();

     Config::SetDefault ("ns3::RangePropagationLossModel::MaxRange", DoubleValue (range));
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


  switch (protocol) {
    case 1:
      internet.SetRoutingHelper(aodv);
      break;
    case 2:
      internet.SetRoutingHelper(olsr);
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

  for (int i = 0, port = 80; i < tramNodes.GetN () - 1; i++, port--) {

      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

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


  for (int i = 0; i < tramsN-1; i++) {
    Simulator::ScheduleWithContext (sockets[i]->GetNode ()->GetId (), Seconds(3.0+i), &GenerateTraffic,
                                    sockets[i], packetSize, Seconds (packetIntervalTrace));
  }

}


/********************************** Spustenie ************************************/


void CBTC::Run(int argc = -1, char **argv = nullptr) {

  // CMD line
  if (argc > 0)
      SetCommandLineArgs(argc, argv);

  if (runtimeIntervalChange) {
      // odchytavanie udalosti – zmeny packetIntervalTrace
      TraceConnectWithoutContext ("packetIntervalTrace", MakeCallback (&PacketIntervalChanged));
      // zmena intervalu posielania paketov po 20 sekundach
      Simulator::Schedule (Seconds(20.0), &RewritePacketInterval, this);
  }
  else {
      packetIntervalTrace = packetInterval;
  }

  collisionCounter = 0;
  stopCounter = 0;
  sentPacketsCounter = 0;
  receivedPacketsCounter = 0;

  timings = {};
  isStopped = {};
  resetStop = {};
  tramPositions = {};

  CreateNodes();
  delay = tramsN;
  pauseLength = Seconds(tramsN + 2.0);

  MobilityHelper mobility;
  StopsConstantPositionModel(mobility);
  TramsWaypointModel(mobility);

  Simulator::Schedule (Seconds (2), &CheckDistances, tramNodes);

// L2
  SetP2PDevices();

  // L1
  NetDeviceContainer nicElektricky = SetWifiDevices ();
  // L3
  SetRouting (nicElektricky);
  // L4-L5
  SetApplications ();

  // NetAnim
  if (saveAnim)
    {

      AnimationInterface anim ("CBTC_animacia.xml");
      anim.EnablePacketMetadata ();

      for (uint32_t i = 0; i < stopsN; ++i)
        {
          anim.UpdateNodeColor (stopNodes.Get (i), 0, 0, 200);
          anim.UpdateNodeDescription (stopNodes.Get (i),
                                      Names::FindName (stopNodes.Get (i)));
          anim.UpdateNodeSize (stopNodes.Get (i)->GetId (), 10, 10);
        }

      for (uint32_t i = 0; i < tramsN; ++i)
        {
          anim.UpdateNodeColor (tramNodes.Get (i), 0, 200, 0);
          anim.UpdateNodeDescription (tramNodes.Get (i),
                                      Names::FindName (tramNodes.Get (i)));
          anim.UpdateNodeSize (tramNodes.Get (i)->GetId (), 10, 10);
        }

      NS_LOG_UNCOND("[INFO] Animacia ulozena do CBTC_animacia.xml\n");
      Simulator::Stop (Seconds (totalTime));
      Simulator::Run ();

      Simulator::Destroy ();
    }
  else
    {
      Simulator::Stop (Seconds (totalTime));
      Simulator::Run ();

      Simulator::Destroy ();
    }

}


void visualize (vector<int> X, vector<double> avgY, vector<double> devY, string title, string xAxis, string yAxis, string filename) {


  Gnuplot graf (filename + ".svg");
  graf.SetTerminal ("svg");
  graf.SetTitle (title);
  graf.SetLegend (xAxis, yAxis);

  Gnuplot2dDataset data;
  data.SetTitle("graf");
  data.SetStyle (Gnuplot2dDataset::LINES_POINTS);

  Gnuplot2dDataset errorBars;
  errorBars.SetTitle("smerodajna odchylka");
  errorBars.SetStyle(Gnuplot2dDataset::POINTS);
  errorBars.SetErrorBars(Gnuplot2dDataset::Y);

  for ( int i = 0; i < X.size(); i++) {
    data.Add(X[i], avgY[i]);
    errorBars.Add(X[i],avgY[i],devY[i]);
  }

  graf.AddDataset(errorBars);
  graf.AddDataset(data);

  std::ofstream plotFile (filename + ".plt");
  graf.GenerateOutput (plotFile);
  plotFile.close ();
  if (system (("gnuplot " + filename + ".plt").c_str()));
}

double average(vector<int> data){

  double sum = 0.0;

  for(int i = 0; i < data.size(); ++i)
      {
          sum += data[i];
      }

  return sum/data.size();

}

double deviation(vector<int> data) {
    double sum = 0.0, standardDeviation = 0.0;

    double mean = average(data);

    for(int i = 0; i < data.size(); ++i)
        standardDeviation += pow(data[i] - mean, 2);

    return sqrt(standardDeviation / data.size());
}


/************************************ Main ***************************************/


void CollisionsOverTime(int protocol) {

  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> timesX;
  vector<double> avgCollisionsY;
  vector<double> devCollisionsY;

  for (int testNumber = 0, totalTime = 200; testNumber < 10; testNumber++, totalTime+=15) {

        vector<int> collisionCounts;

        for(int f=0; f < 3; f++) {
            /*(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
                       uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true)*/
            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(4, 50, totalTime, false, protocol, 1, true, 100);
            cbtcExperiment->Run();
            collisionCounts.push_back(cbtcExperiment->collisionCounter);

        }

        timesX.push_back(totalTime);
        avgCollisionsY.push_back(average(collisionCounts));
        devCollisionsY.push_back(deviation(collisionCounts));

  }
  visualize(timesX,
            avgCollisionsY,
            devCollisionsY,
            "Number of collisions over time ("+protocolStr+")",
            "Time [s]",
            "Collisions",
            "CollisionsOverTime"+protocolStr);

}


void CollisionsTramCount(int protocol){
  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> tramCountX;
  vector<double> avgCollisionsY;
  vector<double> devCollisionsY;

  for (int testNumber = 0, tramCount = 2; testNumber < 10; testNumber++, tramCount++) {

        vector<int> collisionCounts;

        for(int f=0; f < 3; f++) {
            /*(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
                       uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true)*/
            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(tramCount, 50, 50.0, false, protocol, 1, true, 100);
            cbtcExperiment->Run();
            collisionCounts.push_back(cbtcExperiment->collisionCounter);

        }

        tramCountX.push_back(tramCount);
        avgCollisionsY.push_back(average(collisionCounts));
        devCollisionsY.push_back(deviation(collisionCounts));

  }
  visualize(tramCountX,
            avgCollisionsY,
            devCollisionsY,
            "Number of collisions in relation to number of trams during 50 sec ("+protocolStr+")",
            "Number of trams",
            "Collisions",
            "CollisionsTramCount"+protocolStr);
}


void StopTramCount(int protocol){
  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> tramCountX;
  vector<double> avgStopsY;
  vector<double> devStopsY;

  for (int testNumber = 0, tramCount = 2; testNumber < 10; testNumber++, tramCount++) {

        vector<int> stopCounts;

        for(int f=0; f < 3; f++) {
            /*(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
                       uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true)*/
            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(tramCount, 50, 50.0, false, protocol, 1, false, 100);
            cbtcExperiment->Run();
            stopCounts.push_back(cbtcExperiment->stopCounter);

        }

        tramCountX.push_back(tramCount);
        avgStopsY.push_back(average(stopCounts));
        devStopsY.push_back(deviation(stopCounts));

  }
  visualize(tramCountX,
            avgStopsY,
            devStopsY,
            "Number of stops in relation to number of trams during 50 sec ("+protocolStr+")",
            "Number of trams",
            "Number of stops",
            "StopTramCount"+protocolStr);
}


/*void StopCollisionCount(int protocol){
  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> stopCountX;
  vector<double> avgCollisionsY;
  vector<double> devCollisionsY;

  for (int testNumber = 0; testNumber < 10; testNumber++) {

        vector<int> collisionCounts;
        vector<int> stopCounts;

        for(int f=0; f < 3; f++) {

            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(5, 50, 100.0, false, protocol, 1, false);
            cbtcExperiment->Run();
            stopCounts.push_back(cbtcExperiment->stopCounter);
            collisionCounts.push_back(cbtcExperiment->collisionCounter);

        }

        stopCountX.push_back(average(stopCounts));
        avgCollisionsY.push_back(average(collisionCounts));
        devCollisionsY.push_back(deviation(collisionCounts));

  }



  visualize(stopCountX,
            avgCollisionsY,
            devCollisionsY,
            "Number of stops in relation to number of collisions during 100 sec ("+protocolStr+")",
            "Number of stops",
            "Number of collisions",
            "StopCollisionCount"+protocolStr);
}*/


// range posielania paketov x pocet paketov
void RangePacketCount(int protocol) {
  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> rangeX;
  vector<double> avgRecPacketsY;
  vector<double> devRecPacketsY;

  for (int testNumber = 0, range = 20; testNumber < 10; testNumber++, range += 20) {

        vector<int> recPackets;

        for(int f=0; f < 3; f++) {
            /*(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
                       uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true)*/
            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(4, 50, 50.0, false, protocol, 1, false, range);
            cbtcExperiment->Run();
            recPackets.push_back(cbtcExperiment->receivedPacketsCounter);
        }

        rangeX.push_back(range);
        avgRecPacketsY.push_back(average(recPackets));
        devRecPacketsY.push_back(deviation(recPackets));

  }
  visualize(rangeX,
            avgRecPacketsY,
            devRecPacketsY,
            "Range of signal in relation to number of received packets during 50 sec ("+protocolStr+")",
            "Range of signal [m]",
            "Number of received packets",
            "RangePacketCount"+protocolStr);
}


//pocet elektriciek x pocet stratenych paketov
void TramCountLostPackets(int protocol){
  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> tramCountX;
  vector<double> avgLostPacketsY;
  vector<double> devLostPacketsY;

  for (int testNumber = 0, tramCount = 2; testNumber < 10; testNumber++, tramCount++) {

        vector<int> lostPacketsCounts;

        for(int f=0; f < 3; f++) {
            /*(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
                       uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true)*/
            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(tramCount, 50, 50.0, false, protocol, 1, false, 100);
            cbtcExperiment->Run();

            int rec = cbtcExperiment->receivedPacketsCounter;
            int sent = cbtcExperiment->sentPacketsCounter;

            lostPacketsCounts.push_back(sent-rec);

        }

        tramCountX.push_back(tramCount);
        avgLostPacketsY.push_back(average(lostPacketsCounts));
        devLostPacketsY.push_back(deviation(lostPacketsCounts));

  }
  visualize(tramCountX,
            avgLostPacketsY,
            devLostPacketsY,
            "Number of trams in relation to number of lost packets during 50 sec ("+protocolStr+")",
            "Number of trams",
            "Number of lost packets",
            "TramCountLostPackets"+protocolStr);
}


// pomer prijatych paketov k odoslanym paketom za cas (v %)
void ReceivedPacketsRatioOverTime(int protocol){
  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> timesX;
  vector<double> avgRecPacketsRatioY;
  vector<double> devRecPacketsRatioY;

  for (int testNumber = 0, totalTime = 10; testNumber < 10; testNumber++, totalTime+=5) {

        vector<int> recPacketsRatios;

        for(int f=0; f < 5; f++) {
            /*(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
                       uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true)*/
            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(4, 50, totalTime, false, protocol, 1, false, 100);
            cbtcExperiment->Run();

            int rec = cbtcExperiment->receivedPacketsCounter;
            int sent = cbtcExperiment->sentPacketsCounter;

            double percent = ((double)rec)/(double)sent * 100.0;
            cout << percent << endl;

            recPacketsRatios.push_back(percent);
        }

        timesX.push_back(totalTime);
        avgRecPacketsRatioY.push_back(average(recPacketsRatios));
        devRecPacketsRatioY.push_back(deviation(recPacketsRatios));
  }

  visualize(timesX,
            avgRecPacketsRatioY,
            devRecPacketsRatioY,
            "Ratio of received packets to sent packets (in %) over time ("+protocolStr+")",
            "Time [s]",
            "Received packets / sent packets [%]",
            "ReceivedPacketsRatioOverTime"+protocolStr);
}


// velkost intervalu paketov x pocet kolizii
void IntervalSizeCollisionCount(int protocol){
  string protocolStr = protocol == 1 ? "AODV" : "OLSR";

  vector<int> intervalSizeX;
  vector<double> avgCollisionCountY;
  vector<double> devCollisionCountY;

  for (int testNumber = 0, interval = 1; testNumber < 10; testNumber++, interval++) {

        vector<int> collisionCounts;

        for(int f=0; f < 5; f++) {
            /*(uint32_t tramsN = 4, uint32_t packetSize = 50, uint32_t totalTime = 300, bool saveAnim = true,
                       uint32_t protocol = 2, uint32_t packetInterval = 1, bool runtimeIntervalChange = true)*/
            Names::Clear();
            Ptr<CBTC> cbtcExperiment = CreateObject<CBTC>(5, 50, 200.0, false, protocol, interval, false, 100);
            cbtcExperiment->Run();

            collisionCounts.push_back(cbtcExperiment->collisionCounter);

        }

        intervalSizeX.push_back(interval);
        avgCollisionCountY.push_back(average(collisionCounts));
        devCollisionCountY.push_back(deviation(collisionCounts));

  }
  visualize(intervalSizeX,
            avgCollisionCountY,
            devCollisionCountY,
            "Number of collisions in relation to interval size during 50 sec ("+protocolStr+")",
            "Interval size [s]",
            "Number of collisions",
            "IntervalSizeCollisionCount"+protocolStr);
}



int main(int argc, char **argv) {

  SeedManager::SetSeed(rand());
  SeedManager::SetRun(10);

  //CollisionsOverTime(1);
  //CollisionsOverTime(2);

  //CollisionsTramCount(1);
  //CollisionsTramCount(2);

  //StopTramCount(1);
  //StopTramCount(2);

  //RangePacketCount(1);
  //RangePacketCount(2);

  //TramCountLostPackets(1);
  //TramCountLostPackets(2);

  //ReceivedPacketsRatioOverTime(1);
  //ReceivedPacketsRatioOverTime(2);

  //IntervalSizeCollisionCount(1);
  IntervalSizeCollisionCount(2);


  cout << "\nvybafco" << endl;


  return 0;
}

