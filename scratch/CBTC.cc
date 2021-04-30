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

1b: Vizualizácia Netanim, PyViz. Pomocou parametru povoľte/zakážte tvorby animácie, preto aby nevznikal/nezanikal súbor nanovo a tým nebrzdil simuláciu

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
   - mena intervalu Hello Paketov
   - na transportnej/aplikacnej/spojovej vrstve

 */

#include "CBTC.h"

#include <ns3/address.h>
#include <ns3/assert.h>
#include <ns3/boolean.h>
#include <ns3/callback.h>
#include <ns3/command-line.h>
#include <ns3/config.h>
#include <ns3/event-id.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-global-routing-helper.h>
#include <ns3/ipv4-interface-address.h>
#include <ns3/log.h>
#include <ns3/names.h>
#include <ns3/node-list.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/position-allocator.h>
#include <ns3/simulator.h>
#include <ns3/trace-helper.h>
#include <ns3/type-id.h>
#include <ns3/waypoint.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/yans-wifi-helper.h>
#include <stddef.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <utility>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Tema 5: Simulacia dopravy – CBTC");

double CBTC::interval = 5.0;
double CBTC::delay = 3.0;
Time CBTC::aktualnyCas = Seconds(0.0);
Time CBTC::stopLength = Seconds(2.0);

map<int, Time> CBTC::timings;
map<int, bool > CBTC::isStopped;
map<int, int> CBTC::tramPositions;

//   cesta: A (0) -> B (1) -> C (2) -> D (3) -> H (7) -> I (8) -> H (7) -> G (6) ->
//          J (9) -> G (6) -> F (5) -> E (4) -> F (5) -> C (2) -> B (1) -> A (0)
vector<int> CBTC::cesta = {0,1,2,3,7,8,7,6,9,6,5,4,5,2,1,0};
vector<string> CBTC::nazvyZastavok = {"A","B", "C", "D", "E", "F", "G", "H", "I", "J"};
vector<Vector> CBTC::pozicieZastavok = { Vector(0.0, 0.0, 0.0),        //A
                                         Vector(0.0, 200.0, 0.0),      //B

                                         Vector(200.0, 100.0, 0.0),    //C
                                         Vector(200.0, 200.0, 0.0),    //D

                                         Vector(300.0, 0.0, 0.0),      //E
                                         Vector(300.0, 100.0, 0.0),    //F

                                         Vector(400.0, 100.0, 0.0),    //G
                                         Vector(400.0, 200.0, 0.0),    //H
                                         Vector(400.0, 300.0, 0.0),    //I

                                         Vector(500.0, 100.0, 0.0) };  //J


CBTC::CBTC() {
  // CMD line argumenty
  pocetZastavok = 10;
  pocetElektriciek = 2;
  velkostUdajov = 1024;
  trvanieSimulacie = 1000.0;
  ulozAnimaciu = false;
  //  logging = true;
}


CBTC::~CBTC() {}


/********************************** Spustenie ************************************/

void CBTC::Run() {

// L1
  CreateNodes();
  
  MobilityHelper mobility;


  for (int i = 0; i < nodyElektricky.GetN(); i++) {
      timings.insert({i, Seconds(5) + Seconds(i * delay)});
      isStopped.insert({i, false});
  }

  ZastavkyConstantPositionModel(mobility);
  ElektrickyWaypointModel(mobility);

  for (int i = 0; i < nodyElektricky.GetN(); i++) {
        Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel>(nodyElektricky.Get(i)->GetObject<MobilityModel>());
        // Schedulovanie zastavok pre jednotilve elektricky
        Simulator::Schedule (timings[i], &ScheduleNextStop, model, i);
  }
  
  Simulator::Schedule (Seconds(5), &Stop, DynamicCast<WaypointMobilityModel>(nodyElektricky.Get(1)->GetObject<MobilityModel>()), 1);
  Simulator::Schedule (Seconds(23), &Stop, DynamicCast<WaypointMobilityModel>(nodyElektricky.Get(1)->GetObject<MobilityModel>()), 1);

// L2
  SetP2PDevices();
  NetDeviceContainer nicElektricky = SetWifiDevices();
// L3
  SetRouting(nicElektricky);
// L4-L5
  SetApplications();

  // callback
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChange));

  // animacia
  if (ulozAnimaciu) {

    AnimationInterface anim ("animations/CBTC_animacia.xml");
    anim.EnablePacketMetadata();

    for (uint32_t i = 0; i < pocetZastavok; ++i){
        anim.UpdateNodeColor(nodyZastavky.Get(i),0,0,200);
        anim.UpdateNodeDescription (nodyZastavky.Get(i), Names::FindName(nodyZastavky.Get(i)));
        anim.UpdateNodeSize(nodyZastavky.Get(i)->GetId(),10,10);
    }

    for (uint32_t i = 0; i < pocetElektriciek; ++i){
        anim.UpdateNodeColor(nodyElektricky.Get(i),0,200,0);
        anim.UpdateNodeDescription (nodyElektricky.Get(i), Names::FindName(nodyElektricky.Get(i)));
        anim.UpdateNodeSize(nodyElektricky.Get(i)->GetId(),10,10);
    }
    cout << endl << "Animacia ulozena do animations/CBTC_animacia.xml" << endl;

    Simulator::Stop(Seconds(trvanieSimulacie));
    Simulator::Run ();
    Simulator::Destroy ();

  } else {

      Simulator::Stop (Seconds (trvanieSimulacie));
      Simulator::Run ();
      Simulator::Destroy ();
  }

}


/************************************* Ping ***************************************/

void CBTC::VytvorSocketyMedziElektrickami (){
  for (int i = 0, port = 80; i < nodyElektricky.GetN () - 1; i++, port--) {

      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

      Ptr<Socket> recvSink = Socket::CreateSocket (nodyElektricky.Get (i + 1), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
      recvSink->Bind (local);
      recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

      Ptr<Socket> source = Socket::CreateSocket (nodyElektricky.Get (i), tid);
      InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), port);
      source->SetAllowBroadcast (true);
      source->Connect (remote);

      sockets.push_back(source);
  }

  //    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  //
  //
  //    Ptr<Socket>  srcSocket = Socket::CreateSocket (nodyElektricky.Get (1), tid);
  //    srcSocket->SetAllowBroadcast (true);
  //    srcSocket->Connect (InetSocketAddress (Ipv4Address (socketAdresses[1]), 80));
  //
  //    Ptr<Socket> recSocket = Socket::CreateSocket (nodyElektricky.Get (0), tid);
  //    recSocket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 80));
  //    recSocket->SetRecvCallback (MakeCallback (&ReceivePacket));
  //
  //  // source posle,receiver dostane
  //  Simulator::ScheduleWithContext (srcSocket->GetNode()->GetId(), Seconds (1.0), &GenerateTraffic, srcSocket, velkostUdajov, 200, Seconds(0.5));

}


void CBTC::PingniZoSource(Ptr<Socket> source, Time time) {
  if(!source){
      NS_LOG_ERROR("Source is null");
      return;
  }

//  cout << source->GetNode()->GetId() << endl;

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (), time, &GenerateTraffic,
                                  source, velkostUdajov, 5, Seconds (1.0));
}


/*********************************** Config **************************************/

void CBTC::SetCommandLineArgs(int argc, char **argv) {
  /* Vypis cmd line argumentov: ./waf --run "CBTC --PrintHelp" */

  CommandLine cmd;
  cmd.AddValue ("ulozAnimaciu", "Ulozenie animacie", ulozAnimaciu);
  cmd.AddValue("velkostUdajov", "Velkost udajov.", velkostUdajov);
  cmd.AddValue("trvanieSimulacie", "Trvanie simulacie.", trvanieSimulacie);
  cmd.AddValue("pocetElektriciek", "Pocet elektriciek.", pocetElektriciek);
  // dalsie parametre ...
  cmd.Parse(argc, argv);
}


void CBTC::CreateNodes() {

  // zastavky
  nodyZastavky.Create(pocetZastavok);
  for (int i = 0; i < pocetZastavok; i++)
      Names::Add (nazvyZastavok[i], nodyZastavky.Get(i));

  // elektricky
  nodyElektricky.Create(pocetElektriciek);
  for (int i = 0; i < pocetElektriciek; i++)
      Names::Add (to_string(i+1), nodyElektricky.Get(i));
}


void CBTC::ZastavkyConstantPositionModel(MobilityHelper &mobility) {

  Ptr <ListPositionAllocator> pozicieZastavokAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: pozicieZastavok) {
      pozicieZastavokAlloc -> Add(pozicia);
  }

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(pozicieZastavokAlloc);
  mobility.Install(nodyZastavky);
}


void CBTC::ElektrickyWaypointModel(MobilityHelper &mobility) {

  mobility.SetMobilityModel("ns3::WaypointMobilityModel", "LazyNotify", BooleanValue(true));
  mobility.Install(nodyElektricky);

  // nastavenie pozicii do mobility modelu
  Ptr <ListPositionAllocator> pozicieElektriciekAlloc = CreateObject<ListPositionAllocator>();
  for( int i = 0; i < pocetElektriciek; i++) {
      pozicieElektriciekAlloc -> Add(Vector(0.0, 0.0, 0.0));
  }

  mobility.SetPositionAllocator(pozicieElektriciekAlloc);

  for (int i = 0; i < pocetElektriciek; i++) {

//      cout << "-- ELEKTRICKA" << i << " --" << endl;
      tramPositions.insert(pair<int, int>(i, 1));
      Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel> (
          nodyElektricky.Get (i)->GetObject<MobilityModel> ());

      //0. zastavka
      model->AddWaypoint (Waypoint(Seconds(i * delay), pozicieZastavok[cesta[0]]));

      //1. zastavka
      model->AddWaypoint (Waypoint(timings[i], pozicieZastavok[cesta[1]]));
  }
}


void CBTC::SetP2PDevices() {

  // spojenia medzi dvojicami zastavok
  vector<vector<string>> spojenia = { {"A","B"}, {"B","C"}, {"C","D"}, {"C","F"}, {"D","F"}, {"D","H"},
                                      {"F","E"}, {"F","G"}, {"F","H"}, {"G","H"}, {"G","J"}, {"H","I"} };

  PointToPointHelper p2pHelper;
  for (auto &spoj: spojenia)
    p2pHelper.Install(spoj[0], spoj[1]);

}


NetDeviceContainer CBTC::SetWifiDevices() {
     // wi-fi channel - elektricky
     YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();
     YansWifiPhyHelper phy;
     phy.SetChannel (wChannel.Create ());

     WifiHelper wifi;
     wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

     // ad-hoc siet
     WifiMacHelper mac;
     mac.SetType ("ns3::AdhocWifiMac");

     return wifi.Install (phy, mac, nodyElektricky);
}


void CBTC::SetRouting(NetDeviceContainer nicElektricky) {

  InternetStackHelper internet;
  internet.Install(nodyElektricky);

  // elektricky: subnet 10.1.1.0 maska 255.255.255.0
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  ipv4.Assign(nicElektricky);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}


void CBTC::SetApplications() {

  VytvorSocketyMedziElektrickami();

//  kontrolovanie vzdialenosti medzi elektrickami
  Simulator::Schedule (Seconds(5), &CheckDistances, nodyElektricky);

  // ping z prvej elektricky
  PingniZoSource(sockets[0], Seconds(5.0));

  // ping z druhej elektricky
  PingniZoSource(sockets[1], Seconds(10.0));
}


/********************************** Callbacky *************************************/

void CBTC::CourseChange (string context, Ptr<const MobilityModel> model) {

  //  cout << elektricka->GetId() << ": " << waypointMobModel->WaypointsLeft() << endl;
  //  cout << Simulator::Now ()<< ", x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;


  /*Vector pos = model->GetPosition ();
  Ptr<Node> elektricka = GetNodeFromContext(context);
  Ptr<WaypointMobilityModel> waypointModel = DynamicCast<WaypointMobilityModel>(elektricka->GetObject<MobilityModel>());

  // ked presiel vsetkymi waypointmi, pridaju sa mu znova
  if (waypointModel->WaypointsLeft() == 0) {

      int id_elektricky = elektricka->GetId() - 10;  // elektricky maju v mobility modeli idcka 10,11,12 preto id-10

      aktualnyCas = Simulator::Now() + Seconds(interval + id_elektricky * delay); // posunie sa cas

      // pridaju sa waypointy
      for (int j = 1; j < cesta.size(); j++) {
          auto cas = aktualnyCas + Seconds(interval * j + id_elektricky * delay);
          auto pozicia = pozicieZastavok[cesta[j]];
          waypointModel->AddWaypoint (Waypoint(cas,pozicia));
      }

    }*/
}


void CBTC::CheckDistances (NodeContainer nodyElektricky){

  for (int i = 0; i < nodyElektricky.GetN () - 1; i++){
      Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel> (
          nodyElektricky.Get (i)->GetObject<MobilityModel> ());
      Ptr<WaypointMobilityModel> nextModel = DynamicCast<WaypointMobilityModel> (
          nodyElektricky.Get (i + 1)->GetObject<MobilityModel> ());

      cout << "ELEKTRICKA #" << i << " ---|" << model->GetDistanceFrom (nextModel)
           << "|--- " << "ELEKTRICKA #" << (i + 1) << endl;

      if(model->GetDistanceFrom (nextModel) < 130.0){
//          Stop(DynamicCast<WaypointMobilityModel>(nodyElektricky.Get(i + 1)->GetObject<MobilityModel>()), i + 1);
//          Simulator::Schedule (Seconds (0.1), &Stop, );
//          Simulator::Schedule (Seconds(5), &Stop, nextModel, i + 1);

       }
  }

  Simulator::Schedule (Seconds (2), &CheckDistances, nodyElektricky);
}


void CBTC::ReceivePacket (Ptr<Socket> socket) {

  while (Ptr<Packet> packet = socket->Recv()){

      cout << "\nping z " << socket -> GetNode() -> GetId() - 10 << " elektricky" << endl;

      int packetSize = packet->GetSize();
      uint8_t *buffer = new uint8_t[packetSize];

      int size = packet->CopyData(buffer, packetSize);
      string packetContent = string(buffer, buffer + packetSize);

      Ptr<Ipv4> ipv4 = socket -> GetNode() -> GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);

      cout<< "Receiver: " << iaddr.GetLocal() << endl;
      cout<< "Received:" << packetContent << endl << endl;
    }
}


void CBTC::GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval ) {
  Ptr<Ipv4> ipv4 = socket -> GetNode() -> GetObject<Ipv4> ();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);

  ostringstream msg; msg << "Hello World from " << iaddr.GetLocal() << '\0';

  if (pktCount > 0) {
      uint16_t packetSize = msg.str().length()+1;
      Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), packetSize);
      socket->Send (packet);
      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
  } else {
      socket->Close ();
  }
}


void CBTC::ScheduleNextStop (Ptr<WaypointMobilityModel> model, int id){

  if (isStopped[id]){
      isStopped.find (id) -> second = false;
  } else {
    int nextStop = tramPositions.at (id) + 1;

    if (nextStop == cesta.size () - 1){
        tramPositions.find (id)->second = 0;
        nextStop = 0;
    } else {
        tramPositions.find (id)->second = nextStop;
    }

    // nastavenie nahodnej rychlosti
    Time randTime = Seconds(rand() % 11 + 5);
    timings.find(id) -> second = Simulator::Now () + randTime + stopLength;

    cout << "ELEKTRICKA #" << id << " next stop -> " << nextStop << endl;
    Vector nextStopPostition = pozicieZastavok[cesta[nextStop]];
    Vector currentStopPostition = pozicieZastavok[cesta[nextStop - 1]];

    // statie
    model->AddWaypoint (Waypoint (Simulator::Now () + stopLength, currentStopPostition));
    // dalsia zastavka
    model->AddWaypoint (Waypoint (timings[id], nextStopPostition));
    // naschedulovanie dalsej zastavky
    Simulator::Schedule (randTime + stopLength, &ScheduleNextStop, model, id);
  }
}


// zastavenie elektricky
void CBTC::Stop(Ptr<WaypointMobilityModel> waypointModel, int id) {
  isStopped.find(id)-> second = true;
  Vector pos = waypointModel->GetPosition();

  waypointModel->EndMobility();
  waypointModel->AddWaypoint(Waypoint(Simulator::Now() + stopLength, pos));
  cout << "next stop at" <<stopLength + timings[id] << endl;
  waypointModel->AddWaypoint(Waypoint(stopLength + timings[id], pozicieZastavok[cesta[tramPositions.at (id)]]));

  Simulator::Schedule (stopLength + (timings[id] - Simulator::Now()), &ScheduleNextStop, waypointModel, id);
}


/********************************** Other *************************************/


vector<string> GetElementsFromContext (string& context) {
  vector <string> elements;
  size_t pos1 = 0, pos2;
  while (pos1 != context.npos)
    {
      pos1 = context.find ("/",pos1);
      pos2 = context.find ("/",pos1 + 1);
      elements.push_back (context.substr (pos1 + 1,pos2 - (pos1 + 1)));
      pos1 = pos2;
      pos2 = context.npos;
    }
  return elements;
}


Ptr <Node> GetNodeFromContext (string& context) {
  vector <string> elements = GetElementsFromContext (context);
  Ptr <Node> n = NodeList::GetNode (atoi (elements.at (1).c_str ()));
  NS_ASSERT (n);
  return n;
}


/************************************ Main ***************************************/

int main(int argc, char **argv) {
  //      model->AddWaypoint (Waypoint(cas + timings[i].second, pozicia));
        //timings.find(i)-> second = make_pair(timings[i].first, cas);
  CBTC cbtcExperiment;
  cbtcExperiment.SetCommandLineArgs(argc, argv);
  cbtcExperiment.Run();

  return 0;
}
