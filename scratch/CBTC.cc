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
    -

1b použitie časových udalostí (záznam o zmene)
    - Simulator::Schedule

✔ 2b použitie udalostí zmenu stavu (záznam o zmene);(atribútu modelu)
    - Course change

3b použitie vlastnej triedy v projekte (jeho zmena vlastnosti, využitie NS3 funkcionality udalostný systém)
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
#include <ns3/animation-interface.h>
#include <ns3/assert.h>
#include <ns3/boolean.h>
#include <ns3/callback.h>
#include <ns3/command-line.h>
#include <ns3/config.h>
#include <ns3/csma-helper.h>
#include <ns3/event-id.h>
#include <ns3/inet-socket-address.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4.h>
#include <ns3/ipv4-address.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-global-routing-helper.h>
#include <ns3/ipv4-interface-address.h>
#include <ns3/log.h>
#include <ns3/names.h>
#include <ns3/node-list.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/position-allocator.h>
#include <ns3/simulator.h>
#include <ns3/type-id.h>
#include <ns3/waypoint.h>
#include <ns3/waypoint-mobility-model.h>
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
  pocetElektriciek = 3;
  velkostUdajov = 1024;
  trvanieSimulacie = 200.0;
  ulozAnimaciu = true;
  //  logging = true;
}

CBTC::~CBTC() {}

/********************************** Spustenie ************************************/

void CBTC::Stop(Ptr<Node> elektricka) {
  Ptr<WaypointMobilityModel> waypointModel = DynamicCast<WaypointMobilityModel> (elektricka->GetObject<MobilityModel> ());
  auto pos = waypointModel->GetPosition();
  waypointModel->EndMobility();
  waypointModel->AddWaypoint(Waypoint(Simulator::Now()+Seconds(1.0), pos));

}


void CBTC::Run() {

// L1
  CreateNodes();
  zastavkyConstantPositionModel();
  elektrickyWaypointModel();
// L2
  SetCsmaDevices();
  SetWifiDevices();
// L3
  SetRouting();
// L4-L5
  SetApplications();

  // callback
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeBoundCallback (&CourseChange, tramPositions, waypoints));


  // animacia

  if (ulozAnimaciu) {
      // animacia
      AnimationInterface anim ("animacia_tema5.xml");
      anim.EnablePacketMetadata();

        for (uint32_t i = 0; i < pocetZastavok; ++i){
            anim.UpdateNodeColor(nodyZastavky.Get(i),0,0,200);
            anim.UpdateNodeDescription (nodyZastavky.Get(i), Names::FindName(nodyZastavky.Get(i)));
            anim.UpdateNodeSize(nodyZastavky.Get(i)->GetId(),10,10);
        }


        for (uint32_t i = 1; i < pocetElektriciek; ++i){
            anim.UpdateNodeColor(nodyElektricky.Get(i),0,200,0);
            anim.UpdateNodeDescription (nodyElektricky.Get(i), Names::FindName(nodyElektricky.Get(i)));
            anim.UpdateNodeSize(nodyElektricky.Get(i)->GetId(),10,10);
        }

  }

  Simulator::Schedule (Seconds(8.5), &Stop, nodyElektricky.Get(0)); // test zastavenia elektricky


  Simulator::Stop (Seconds (trvanieSimulacie));
  Simulator::Run();
  Simulator::Destroy();

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


void CBTC::zastavkyConstantPositionModel() {

  Ptr <ListPositionAllocator> pozicieZastavokAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: pozicieZastavok) {
      pozicieZastavokAlloc -> Add(pozicia);
  }

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(pozicieZastavokAlloc);
  mobility.Install(nodyZastavky);

}


void CBTC::elektrickyWaypointModel() {

  mobility.SetMobilityModel("ns3::WaypointMobilityModel", "LazyNotify",BooleanValue(true));
  mobility.Install(nodyElektricky);

  // nastavenie pozicii do mobility modelu
  Ptr <ListPositionAllocator> pozicieElektriciekAlloc = CreateObject<ListPositionAllocator>();
  for( int i = 0; i < pocetElektriciek; i++) {
      pozicieElektriciekAlloc -> Add(Vector(0.0, 0.0, 0.0));
  }
  mobility.SetPositionAllocator(pozicieElektriciekAlloc);

  /* waypoint model pre elektricky
       * vsetky maju rovnaku cestu, iba maju posunute intervaly
       * zastavuju na zastavkach
       * TODO: treba aby si posielali spravy a podla toho spomalovali/zrychlovali
  */

  for (int i = 0; i < pocetElektriciek; i++) {

      tramPositions.insert(pair<int, int>(i + 10, 0));
      Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel>( nodyElektricky.Get(i)->GetObject<MobilityModel>() );

      auto cas = aktualnyCas + Seconds(interval * 0 + i * delay);
      auto pozicia = pozicieZastavok[cesta[0]];
      model->AddWaypoint (Waypoint(cas,pozicia));
      waypoints[i + 10].push(cesta[0]);

      for (int j = 1; j < cesta.size(); j++) {
          auto cas = aktualnyCas + Seconds(interval * j + i * delay);
          auto pozicia = pozicieZastavok[cesta[j]];

//          cout << "zastavka: " << cesta[j] << " cas: " << cas  << endl;
          model->AddWaypoint (Waypoint(cas,pozicia));
          model->AddWaypoint (Waypoint(cas + Seconds(2),pozicia));
          waypoints[i + 10].push(cesta[j]);
          waypoints[i + 10].push(cesta[j]);

      }
  }
}


void CBTC::SetCsmaDevices() {

  // spojenia medzi dvojicami zastavok
  vector<NodeContainer> spojenia = { NodeContainer(NodeContainer("A"),NodeContainer("B")),
                                     NodeContainer(NodeContainer("B"),NodeContainer("C")),
                                     NodeContainer(NodeContainer("C"),NodeContainer("D")),
                                     NodeContainer(NodeContainer("C"),NodeContainer("F")),
                                     NodeContainer(NodeContainer("D"),NodeContainer("F")),
                                     NodeContainer(NodeContainer("D"),NodeContainer("H")),
                                     NodeContainer(NodeContainer("F"),NodeContainer("E")),
                                     NodeContainer(NodeContainer("F"),NodeContainer("G")),
                                     NodeContainer(NodeContainer("F"),NodeContainer("H")),
                                     NodeContainer(NodeContainer("G"),NodeContainer("H")),
                                     NodeContainer(NodeContainer("G"),NodeContainer("J")),
                                     NodeContainer(NodeContainer("H"),NodeContainer("I")) };
  vector<NetDeviceContainer> nicZastavky = {};

  CsmaHelper csma;
  // instalovanie net devices
  for (const auto& spoj : spojenia)
      nicZastavky.push_back(csma.Install(spoj));

}


void CBTC::SetWifiDevices() {
     // wi-fi channel - elektricky
     YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();
     YansWifiPhyHelper phy;
     phy.SetChannel (wChannel.Create ());

     WifiHelper wifi;
     wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

     // ad-hoc siet
     WifiMacHelper mac;
     mac.SetType ("ns3::AdhocWifiMac");
     nicElektricky = wifi.Install (phy, mac, nodyElektricky);
}


void CBTC::SetRouting() {

  InternetStackHelper internet;
  internet.Install(nodyElektricky);

  // elektricky: subnet 10.1.1.0 maska 255.255.255.0
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  interfacesElektricky = ipv4.Assign(nicElektricky);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}


void CBTC::SetApplications() {


  /****** socket:  src = elektricka [0], recv = elektricka [1], broadcast allowed ******/

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");


  srcSocket = Socket::CreateSocket (nodyElektricky.Get (1), tid);

  srcSocket->SetAllowBroadcast (true);
  srcSocket->Connect (remote);

  recSocket = Socket::CreateSocket (nodyElektricky.Get (0), tid);

  recSocket->Bind (local);
  recSocket->SetRecvCallback (MakeCallback (&ReceivePacket));


  // source posle,receiver dostane
  Simulator::ScheduleWithContext (srcSocket->GetNode()->GetId(), Seconds (1.0), &GenerateTraffic, srcSocket, velkostUdajov, 200, Seconds(0.5));


}

/********************************** Callbacky *************************************/

void CBTC::CourseChange (map<int, int> tramPositions, map<int, stack<int>> waypoints, string context /*kontext*/, Ptr<const MobilityModel> model /*mobility model*/) {

  Vector pos = model->GetPosition();
  Ptr<Node> elektricka = GetNodeFromContext (context);
  Ptr<WaypointMobilityModel> waypointModel = DynamicCast<WaypointMobilityModel> (elektricka->GetObject<MobilityModel> ());

  int id_elektricky = elektricka->GetId () - 10; // elektricky maju v mobility modeli idcka 10,11,12 preto id-10
  waypoints[id_elektricky + 10].pop ();

  if (waypoints[id_elektricky + 10].size () == 1) {

      int tramPositionIndex = tramPositions.at (id_elektricky + 10) + 1;
      int nexStop;
      cout << id_elektricky << endl;

      if (tramPositionIndex == cesta.size () - 1) {
        tramPositions.find (id_elektricky + 10)->second = 0;
        nexStop = 1;
      } else {
        tramPositions.find (id_elektricky + 10)->second = tramPositionIndex;
        nexStop = tramPositionIndex + 1;
      }
      auto previousWait = Seconds(2);
      aktualnyCas = Simulator::Now () + Seconds (interval) +  previousWait; // posunie sa cas, ten previousWait by mal byt z predchadzajuces zastavky

      cout << "current position " << waypointModel->GetPosition () << endl;
      cout << "next position " << pozicieZastavok[cesta[nexStop]] << endl;

      auto pozicia = pozicieZastavok[cesta[nexStop]];
      waypointModel->AddWaypoint (Waypoint (aktualnyCas, pozicia));
      waypoints[id_elektricky + 10].push (cesta[nexStop]);

      waypointModel->AddWaypoint (Waypoint (aktualnyCas + previousWait, pozicia));
      waypoints[id_elektricky + 10].push (cesta[nexStop]);


    }

  //ked presiel vsetkymi waypointmi, pridaju sa mu znova
  /*if (waypoints.empty()) {

      cout << "rest" << endl;

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


void ReceivePacket (Ptr<Socket> socket) {

  while (Ptr<Packet> packet = socket->Recv()){
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


void CBTC::GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval) {
  Ptr<Ipv4> ipv4 = socket -> GetNode() -> GetObject<Ipv4> ();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);

  ostringstream msg; msg << "Hello World from " << iaddr.GetLocal() << '\0';

  cout << socket->GetNode()->GetId() << endl;

  if (pktCount > 0) {
      uint16_t packetSize = msg.str().length()+1;
      Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), packetSize);
      socket->Send (packet);
//      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
  } else {
      socket->Close ();
  }
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

  CBTC cbtcExperiment;
  cbtcExperiment.SetCommandLineArgs(argc, argv);
  cbtcExperiment.Run();

  return 0;
}


