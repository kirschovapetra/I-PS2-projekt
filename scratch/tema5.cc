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
 */

#include <bits/stdint-uintn.h>
#include <ns3/assert.h>
#include <ns3/boolean.h>
#include <ns3/callback.h>
#include <ns3/command-line.h>
#include <ns3/config.h>
#include <ns3/csma-helper.h>
#include <ns3/data-rate.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/log.h>
#include <ns3/mobility-helper.h>
#include <ns3/names.h>
#include <ns3/net-device-container.h>
#include <ns3/node.h>
#include <ns3/node-container.h>
#include <ns3/node-list.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/position-allocator.h>
#include <ns3/ptr.h>
#include <ns3/simulator.h>
#include <ns3/ssid.h>
#include <ns3/vector.h>
#include <ns3/waypoint.h>
#include <ns3/waypoint-mobility-model.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/yans-wifi-helper.h>
#include <stddef.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Tema 5: Simulacia dopravy – CBTC");

/* -------------------------------------------------------------------------------------------------------- */

// CMD line argumenty
uint32_t pocetZastavok = 10;
uint32_t pocetElektriciek = 3;
uint32_t velkostUdajov = 1024;
double trvanieSimulacie = 200.0;
bool ulozAnimaciu = false;  // TODO na toto sa daju nejake ify tam, kde bubeme vytvarat to xmlko s animaciou

// timing pohybu elektriciek
double delay = 3.0;
double interval = 5.0;
auto aktualnyCas = Seconds(0.0);


/* cesta: A (0) -> B (1) -> C (2) -> D (3) -> H (7) -> I (8) -> H (7) -> G (6) -> J (9) ->
          G (6) -> F (5) -> E (4) -> F (5) -> C (2) -> B (1) -> A (0)
*/
const vector<int> cesta = {0,1,2,3,7,8,7,6,9,6,5,4,5,2,1,0};

const vector<string> nazvyZastavok = {"A","B", "C", "D", "E", "F", "G", "H", "I", "J"};
const vector<Vector> pozicieZastavok = { Vector(0.0, 0.0, 0.0),        //A
                                          Vector(0.0, 200.0, 0.0),      //B

                                          Vector(200.0, 100.0, 0.0),    //C
                                          Vector(200.0, 200.0, 0.0),    //D

                                          Vector(300.0, 0.0, 0.0),      //E
                                          Vector(300.0, 100.0, 0.0),    //F

                                          Vector(400.0, 100.0, 0.0),    //G
                                          Vector(400.0, 200.0, 0.0),    //H
                                          Vector(400.0, 300.0, 0.0),    //I

                                          Vector(500.0, 100.0, 0.0) };  //J

/* -------------------------------------------------------------------------------------------------------- */

void zastavkyConstantPositionModel(MobilityHelper& mobility, NodeContainer& nodyZastavky) {

  Ptr <ListPositionAllocator> pozicieZastavokAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: pozicieZastavok) {
      pozicieZastavokAlloc -> Add(pozicia);
  }

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(pozicieZastavokAlloc);
  mobility.Install(nodyZastavky);

}


vector<NetDeviceContainer> zastavkyCsmaSpojenia(CsmaHelper& csma) {

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

  vector<NetDeviceContainer> nic_zastavky = {};
//   instalovanie net devices
  for (const auto& spoj : spojenia) {
      nic_zastavky.push_back(csma.Install(spoj));
  }

  return nic_zastavky;
}


void elektrickyWaypointModel(MobilityHelper& mobility, NodeContainer& nodyElektricky){

  /** Pre kazdu elektricku:
   *      * vytvorit waypointy = zastavky
   *      * vytvorit model a pridat mu vytvorene waypointy
   *      * nastavit mobility helperu dany model a nainstalovat jednu elektricku
   */

  mobility.SetMobilityModel("ns3::WaypointMobilityModel", "LazyNotify",BooleanValue(true));
  mobility.Install(nodyElektricky);

  // nastavenie pozicii do mobility modelu
  Ptr <ListPositionAllocator> pozicieElektriciekAlloc = CreateObject<ListPositionAllocator>();
  for( int i = 0; i < pocetElektriciek; i++) {
      pozicieElektriciekAlloc -> Add(Vector(0.0, 0.0, 0.0));
  }
  mobility.SetPositionAllocator(pozicieElektriciekAlloc);

  /* waypoint model pre vsetky elektricky
       * vsetky maju rovnaku cestu, iba maju posunute intervaly
       * TODO zatial sa len loopuje ich pohyb, ale treba aby si posielali spravy a podla toho spomalovali/zrychlovali
  */

  for (int i = 0; i < nodyElektricky.GetN(); i++) {


      cout << "-- ELEKTRICKA" << i << " --" << endl;

      Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel>( nodyElektricky.Get(i)->GetObject<MobilityModel>() );

      for (int j = 0; j < cesta.size(); j++) {
          auto cas = aktualnyCas + Seconds(interval * j + i * delay);
          auto pozicia = pozicieZastavok[cesta[j]];

          cout << "zastavka: " << cesta[j] << " cas: " << cas  << endl;

          model->AddWaypoint (Waypoint(cas,pozicia));
      }
  }
}


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


static void CourseChange (std::string context, Ptr<const MobilityModel> model) {


  //  cout << elektricka->GetId() << ": " << waypointMobModel->WaypointsLeft() << endl;
  //  cout << Simulator::Now ()<< ", x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;


  Vector pos = model->GetPosition ();
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

    }
}

<<<<<<< Updated upstream
=======


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

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval ) {
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


void vytvorSocketyMedziElektrickami(NodeContainer nody_elektricky, Ptr<Socket> sockety[]){

  int n = nody_elektricky.GetN()-1;

  const char *socketAdresses[2];
  socketAdresses[0] = "255.255.255.255";
  socketAdresses[1] = "255.255.255.254";

  for (int i=0; i<n; i++){
      uint16_t port = 80 - i;
      //cout << "server: " << i << " klient: " << i+1 << " port: " << port << " adress: "
      //string adress = "255.255.255.255";
      //uint32_t socketAdress = "255.255.255.255";// (uint32_t) atoi ("255.255.255.255"); //+ std::to_string(255); // p
      //cout << socketAdress;

      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      Ptr<Socket> recvSink = Socket::CreateSocket (nody_elektricky.Get (i+1), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
      recvSink->Bind (local);
      recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

      Ptr<Socket> source = Socket::CreateSocket (nody_elektricky.Get (i), tid);
      InetSocketAddress remote = InetSocketAddress (Ipv4Address (socketAdresses[i]), port);
      source->SetAllowBroadcast (true);
      source->Connect (remote);

      sockety[i] = source;
  }


}

void pingniZoSource(Ptr<Socket> source) {
  if(source == NULL){
      NS_LOG_ERROR("Source is null");
      return;
  }

  cout << "dadsa" << source->GetNode()->GetId();

  Simulator::ScheduleWithContext (source->GetNode()->GetId(),
                                  Seconds (1.0),
                                  &GenerateTraffic,
                                  source,
                                  velkostUdajov,
                                  200,
                                  Seconds(0.5));
}


>>>>>>> Stashed changes
int main(int argc, char *argv[]) {

   /* Vypisu sa cmd line argumenty: ./waf --run "tema5 --PrintHelp" */

   CommandLine cmd;
   cmd.AddValue ("saveAnimation", "Ulozenie animacie", ulozAnimaciu);
   cmd.AddValue("velkostUdajov", "Velkost udajov.", velkostUdajov);
   cmd.AddValue("trvanieSimulacie", "Trvanie simulacie.", trvanieSimulacie);
   cmd.AddValue("pocetElektriciek", "Pocet elektriciek.", pocetElektriciek);
   // dalsie parametre ...
   cmd.Parse(argc, argv);

// L1

   /*************** Zastavky ***************/



   NodeContainer nody_zastavky;
   nody_zastavky.Create(pocetZastavok);
   for (int i = 0; i < pocetZastavok; i++) {
       Names::Add (nazvyZastavok[i], nody_zastavky.Get(i)); // nazvy zastavok
   }

   // rozmiestnenie zastavok
   MobilityHelper mobility;
   zastavkyConstantPositionModel(mobility,nody_zastavky);

   /*************** Elektricky ***************/

   NodeContainer nody_elektricky;
   nody_elektricky.Create(pocetElektriciek);
   for (int i = 0; i < pocetElektriciek; i++) {
       Names::Add (to_string(i+1), nody_elektricky.Get(i)); // nazvy elektriciek
   }

   // rozmiestnenie elektriciek
   elektrickyWaypointModel(mobility,nody_elektricky);


 // L2 -- nastavenia NIC a kanalov

     // CSMA spojenia medzi zastavkami
     CsmaHelper csma;
     csma.SetChannelAttribute ("DataRate", DataRateValue (5000000));
     csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2)));
     vector<NetDeviceContainer> nic_zastavky_all = zastavkyCsmaSpojenia(csma);

<<<<<<< Updated upstream
     // wi-fi channel - elektricky
     YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();
     YansWifiPhyHelper phy;
     phy.SetChannel (wChannel.Create ());
     WifiHelper wifiHelper;
     wifiHelper.SetRemoteStationManager ("ns3::AarfWifiManager");
     Ssid ssid = Ssid ("imhd");

     // mac - elektricky
     WifiMacHelper mac;
     mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));
     NetDeviceContainer nic_elektricky;
     nic_elektricky = wifiHelper.Install (phy, mac, nody_elektricky);
     mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
     NetDeviceContainer nicWap;
 //    nicWap = wifiHelper.Install (phy, mac, accessPoint); //budeme mat nejaky access point?


 // L3 -- IP
     InternetStackHelper stack;
     stack.InstallAll();
     Ipv4AddressHelper address;

     // ZASTAVKY:
     address.SetBase("10.2.1.0", "255.255.255.0"); // vytvorenie subnetu/podsiete s IP 10.2.1.0 a maskou 255.255.255.0
     for (auto& nic_zastavky: nic_zastavky_all){
         Ipv4InterfaceContainer zastavky_networkContainer = address.Assign(nic_zastavky);
     }

 //    // ELEKTRICKY:
     address.SetBase("10.2.2.0", "255.255.255.0"); // subnet 10.2.2.0 maska 255.255.255.0
     Ipv4InterfaceContainer elektricky_networkContainer = address.Assign(nic_elektricky);

 //    Ipv4GlobalRoutingHelper::PopulateRoutingTables (); // toto hadze error

=======
      // wi-fi channel - elektricky
      YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();
      YansWifiPhyHelper phy;
      phy.SetChannel (wChannel.Create ());

      WifiHelper wifi;
      wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

      // ad-hoc siet
      WifiMacHelper mac;
      mac.SetType ("ns3::AdhocWifiMac");
      NetDeviceContainer nic_elektricky = wifi.Install (phy, mac, nody_elektricky);


  // L3, L4 TCP/IP

      InternetStackHelper internet;
      internet.Install(nody_elektricky);

      // elektricky: subnet 10.1.1.0 maska 255.255.255.0
      Ipv4AddressHelper ipv4;
      ipv4.SetBase("10.1.1.0", "255.255.255.0");
      auto interfaces_elektricky = ipv4.Assign(nic_elektricky);


      //L5 - L7 aplikacna vrstva
      Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

      Ptr<Socket> sockety[2];

      vytvorSocketyMedziElektrickami(nody_elektricky, sockety);

     // cout << "prvy " << sockety[0] << "druhy " << sockety[1];

      // ping z prvej elektricky
      cout << "ping z prvej elektricky";
      pingniZoSource(sockety[0]);

      // ping z druhej elektricky
      cout << "ping z druhej elektricky";
      pingniZoSource(sockety[1]);



      //Simulator::ScheduleWithContext (sockety[1]->GetNode()->GetId(), Seconds (1.0), &GenerateTraffic, sockety[1], velkostUdajov, 200, Seconds(0.5));

      /****** socket:  src = elektricka [0], recv = elektricka [1], broadcast allowed ******/

     /*  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
       Ptr<Socket> recvSink = Socket::CreateSocket (nody_elektricky.Get (1), tid);
       InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
       recvSink->Bind (local);
       recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

       Ptr<Socket> source = Socket::CreateSocket (nody_elektricky.Get (0), tid);
       InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
       source->SetAllowBroadcast (true);
       source->Connect (remote);

       Simulator::ScheduleWithContext (source->GetNode()->GetId(), Seconds (1.0), &GenerateTraffic, source, velkostUdajov, 200, Seconds(0.5));
>>>>>>> Stashed changes

*/


*/


 //L4 - L7 aplikacna vrstva


     // TODO asi radsej tcp - TCP lebo zarucuje ze sa spravy dorucia v poriadku
 //    UdpEchoServerHelper echoServer(9);
 //
 //    ApplicationContainer serverApps = echoServer.Install("PCs");
 //    serverApps.Start(Seconds(1.0));
 //    serverApps.Stop(Seconds(10.0));
 //
 //    // TODO ???
 //    Ptr <Node> src = nody_elektriciek.Get(0);// TODO
 //    Ptr <Node> dst = nody_zastavky.Get(pocetZastavok); // TODO
 //    UdpEchoClientHelper echoClient(nic_zastavky.GetAddress(pocetZastavok), 9);// TODO
 //    echoClient.SetAttribute("MaxPackets", UintegerValue(5)); // vstupny param?
 //    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(1000)));// vstupny param?
 //    echoClient.SetAttribute("PacketSize", UintegerValue(velkostUdajov));
 //
 //
 //    ApplicationContainer clientApps = echoClient.Install(src);
 //    clientApps.Start(Seconds(0.0));
 //    clientApps.Stop(Seconds(10.0));
 //    Simulator::Stop(Seconds(10.0));
 //
 //    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
 //    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
 //    csma.EnablePcap("ttt", nic_zastavky, true);
 //    wifi.EnablePcap("sss", nic_elektricky, true);
 //
 //    AnimationInterface aml("a.xml");
 //    aml.EnablePacketMetadata();
 //    aml.SetConstantPosition(src, 0, 0);
 //    aml.UpdateNodeDescription(src, "src");
 //    aml.UpdateNodeDescription(dst, "dst");


     Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChange));

     Simulator::Stop (Seconds (trvanieSimulacie));
     Simulator::Run();
     Simulator::Destroy();

     return 0;
 }
