#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("Tema 5: Simulacia dopravy – CBTC");

/*

5. Simulácia dopravy – CBTC (električka, vlak …)

CBTC systémy sú systémy na zefektívnenie dopravy. Dopravné prostriedky sa pohybujú za sebou (bez predbiehania) približujú/vzďaľujú sa od sebe prípadne sa môžu prekrývať. Dôležité je zadefinovať si trasy a zástavky.

Príklady správ:
rovnaká linka: Podľa poslatia správy (napr. PING) zistia že je nietko v blízkosti a spomalia/zabrzdia.
rôzna linka: Synchronizácia pri príjazdoch na zástavku/odchod

Efektivita môže byť definovaná ako skrátenie času presunu, alebo minimalizovanie státia, či minimalizovanie zrýchľovania/spomaľovania pohybu.


List position allocator: https://coe.northeastern.edu/Research/krclab/crens3-doc/classns3_1_1_list_position_allocator.html
Mapa elektriciek: https://dpba.blob.core.windows.net/media/Default/Obr%C3%A1zky/elektricky_201912.png

 */

vector<Vector> zastavkyConstPosModel(MobilityHelper& mobility, NodeContainer& nody_zastavky) {

  vector<Vector> pozicieZastavokVecList = { Vector(0.0, 0.0, 0.0),        //A
                                            Vector(0.0, 200.0, 0.0),      //B

                                            Vector(200.0, 100.0, 0.0),    //C
                                            Vector(200.0, 200.0, 0.0),    //D

                                            Vector(300.0, 0.0, 0.0),      //E
                                            Vector(300.0, 100.0, 0.0),    //F

                                            Vector(400.0, 100.0, 0.0),    //G
                                            Vector(400.0, 200.0, 0.0),    //H
                                            Vector(400.0, 300.0, 0.0),    //I

                                            Vector(500.0, 100.0, 0.0) };  //J


  Ptr <ListPositionAllocator> pozicieZastavokAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: pozicieZastavokVecList) {
      pozicieZastavokAlloc -> Add(pozicia);
  }

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(pozicieZastavokAlloc);
  mobility.Install(nody_zastavky);

  return pozicieZastavokVecList;

}


void zastavkyCsmaConnections(CsmaHelper& csma) {

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

//   instalovanie net devices
  for (const auto& spoj : spojenia) {
     NetDeviceContainer container = csma.Install(spoj);
  }
}


void elektrickyWaypointModel(MobilityHelper& mobility, NodeContainer& nody_elektricky, vector<Vector> pozicie_zastavok, double interval){

  /** Pre kazdu elektricku:
   *      * vytvorit waypointy = zastavky
   *      * vytvorit model a pridat mu vytvorene waypointy
   *      * nastavit mobility helperu dany model a nainstalovat jednu elektricku
   */


  // init pozicie elektriciek

  Vector pozicieElektriciekVecList[] = { Vector(0.0, 0.0, 0.0),       // 1
                                         Vector(300.0, 0.0, 0.0),     // 2
                                         Vector(400.0, 300.0, 0.0) }; // 3

  // nastavenie pozicii do mobility modelu
  Ptr <ListPositionAllocator> pozicieElektriciekAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: pozicieElektriciekVecList) {
      pozicieElektriciekAlloc -> Add(pozicia);
  }

  // waypoint model
  ObjectFactory mobilityFactory;
  mobilityFactory.SetTypeId ("ns3::WaypointMobilityModel");
  mobilityFactory.Set ("LazyNotify", BooleanValue (true));

  Ptr<MobilityModel> model = mobilityFactory.Create()->GetObject<MobilityModel>();
  Ptr<WaypointMobilityModel> mob = model->GetObject<WaypointMobilityModel>();


  /* elektricka 1: A (0) -> B (1) -> C (2) -> D (3) -> H (7)-> I (8)*/

  vector<int> elektricka1_cesta{0,1,2,3,7,8};

  for (auto i : elektricka1_cesta) {
      mob->AddWaypoint (Waypoint(Seconds(interval*(i+1)), pozicie_zastavok[i]));
  }
//  mob->SetPosition(pozicie_zastavok[0]);
  nody_elektricky.Get(0)->AggregateObject(mob);
  mobility.Install(nody_elektricky.Get(0));


  // TODO viacero elektriciek mi nechce ist :((
  // + casom spomaluje z nejakeho dovodu

  /* elektricka 2: E (4) -> F (5) -> G (6) -> J (9) */

//  vector<int> elektricka2_cesta{4,5,6,9};
//
//  for (auto i : elektricka2_cesta) {
//          mob->AddWaypoint (Waypoint(Seconds(interval * i), pozicie_zastavok[i]));
//    }
//  mob->SetPosition(pozicie_zastavok[4]);
//  nody_elektricky.Get(1)->AggregateObject(mob);
//  mobility.Install(nody_elektricky.Get(1));
//
  /* elektricka 3: A (0)-> B (1) -> F (2)-> H (5) ->  G (6) -> J (9) */

//  vector<int> elektricka3_cesta{0,1,2,5,6,9};
//
//  for (auto i : elektricka3_cesta) {
//        mob->AddWaypoint (Waypoint(Seconds(interval * i), pozicie_zastavok[i]));
//  }
//  mob->SetPosition(pozicie_zastavok[0]);
//  nody_elektricky.Get(2)->AggregateObject(mob);
//  mobility.Install(nody_elektricky.Get(2));
}


int main(int argc, char *argv[]) {
  uint32_t pocet_zastavok = 10;
  uint32_t pocet_elektriciek = 3;
  uint32_t velkost_udajov = 1024;

  string nazvy_zastavok[] = {"A","B", "C", "D", "E", "F", "G", "H", "I", "J"};
  string nazvy_elektriciek[] = {"1", "2", "3"};

  CommandLine cmd;
  // TODO: Parameter
  cmd.AddValue("velkost_udajov", "Velkost udajov.", velkost_udajov);
  // dalsie parametre ...
  cmd.Parse(argc, argv);



  /*************** Zastavky ***************/

  // L1

  NodeContainer nody_zastavky;
  nody_zastavky.Create(pocet_zastavok);
  for (int i = 0; i < pocet_zastavok; i++) {
      Names::Add (nazvy_zastavok[i], nody_zastavky.Get(i)); // nazvy zastavok
  }

  // rozmiestnenie zastavok
  MobilityHelper mobility;
  vector<Vector> pozicie_zastavok = zastavkyConstPosModel(mobility,nody_zastavky);

  /*************** Elektricky ***************/

  NodeContainer nody_elektricky;
  nody_elektricky.Create(pocet_elektriciek);
  for (int i = 0; i < pocet_elektriciek; i++) {
      Names::Add (nazvy_elektriciek[i], nody_elektricky.Get(i)); // nazvy elektriciek
  }

  // rozmiestnenie elektriciek
    elektrickyWaypointModel(mobility,nody_elektricky, pozicie_zastavok, 10.0);


//L2

    // L2 -- nastavenia NIC a kanalov

    // CSMA spojenia medzi zastavkami
    CsmaHelper csma;
    csma.SetChannelAttribute ("DataRate", DataRateValue (5000000));
    csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2)));
    zastavkyCsmaConnections(csma);


//    // wi-fi channel
//    YansWifiChannelHelper wChannel = YansWifiChannelHelper::Default ();
//    YansWifiPhyHelper phy;
//    phy.SetChannel (wChannel.Create ());
//    WifiHelper wifiHelper;
//    wifiHelper.SetRemoteStationManager ("ns3::AarfWifiManager");
//    Ssid ssid = Ssid ("eduroam");
//
//    // mac
//    WifiMacHelper mac;
//    mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));
//    NetDeviceContainer nic_elektricky;
//    nic_elektricky = wifiHelper.Install (phy, mac, nody_elektriciek);
//    mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
//    NetDeviceContainer nicWap;
//    nicWap = wifiHelper.Install (phy, mac, accessPoint); budeme mat nejaky access point?


// L3 -- IP
    InternetStackHelper stack;
    stack.InstallAll();
    Ipv4AddressHelper address;
    // vytvorenie subnetu/podsiete s IP 10.2.1.0 a maskou 255.255.255.0
    // TODO: vytvorit jednu siet pre zastavky a jednu pre elektricky?
    // ZASTAVKY:
//    address.SetBase("10.2.1.0", "255.255.255.0");
//    Ipv4InterfaceContainer networkSContainer = address.Assign(nic_zastavky);
//    // ELEKTRICKY:
//    address.SetBase("10.2.2.0", "255.255.255.0");
//    Ipv4InterfaceContainer networkWContainer = address.Assign(nic_elektricky);
//    Ipv4GlobalRoutingHelper::PopulateRoutingTables();


//L4 TODO

//L5 TODO

//L6 TODO

//L7
    // TODO asi radsej tcp - TCP lebo zarucuje ze sa spravy dorucia v poriadku
//    UdpEchoServerHelper echoServer(9);
//
//    ApplicationContainer serverApps = echoServer.Install("PCs");
//    serverApps.Start(Seconds(1.0));
//    serverApps.Stop(Seconds(10.0));
//
//    // TODO ???
//    Ptr <Node> src = nody_elektriciek.Get(0);// TODO
//    Ptr <Node> dst = nody_zastavky.Get(pocet_zastavok); // TODO
//    UdpEchoClientHelper echoClient(nic_zastavky.GetAddress(pocet_zastavok), 9);// TODO
//    echoClient.SetAttribute("MaxPackets", UintegerValue(5)); // vstupny param?
//    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(1000)));// vstupny param?
//    echoClient.SetAttribute("PacketSize", UintegerValue(velkost_udajov));
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

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
