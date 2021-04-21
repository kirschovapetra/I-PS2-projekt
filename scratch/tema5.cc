#include <bits/stdint-uintn.h>
#include <ns3/application-container.h>
#include <ns3/boolean.h>
#include <ns3/command-line.h>
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
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/position-allocator.h>
#include <ns3/ptr.h>
#include <ns3/simulator.h>
#include <ns3/ssid.h>
#include <ns3/uinteger.h>
#include <ns3/v4ping-helper.h>
#include <ns3/vector.h>
#include <ns3/waypoint.h>
#include <ns3/waypoint-mobility-model.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/yans-wifi-helper.h>
#include <string>
#include <vector>

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


* List position allocator: https://coe.northeastern.edu/Research/krclab/crens3-doc/classns3_1_1_list_position_allocator.html
* Mapa elektriciek: https://dpba.blob.core.windows.net/media/Default/Obr%C3%A1zky/elektricky_201912.png
* Waypoint mobility model: https://groups.google.com/g/ns-3-users/c/8r07uUD78Dk


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


vector<NetDeviceContainer> zastavkyCsmaConnections(CsmaHelper& csma) {

  vector<NetDeviceContainer> nic_zastavky = {};

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
      nic_zastavky.push_back(csma.Install(spoj));
  }

  return nic_zastavky;
}


void setWaypoints(vector<Vector> pozicie_zastavok, Ptr<Node> elektricka, double interval, vector<int> cesta) {
  Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel>( elektricka->GetObject<MobilityModel>() );

  for (int i = 0; i < cesta.size(); i++) {
      model->AddWaypoint (Waypoint(Seconds(interval*i), pozicie_zastavok[cesta[i]]));
  }

}


void elektrickyWaypointModel(MobilityHelper& mobility, NodeContainer& nody_elektricky, vector<Vector> pozicie_zastavok, double interval){

  /** Pre kazdu elektricku:
   *      * vytvorit waypointy = zastavky
   *      * vytvorit model a pridat mu vytvorene waypointy
   *      * nastavit mobility helperu dany model a nainstalovat jednu elektricku
   */

  mobility.SetMobilityModel("ns3::WaypointMobilityModel", "LazyNotify",BooleanValue(true));
  mobility.Install(nody_elektricky);

  // init pozicie elektriciek

  Vector pozicieElektriciekVecList[] = { Vector(0.0, 0.0, 0.0),       // 1
                                         Vector(300.0, 0.0, 0.0),     // 2
                                         Vector(400.0, 300.0, 0.0) }; // 3

  // nastavenie pozicii do mobility modelu
  Ptr <ListPositionAllocator> pozicieElektriciekAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: pozicieElektriciekVecList) {
      pozicieElektriciekAlloc -> Add(pozicia);
  }
  mobility.SetPositionAllocator(pozicieElektriciekAlloc);

  // waypoint model

  /* elektricka 1: A (0) -> B (1) -> C (2) -> D (3) -> H (7)-> I (8)*/
  setWaypoints(pozicie_zastavok,nody_elektricky.Get(0), interval, {0,1,2,3,7,8});

  /* elektricka 2: E (4) -> F (5) -> G (6) -> J (9) */
  setWaypoints(pozicie_zastavok,nody_elektricky.Get(1), interval, {4,5,6,9});

  /* elektricka 3: J (9) -> G (6) -> H (5) -> F (2) -> B (1) -> A (0) */
  setWaypoints(pozicie_zastavok,nody_elektricky.Get(2), interval, {9,6,5,2,1,0});

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
    elektrickyWaypointModel(mobility,nody_elektricky, pozicie_zastavok, 5.0);


// L2 -- nastavenia NIC a kanalov

    // CSMA spojenia medzi zastavkami
    CsmaHelper csma;
    csma.SetChannelAttribute ("DataRate", DataRateValue (5000000));
    csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2)));
    vector<NetDeviceContainer> nic_zastavky_all = zastavkyCsmaConnections(csma);


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