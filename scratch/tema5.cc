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
NS_LOG_COMPONENT_DEFINE ("Zadanie - ");

/**
 * 5. Simulácia dopravy – CBTC (električka, vlak …)
CBTC systémy sú systémy na zefektívnenie dopravy. Dopravné prostriedky sa pohybujú za sebou (bez predbiehania) približujú/vzďaľujú sa od sebe prípadne sa môžu prekrývať. Dôležité je zadefinovať si trasy a zástavky.

Príklady správ:
rovnaká linka: Podľa poslatia správy (napr. PING) zistia že je nietko v blízkosti a spomalia/zabrzdia.
rôzna linka: Synchronizácia pri príjazdoch na zástavku/odchod
…
Efektivita môže byť definovaná ako skrátenie času presunu, alebo minimalizovanie státia, či minimalizovanie zrýchľovania/spomaľovania pohybu.
 */
int main(int argc, char *argv[]) {
    uint32_t pocet_zastavok = 3;
    uint32_t pocet_elektriciek = 3;
    uint32_t velkost_udajov = 1024;
    uint32_t vstupny_parameter = 0;

// L1
    NodeContainer nody_elektriciek, nody_zastavky, nP;
    nody_elektriciek.Create(pocet_elektriciek);
    nody_zastavky.Create(pocet_zastavok);
    Names::Add("Zastavky", nody_zastavky.Get(pocet_zastavok));
    np.Create(1);
    Names::Add ("R1", nP.Get(0)); // TODO ?

    MobilityHelper mobility;
    // https://coe.northeastern.edu/Research/krclab/crens3-doc/classns3_1_1_list_position_allocator.html
    Ptr <ListPositionAllocator> pozicie_elektriciek = CreateObject<ListPositionAllocator>();
    // TODO: definovat pociatocne X,Y,Z suradnice vsetkych elektriciek
    // https://dpba.blob.core.windows.net/media/Default/Obr%C3%A1zky/elektricky_201912.png
    pozicie_elektriciek->Add(Vector(90.0, 358.0, 0.0));
    mobility.SetPositionAllocator(pozicie_elektriciek);


    ObjectFactory mobilityFactory;
    mobilityFactory.SetTypeId("ns3::WaypointMobilityModel");
    mobilityFactory.Set("LazyNotify", BooleanValue(true)); // TODO
    /** TODO Pre kazdu elektricku:
     *      * vytvorit waypointy = zastavky
     *      * vytvorit model a pridat mu vytvorene waypointy
     *      * nastavit mobility helperu dany model a nainstalovat jednu elektricku
     */
    // TODO vytvorit waypointy = zastavky
    Waypoint linka_9_0(Seconds(0.0), Vector(90.0, 358.0, 0.0));
    Waypoint linka_9_1(Seconds(1.0), Vector(90.0, 386.0, 0.0));
    Waypoint linka_9_2(Seconds(2.0), Vector(90.0, 415.0, 0.0));
    // TODO vytvorit model a pridat mu vytvorene waypointy
    Ptr <MobilityModel> model = mobilityFactory.Create()->GetObject<MobilityModel>();
    model->AddWaypoint(linka_9_0);
    model->AddWaypoint(linka_9_1);
    model->AddWaypoint(linka_9_2);
    // TODO nastavit mobility helperu dany model a nainstalovat jednu elektricku
    mobility.SetMobilityModel(model);
    mobility.Install(nody_elektriciek.Get(0)); // Get(1), Get(2) ... Get(pocet_elektriciek-1)

    Ptr <ListPositionAllocator> pozicie_zastavok = CreateObject<ListPositionAllocator>();
    // TODO: definovat pociatocne X,Y,Z suradnice vsetkych zastavok
    // https://dpba.blob.core.windows.net/media/Default/Obr%C3%A1zky/elektricky_201912.png
    pozicie_elektriciek->Add(Vector(90.0, 358.0, 0.0)); // Kútiky
    mobility.SetPositionAllocator(pozicie_zastavok);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nody_zastavky);
    mobility.Install (nP.Get(1));


// L2 -- nastavenia NIC a kanalov
    // TODO: ake parametre sa maju pouzit???
    CsmaHelper csma;
    NetDeviceContainer nic_zastavky;
    csma.SetChannelAttribute("DataRate", StringValue("1000Mbps")); // vstupny param?
    csma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(5))); // vstupny param?
    nic_zastavky = csma.Install(nody_zastavky);

    YansWifiChannelHelper wChannel;
    YansWifiPhyHelper phy;
    phy.SetChannel(wChannel.Create());
    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");  // TODO
    WifiMacHelper mac;
    Ssid ssid = Ssid("eduroam");  // TODO
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));  // TODO
    NetDeviceContainer nic_elektricky;
    nic_elektricky = wifi.Install(phy, mac, nody_elektriciek);
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid)); // TODO
    NetDeviceContainer nicWap;
    nicWap = wifi.Install(phy, mac, nP.Get(0));  // TODO np?

// L3 -- IP
    InternetStackHelper stack;
    stack.InstallAll();
    Ipv4AddressHelper address;
    // vytvorenie subnetu/podsiete s IP 10.2.1.0 a maskou 255.255.255.0
    // TODO: vytvorit jednu siet pre zastavky a jednu pre elektricky?
    // ZASTAVKY:
    address.SetBase("10.2.1.0", "255.255.255.0");
    Ipv4InterfaceContainer networkSContainer = address.Assign(nic_zastavky);
    // ELEKTRICKY:
    address.SetBase("10.2.2.0", "255.255.255.0");
    Ipv4InterfaceContainer networkWContainer = address.Assign(nic_elektricky);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();


//L4 TODO

//L5 TODO

//L6 TODO

//L7
    // TODO asi radsej tcp - TCP lebo zarucuje ze sa spravy dorucia v poriadku
    UdpEchoServerHelper echoServer(9);

    ApplicationContainer serverApps = echoServer.Install("PCs");
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    // TODO ???
    Ptr <Node> src = nody_elektriciek.Get(0);// TODO
    Ptr <Node> dst = nody_zastavky.Get(pocet_zastavok); // TODO
    UdpEchoClientHelper echoClient(nic_zastavky.GetAddress(pocet_zastavok), 9);// TODO
    echoClient.SetAttribute("MaxPackets", UintegerValue(5)); // vstupny param?
    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(1000)));// vstupny param?
    echoClient.SetAttribute("PacketSize", UintegerValue(velkost_udajov));


    ApplicationContainer clientApps = echoClient.Install(src);
    clientApps.Start(Seconds(0.0));
    clientApps.Stop(Seconds(10.0));
    Simulator::Stop(Seconds(10.0));

    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    csma.EnablePcap("ttt", nic_zastavky, true);
    wifi.EnablePcap("sss", nic_elektricky, true);

    AnimationInterface aml("a.xml");
    aml.EnablePacketMetadata();
    aml.SetConstantPosition(src, 0, 0);
    aml.UpdateNodeDescription(src, "src");
    aml.UpdateNodeDescription(dst, "dst");

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
