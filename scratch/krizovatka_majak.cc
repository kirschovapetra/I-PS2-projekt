/* 
 DOPRAVNA KRIZOVATKA
 
 V tomto zadani bola vypracovana simulacia dopravnej krizovatky.
 Ako MAC protokol bol pouzity ConstantRateWifiManager. Vybrali sme ho kvoli tomu, ze pouziva konstantne prenosove rychlosti 
    pre kazdy odoslany paket.
 Bol zvoleny transportny protokol TcpL4Protocol - je to najpouzivanejsi protokol v transportnej vrstve a je v prevadzke
    uz viac ako 30 rokov. TcpL4Protocol je taktiez stabilny a je k dispozicii na kazdom modernom operacnom systeme.
 V tomto zadani bol pouzity smerovaci protokol Dsdv, ktory je zalozeny na Bellman Fordovom algoritme. Tento protokol ma osobitne 
    tabulky pre kazdy node, a jeho vyhodou je, ze ma efektivne vysledky na malych sietach. Tomuto smerovaciemu protokolu bol 
    v tomto zadani meneny atribut PeriodicUpdateInterval.
 V tomto zadani sa pomerne casto vyuzivaju casove udalosti (Simulator::Schedule), najma na prepinanie semafora. 
 L1 bol nastaveny na ConstantPositionMobilityModel, pretoze chceme aby auta chodili pravidelne (preto nie RandomWalk2dMobilityModel).
 Zmena v L2-L5 bola v MatrixPropagationLossModeli, kde sa menil parameter defaultLoss (na hodnotu 400). 
 Program podporuje aj vstupne argumenty pri spustani z command linu :
    --simulTime - na ovplyvnenie casu trvania simulacie (double)
    --anim - na vytvorenie, alebo nevytvorenie animacie (boolean)
 
 GRAFY:
 Pri vypracovani grafov som v tomto zadani menil defaultLoss MatrixPropagationLossModelu a interval zmeny semaforu (ako casto sa ma 
    zmenit hodnota semaforov - bolo to menene pomocou premennej change_sem_time) 
 Graf s nazvom 
    - graph_loss400_4sInterval.svg - V tomto grafe bol nastaveny default loss na 400, a semafor menil svoju hodnotu kazde 4 sekundy.
    - graph_loss400_2sInterval.svg - V tomto grafe bol nastaveny default loss na 400, a semafor menil svoju hodnotu kazde 2 sekundy.
    - graph_loss100_4sInterval.svg - V tomto grafe bol nastaveny default loss na 100, a semafor menil svoju hodnotu kazde 4 sekundy.
    - graph_loss100_2sInterval.svg - V tomto grafe bol nastaveny default loss na 100, a semafor menil svoju hodnotu kazde 2 sekundy.
 V grafoch je zobrazena zavislost priepustnosti (v Mbps) od poctov merania. Z grafov je zrejme, ze vyssiu priepustnost (Throughput)
    dosiahneme zmensenim parametra defaultLoss. Taktiez je zrejme, ze cim menej casto sa meni hodnota semafora, tym je priepustnost
    vacsia.
 
 VYPRACOVAL : MARTIN MAJAK (80017)
 */

#include <bits/stdint-uintn.h>
#include <ns3/animation-interface.h>
#include <ns3/application-container.h>
#include <ns3/command-line.h>
#include <ns3/config.h>
#include <ns3/double.h>
#include <ns3/dsdv-helper.h>
#include <ns3/event-id.h>
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
#include <ns3/mobility-helper.h>
#include <ns3/mobility-model.h>
#include <ns3/net-device-container.h>
#include <ns3/node.h>
#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/on-off-helper.h>
#include <ns3/position-allocator.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/ptr.h>
#include <ns3/rng-seed-manager.h>
#include <ns3/simulator.h>
#include <ns3/string.h>
#include <ns3/udp-echo-helper.h>
#include <ns3/uinteger.h>
#include <ns3/vector.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/wifi-standards.h>
#include <ns3/yans-wifi-channel.h>
#include <ns3/yans-wifi-helper.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

using namespace ns3;
using namespace std;

static Vector apPosition = Vector(20.0, 20.0, 0.0);
double fi = 0.0;
static double deg = 2 * M_PI / (double) 990;
Gnuplot2dDataset data;

uint32_t nDrivers = 0;
uint32_t nStandmen = 1;
uint32_t nGroup1 = 3;
uint32_t nGroup2 = 3;
uint32_t nGroup3 = 3;
uint32_t nGroup4 = 3;
uint32_t counter = 0;

int countr = 1;

bool semafor = true;
bool isSafe = true;
bool shouldStop = false;
int change_sem_time = 2;
int numOfMeasurements = 5;

static void zmenSemafor() {
    Simulator::Schedule(Seconds(change_sem_time), &zmenSemafor);

    if (isSafe) {
        shouldStop = false;
        if (semafor) {
            semafor = false;
        } else {
            semafor = true;
        }
    } else {
        shouldStop = true;
        Simulator::Schedule(Seconds(0.01), &zmenSemafor);
    }

}

static void SetPosition(Ptr<Node> node, Vector position) {
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
    mobility->SetPosition(position);
}

static Vector GetRandom3DPoint(int min, int max) {
    return Vector(min + rand() % (max - min + 1), min + rand() % (max - min + 1), 0.0);
}

static Vector GetPosition(Ptr<Node> node) {
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
    return mobility->GetPosition();
}

static void MoveUp(NodeContainer nodes);

static void MoveDown(NodeContainer nodes) {

    if (semafor == false) {
        isSafe = false;
        bool moveDown = true;
        uint32_t count = 0;
        for (uint32_t i = 0; i < nodes.GetN(); i++) {
            Ptr<Node> node = nodes.Get(i);
            Vector pos = GetPosition(node);
            if (!shouldStop || (shouldStop && pos.y > 20)) {

                pos.y += 5.0;

                if (pos.y >= 45) {
                    pos.y = 5;
                }
            } else
                count += 1;

            SetPosition(node, pos);
        }
        if (count == 3)
            isSafe = true;

        if (moveDown)
            Simulator::Schedule(Seconds(0.01), &MoveDown, nodes);
        else
            Simulator::Schedule(Seconds(0.01), &MoveUp, nodes);

    } else
        Simulator::Schedule(Seconds(0.01), &MoveDown, nodes);
}

static void MoveUp(NodeContainer nodes) {
    if (semafor == false) {
        isSafe = false;
        bool moveUp = true;
        uint32_t count = 0;
        for (uint32_t i = 0; i < nodes.GetN(); i++) {
            Ptr<Node> node = nodes.Get(i);

            Vector pos = GetPosition(node);
            if (!shouldStop || (shouldStop && pos.y < 30)) {

                pos.y -= 5.0;

                if (pos.y <= 5) {
                    pos.y = 45;
                }
            } else
                count += 1;

            SetPosition(node, pos);
        }
        if (count == 3)
            isSafe = true;

        if (moveUp)
            Simulator::Schedule(Seconds(0.01), &MoveUp, nodes);
        else
            Simulator::Schedule(Seconds(0.01), &MoveDown, nodes);
    } else
        Simulator::Schedule(Seconds(0.01), &MoveUp, nodes);
}

static void MoveRight(NodeContainer nodes);

static void MoveLeft(NodeContainer nodes) {
    if (semafor) {
        isSafe = false;
        bool moveLeft = true;
        uint32_t count = 0;
        for (uint32_t i = 0; i < nodes.GetN(); i++) {
            Ptr<Node> node = nodes.Get(i);
            Vector pos = GetPosition(node);
            if (!shouldStop || (shouldStop && pos.x < 30)) {
                pos.x -= 5.0;

                if (pos.x <= 5) {
                    pos.x = 45;
                }
            } else
                count += 1;

            SetPosition(node, pos);
        }

        if (count == 3)
            isSafe = true;

        if (moveLeft)
            Simulator::Schedule(Seconds(0.01), &MoveLeft, nodes);
        else
            Simulator::Schedule(Seconds(0.01), &MoveRight, nodes);
    } else
        Simulator::Schedule(Seconds(0.01), &MoveLeft, nodes);
}

static void MoveRight(NodeContainer nodes) {
    if (semafor) {
        isSafe = false;
        bool moveRight = true;
        uint32_t count = 0;
        for (uint32_t i = 0; i < nodes.GetN(); i++) {
            Ptr<Node> node = nodes.Get(i);
            Vector pos = GetPosition(node);
            if (!shouldStop || (shouldStop && pos.x > 20)) {
                pos.x += 5.0;

                if (pos.x >= 45) {
                    pos.x = 5;
                }

            } else
                count += 1;


            SetPosition(node, pos);
        }

        if (count == 3)
            isSafe = true;

        if (moveRight)
            Simulator::Schedule(Seconds(0.01), &MoveRight, nodes);
        else
            Simulator::Schedule(Seconds(0.01), &MoveLeft, nodes);
    } else
        Simulator::Schedule(Seconds(0.01), &MoveRight, nodes);
}

static void Run(Ptr<Node> node) {

    Vector pos = GetPosition(node);
    double r = sqrt((apPosition.x - pos.x)*(apPosition.x - pos.x) + (apPosition.y - pos.y)*(apPosition.x - pos.y));
    counter++;

    if (counter == nDrivers) {
        fi -= deg;
        counter = 0;
    }

    pos.x = apPosition.x + r * cos(fi);
    pos.y = apPosition.y + r * sin(fi);

    Simulator::Schedule(Seconds(0.01), &Run, node);
}

int main(int argc, char *argv[]) {
    srand(time(NULL));
    SeedManager::SetRun(54);

    bool enableCtsRts = true;
    double simulTime = 10.0;
    bool animation_enabled = true;

    CommandLine cmd;
    cmd.AddValue("simulTime", "Time of simulation", simulTime);
    cmd.AddValue("anim", "Enable or disable animation", animation_enabled);

    cmd.Parse(argc, argv);

    uint32_t nWifi = 1 + nGroup4 + nGroup3 + nGroup1 + nGroup2 + nStandmen + nDrivers;

    double dataTable[nWifi - 1][numOfMeasurements];

    for (int k = 0; k < numOfMeasurements; k++) {
        cout << "Meranie " << k + 1 << endl;

        //Enable or disable CTS/RTS
        UintegerValue ctsThr = (enableCtsRts ? UintegerValue(100) : UintegerValue(2200));
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
        MobilityHelper mobility;
        NodeContainer stas;
        NetDeviceContainer staDevs;
        stas.Create(nWifi);

        // 2. Set da mobility
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
        positionAlloc->Add(apPosition);
        mobility.SetPositionAllocator(positionAlloc);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(stas.Get(1));

        for (uint32_t i = 2; i < 2 + nDrivers; i++) {
            positionAlloc = CreateObject<ListPositionAllocator> ();
            positionAlloc->Add(Vector(30.0 + (i - 2) * 3.0, 20.0, 0.0));
            mobility.SetPositionAllocator(positionAlloc);
            mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            mobility.Install(stas.Get(i));
        }

        // hore
        mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                "MinX", DoubleValue(23.0),
                "MinY", DoubleValue(6.0),
                "DeltaX", DoubleValue(1.5),
                "DeltaY", DoubleValue(1.5),
                "GridWidth", UintegerValue(3),
                "LayoutType", StringValue("ColumnFirst"));

        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

        for (uint32_t i = 2 + nDrivers; i < 2 + nDrivers + nGroup1; i++) {
            mobility.Install(stas.Get(i));
        }

        // dole
        mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                "MinX", DoubleValue(27.0),
                "MinY", DoubleValue(44.0),
                "DeltaX", DoubleValue(1.5),
                "DeltaY", DoubleValue(1.5),
                "GridWidth", UintegerValue(3),
                "LayoutType", StringValue("ColumnFirst"));

        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

        for (uint32_t i = 2 + nDrivers + nGroup1; i < 2 + nDrivers + nGroup1 + nGroup2; i++) {
            mobility.Install(stas.Get(i));
        }

        // vpravo
        mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                "MinX", DoubleValue(44.0),
                "MinY", DoubleValue(23.0),
                "DeltaX", DoubleValue(1.5),
                "DeltaY", DoubleValue(1.5),
                "GridWidth", UintegerValue(3),
                "LayoutType", StringValue("RowFirst"));

        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

        for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2; i < 2 + nDrivers + nGroup1 + nGroup2 + nGroup3; i++) {
            mobility.Install(stas.Get(i));
        }

        // vlavo 
        mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                "MinX", DoubleValue(6.0),
                "MinY", DoubleValue(27.0),
                "DeltaX", DoubleValue(1.5),
                "DeltaY", DoubleValue(1.5),
                "GridWidth", UintegerValue(3),
                "LayoutType", StringValue("RowFirst"));

        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

        for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2 + nGroup3; i < 2 + nDrivers + nGroup1 + nGroup2 + nGroup3 + nGroup4; i++) {
            mobility.Install(stas.Get(i));
        }

        for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2 + nGroup3 + nGroup4; i < 1 + nDrivers + nGroup1 + nGroup2 + nGroup3 + nGroup4 + nStandmen; i++) {
            positionAlloc = CreateObject<ListPositionAllocator> ();
            positionAlloc->Add(GetRandom3DPoint(0, 40));
            mobility.SetPositionAllocator(positionAlloc);
            mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            mobility.Install(stas.Get(i));
        }

        positionAlloc = CreateObject<ListPositionAllocator> ();
        positionAlloc->Add(GetRandom3DPoint(0, 40));
        mobility.SetPositionAllocator(positionAlloc);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(stas.Get(0));

        // 3. Create propagation loss matrix
        Ptr<MatrixPropagationLossModel> lossModel = CreateObject<MatrixPropagationLossModel> ();

        lossModel->SetDefaultLoss(400);
        lossModel->SetLoss(stas.Get(0)->GetObject<MobilityModel>(), stas.Get(1)->GetObject<MobilityModel>(), 50); // set symmetric loss 0 <-> 1 to 50 dB

        for (uint32_t i = 2; i < nWifi; i++) {
            lossModel->SetLoss(stas.Get(i)->GetObject<MobilityModel>(), stas.Get(1)->GetObject<MobilityModel>(), 50);
        }

        // 4. Create & setup wifi channel
        Ptr<YansWifiChannel> wifiChannel = CreateObject <YansWifiChannel> ();
        wifiChannel->SetPropagationLossModel(lossModel);
        wifiChannel->SetPropagationDelayModel(CreateObject <ConstantSpeedPropagationDelayModel> ());

        // 5. Install wireless devices
        WifiHelper wifi;
        wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                "DataMode", StringValue("DsssRate2Mbps"),
                "ControlMode", StringValue("DsssRate1Mbps"));

        YansWifiPhyHelper wifiPhy;
        wifiPhy.SetChannel(wifiChannel);
        WifiMacHelper wifiMac = WifiMacHelper();
        wifiMac.SetType("ns3::AdhocWifiMac"); // use ad-hoc MAC
        staDevs = wifi.Install(wifiPhy, wifiMac, stas);

        // 6. Install TCP/IP stack & assign IP addresses
        DsdvHelper dsdv;
        dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(1)));

        InternetStackHelper internet;
        internet.SetRoutingHelper(dsdv); // has effect on the next Install ()
        internet.SetTcp("ns3::TcpL4Protocol");
        internet.Install(stas);

        Ipv4AddressHelper ipv4;
        ipv4.SetBase("10.0.0.0", "255.0.0.0");
        ipv4.Assign(staDevs);

        // 7. Install applications: two CBR streams each saturating the channel 
        ApplicationContainer cbrApps;
        uint16_t cbrPort = 12345;
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address("10.0.0.2"), cbrPort));
        onOffHelper.SetAttribute("PacketSize", UintegerValue(1400));
        onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        // flow 1:  node 0 -> node 1
        onOffHelper.SetAttribute("DataRate", StringValue("3000000bps"));
        onOffHelper.SetAttribute("StartTime", TimeValue(Seconds(1.000000)));
        cbrApps.Add(onOffHelper.Install(stas.Get(0)));

        for (uint32_t i = 2; i < nWifi; i++) {
            stringstream s;
            s << i;
            string dr = s.str();
            onOffHelper.SetAttribute("DataRate", StringValue("3001" + dr + "00bps"));
            onOffHelper.SetAttribute("StartTime", TimeValue(Seconds(1.0 + i * 0.001)));
            cbrApps.Add(onOffHelper.Install(stas.Get(i)));
        }

        uint16_t echoPort = 9;
        UdpEchoClientHelper echoClientHelper(Ipv4Address("10.0.0.2"), echoPort);
        echoClientHelper.SetAttribute("MaxPackets", UintegerValue(1));
        echoClientHelper.SetAttribute("Interval", TimeValue(Seconds(0.1)));
        echoClientHelper.SetAttribute("PacketSize", UintegerValue(10));
        ApplicationContainer pingApps;

        // again using different start times to workaround Bug 388 and Bug 912
        echoClientHelper.SetAttribute("StartTime", TimeValue(Seconds(0.001)));
        pingApps.Add(echoClientHelper.Install(stas.Get(0)));

        for (uint32_t i = 2; i < nWifi; i++) {
            echoClientHelper.SetAttribute("StartTime", TimeValue(Seconds(i * 0.001)));
            pingApps.Add(echoClientHelper.Install(stas.Get(i)));
        }

        NodeContainer down_nodes, up_nodes, right_nodes, left_nodes;

        for (uint32_t i = 2; i < 2 + nDrivers; i++)
            Simulator::Schedule(Seconds(0.1 + i * 0.01), &Run, stas.Get(i));

        for (uint32_t i = 2 + nDrivers; i < 2 + nDrivers + nGroup1; i++)
            down_nodes.Add(stas.Get(i));

        for (uint32_t i = 2 + nDrivers + nGroup1; i < 2 + nDrivers + nGroup1 + nGroup2; i++)
            up_nodes.Add(stas.Get(i));

        for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2; i < 2 + nDrivers + nGroup1 + nGroup2 + nGroup3; i++)
            left_nodes.Add(stas.Get(i));

        for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2 + nGroup3; i < 2 + nDrivers + nGroup1 + nGroup2 + nGroup3 + nGroup4; i++)
            right_nodes.Add(stas.Get(i));

        Simulator::Schedule(Seconds(0.1), &MoveDown, down_nodes);
        Simulator::Schedule(Seconds(0.1), &MoveUp, up_nodes);
        Simulator::Schedule(Seconds(0.1), &MoveRight, right_nodes);
        Simulator::Schedule(Seconds(0.1), &MoveLeft, left_nodes);

        Simulator::Schedule(Seconds(change_sem_time), &zmenSemafor);

        // 8. Install FlowMonitor on all nodes
        FlowMonitorHelper flowmon;
        Ptr<FlowMonitor> monitor = flowmon.InstallAll();

        Simulator::Stop(Seconds(simulTime));
        if (animation_enabled) {
            AnimationInterface anim("animacia.xml");

            anim.UpdateNodeDescription(stas.Get(1), "Access Point");
            anim.UpdateNodeColor(stas.Get(1), 0, 0, 0);

            for (uint32_t i = 2; i < 2 + nDrivers; i++) {
                stringstream ss;
                ss << "Runner " << i - 1;
                string runner = ss.str();

                anim.UpdateNodeDescription(stas.Get(i), runner);
                anim.UpdateNodeColor(stas.Get(i), 255, 0, 0);
            }

            for (uint32_t i = 2 + nDrivers; i < 2 + nDrivers + nGroup1; i++) {
                anim.UpdateNodeDescription(stas.Get(i), "1");
                anim.UpdateNodeColor(stas.Get(i), 120, 186, 26);
            }

            for (uint32_t i = 2 + nDrivers + nGroup1; i < 2 + nDrivers + nGroup1 + nGroup2; i++) {
                anim.UpdateNodeDescription(stas.Get(i), "2");
                anim.UpdateNodeColor(stas.Get(i), 7, 255, 42);
            }

            for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2; i < 2 + nDrivers + nGroup1 + nGroup2 + nGroup3; i++) {
                anim.UpdateNodeDescription(stas.Get(i), "3");
                anim.UpdateNodeColor(stas.Get(i), 7, 2, 255);
            }

            for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2 + nGroup3; i < 2 + nDrivers + nGroup1 + nGroup2 + nGroup3 + nGroup4; i++) {
                anim.UpdateNodeDescription(stas.Get(i), "4");
                anim.UpdateNodeColor(stas.Get(i), 7, 123, 10);
            }

            for (uint32_t i = 2 + nDrivers + nGroup1 + nGroup2 + nGroup3 + nGroup4; i < 1 + nDrivers + nGroup1 + nGroup2 + nGroup3 + nGroup4 + nStandmen; i++) {
                anim.UpdateNodeDescription(stas.Get(i), "Standing Men");
                anim.UpdateNodeColor(stas.Get(i), 96, 100, 255);
            }

            anim.UpdateNodeDescription(stas.Get(0), "");
            anim.UpdateNodeColor(stas.Get(0), 255, 255, 255);
            anim.EnablePacketMetadata();
            Simulator::Run();
        } else {
            Simulator::Run();
        }

        // 10. Print per flow statistics
        monitor->CheckForLostPackets();

        Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());
        FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
        for (map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); i++) {
            if (i->first >= (nWifi)) {
                double throughput = i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000;
                data.Add(countr, throughput);
                countr += 1;

                cout << "  Throughput: " << throughput << " Mbps" << endl;
                dataTable[i->first - nWifi][k] = throughput;
            }
        }

        Simulator::Destroy();
    }

    for (uint32_t i = 0; i < nWifi - 1; i++) {
        double mean = 0;

        for (int j = 0; j < numOfMeasurements; j++) {
            mean += dataTable[i][j];
        }

        mean /= numOfMeasurements;
        double sigma = 0;

        for (int j = 0; j < numOfMeasurements; j++) {
            sigma += pow(dataTable[i][j] - mean, 2);
        }

        sigma /= numOfMeasurements;
        sigma = sqrt(sigma);
    }

    Gnuplot graf("graph.svg");
    graf.SetTerminal("svg");
    graf.SetLegend("Cislo merania", "Priepustnost v Mbps");
    graf.SetTitle("Graf zavislosti");
    graf.AddDataset(data);
    std::ofstream plotFile("graph.gnuplot");
    graf.GenerateOutput(plotFile);
    plotFile.close();
    if (system("gnuplot graph.gnuplot"));

    return 0;
}
