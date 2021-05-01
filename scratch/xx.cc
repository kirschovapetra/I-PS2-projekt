/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Vikas Pushkar (Adapted from third.cc)
 */

#include <bits/stdint-uintn.h>
#include <ns3/animation-interface.h>
#include <ns3/callback.h>
#include <ns3/double.h>
#include <ns3/event-id.h>
#include <ns3/event-impl.h>
#include <ns3/flow-classifier.h>
#include <ns3/flow-monitor.h>
#include <ns3/flow-monitor-helper.h>
#include <ns3/gnuplot.h>
#include <ns3/inet-socket-address.h>
#include <ns3/int64x64-128.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-address.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-flow-classifier.h>
#include <ns3/ipv4-global-routing-helper.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/mobility-helper.h>
#include <ns3/net-device-container.h>
#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/object-factory.h>
#include <ns3/pointer.h>
#include <ns3/position-allocator.h>
#include <ns3/ptr.h>
#include <ns3/rng-seed-manager.h>
#include <ns3/simulator.h>
#include <ns3/socket.h>
#include <ns3/string.h>
#include <ns3/type-id.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/wifi-standards.h>
#include <ns3/yans-wifi-helper.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <string>




using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WirelessAnimationExample");

const int numOfRuns = 2;
const int numOfTests = 5;
const int nodesMultiplier = 3;

void ReceivePacket(Ptr<Socket> socket) {
    while (socket->Recv()) {
        //NS_LOG_UNCOND("Received one packet!");
    }
}

static void GenerateTraffic(Ptr<Socket> socket, double pktSize, double pktCount) {
    if (pktCount > 0) {
        socket->Send(Create<Packet> (pktSize));

        Time interval = Seconds(((rand() % 20) + 1)/10);
        Simulator::Schedule(interval, &GenerateTraffic, socket, pktSize, pktCount - 1);
    } else {
        socket->Close();
    }

}

void PrintNodeResults(double nodeNumber, Ipv4Address sourceAddress, Ipv4Address destinationAddress, double packetsSent, double packetsReceived, double throughput)
{
    std::cout << "Flow " << nodeNumber << " (" << sourceAddress << " -> " << destinationAddress << ")\n";
    std::cout << "  Tx Packets: " << packetsSent << "\n";
    std::cout << "  Rx Packets: " << packetsReceived << "\n";
    std::cout << "  Throughput: " << throughput << " Mbps\n";
}

double CalculateAverageThroughput(double runsThroughputs[numOfRuns][numOfTests], int numOfRuns, int testNumber) {
    double sum = 0;
    for (int runNumber = 0; runNumber < numOfRuns; runNumber++)
    {
        sum += runsThroughputs[runNumber][testNumber];
    }

    return sum/numOfRuns;
}

void CalculateStandardDeviation(double runsThroughputs[numOfRuns][numOfTests], int numOfRuns, int numOfTests, Gnuplot2dDataset* gnuplotData)
{
    double averages[numOfTests];
    double deviations[numOfTests];

    for (int testNumber = 0; testNumber < numOfTests; testNumber++)
    {
        averages[testNumber] = CalculateAverageThroughput(runsThroughputs, numOfRuns, testNumber);
        double sum = 0;
        for (int runNumber = 0; runNumber < numOfRuns; runNumber++)
        {
            sum += pow(runsThroughputs[runNumber][testNumber] - averages[testNumber], 2);
        }
        deviations[testNumber] = sqrt(sum/numOfRuns);

        std::cout << "Test: " << testNumber << " odcyhlka: " << deviations[testNumber] << "\n";

        double numOfNodes = (testNumber+1)*nodesMultiplier;
        gnuplotData->Add(numOfNodes, averages[testNumber], deviations[testNumber]);
    }
}

NetDeviceContainer SetupWifi(NodeContainer carNodes)
{
    std::string phyMode("DsssRate1Mbps");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);

    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("RxGain", DoubleValue(-10));
    wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");

    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
            "DataMode", StringValue(phyMode),
            "ControlMode", StringValue(phyMode));

    wifiMac.SetType("ns3::AdhocWifiMac");

    return wifi.Install(wifiPhy, wifiMac, carNodes);
}

Ptr<PositionAllocator> CreateDiscAllocator()
{
    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomDiscPositionAllocator");
    pos.Set("X", DoubleValue(5));
    pos.Set("Y", DoubleValue(5));
    pos.Set("Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=10]"));

    return pos.Create()->GetObject<PositionAllocator> ();
}

void SetupMobility(NodeContainer carNodes)
{
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
            "X", DoubleValue(5),
            "Y", DoubleValue(5),
            "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=10]"));

    Ptr<PositionAllocator> discPositionAllocator = CreateDiscAllocator();

    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
            "Speed", StringValue("ns3::UniformRandomVariable[Min=10|Max=20]"),
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0]"),
            "PositionAllocator", PointerValue(discPositionAllocator));

    mobility.Install(carNodes);
}

int main(int argc, char *argv[]) {
    srand(time(0));
    SeedManager::SetSeed(rand());
    SeedManager::SetRun(numOfRuns);

    Gnuplot gnuplot("demolitionDerby-graph.svg");
    gnuplot.SetTerminal("svg");
    gnuplot.SetTitle("Throughput in relation to number of nodes");
    gnuplot.SetLegend("Number of nodes", "Throughput [Mbps]");
    // give the graph some padding on both sides
    gnuplot.AppendExtra("set xrange[-1:" + std::to_string(numOfTests*nodesMultiplier*1.2) + "]");

    Gnuplot2dDataset gnuplotData;
    gnuplotData.SetStyle (Gnuplot2dDataset::LINES_POINTS);
    gnuplotData.SetErrorBars(Gnuplot2dDataset::Y);

    double runsThroughputs[numOfRuns][numOfTests], runsPacketsSends[numOfRuns][numOfTests], runsPacketsReceiveds[numOfRuns][numOfTests];
    for (int runNumber = 0; runNumber < numOfRuns; runNumber++) {
        //double testsThroughputs[numOfTests], testsPacketsSends[numOfTests], testsPacketsReceiveds[numOfTests];
        for (int testNumber = 0; testNumber < numOfTests; testNumber++) {
            double packetSize = 4096, numOfPackets = 3*6*9;

            double simulationDuration = 800.0;

            double numOfCars = (testNumber + 1)*nodesMultiplier;

            NodeContainer carNodes;
            carNodes.Create(numOfCars);

            NetDeviceContainer devices = SetupWifi(carNodes);
            SetupMobility(carNodes);

            // Internet
            InternetStackHelper stack;
            stack.Install(carNodes);

            Ipv4AddressHelper address;
            address.SetBase("10.1.1.0", "255.255.255.0");

            Ipv4InterfaceContainer staInterfaces;
            staInterfaces = address.Assign(devices);

            TypeId tid;
            tid = TypeId::LookupByName("ns3::UdpSocketFactory");

            Ptr<Socket> recvSinks[carNodes.GetN()];
            Ptr<Socket> sources[carNodes.GetN()];
            for (int i = 0; i < carNodes.GetN(); i++) {
                recvSinks[i] = Socket::CreateSocket(carNodes.Get(i), tid);
                InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
                recvSinks[i]->Bind(local);
                recvSinks[i]->SetRecvCallback(MakeCallback(&ReceivePacket));

                int r;
                while ((r = rand() % carNodes.GetN()) == i);

                sources[i] = Socket::CreateSocket(carNodes.Get(r), tid);
                InetSocketAddress remote = InetSocketAddress(staInterfaces.GetAddress(i, 0), 80);
                sources[i]->Connect(remote);
            }

            Ipv4GlobalRoutingHelper::PopulateRoutingTables();

            for (int i = 0; i < numOfCars; i++) {
                Time interPacketInterval = Seconds(((rand() % 20) + 1)/10);
                Simulator::ScheduleWithContext(sources[i]->GetNode()->GetId(),
                        Seconds(15.0 + i),
                        &GenerateTraffic,
                        sources[i],
                        packetSize,
                        numOfPackets / numOfCars);
            }

            // FlowMon
            FlowMonitorHelper flowmon;
            Ptr<FlowMonitor> monitor = flowmon.InstallAll();

            Simulator::Stop(Seconds(simulationDuration));

            // NetAnim
            AnimationInterface anim("demolitionDerby-animation.xml");

            for (uint32_t i = 0; i < numOfCars; i++) {
                anim.UpdateNodeDescription(carNodes.Get(i), "Car " + std::to_string(i));
                anim.UpdateNodeColor(carNodes.Get(i), 20*i, 20*i, 20*i);
            }

            anim.EnablePacketMetadata(); // Optional
            anim.EnableIpv4RouteTracking("routingtable-wireless.xml", Seconds(0), Seconds(simulationDuration), Seconds(0.25));
            anim.EnableWifiMacCounters(Seconds(0), Seconds(simulationDuration));
            anim.EnableWifiPhyCounters(Seconds(0), Seconds(simulationDuration));

            Simulator::Run();

            // FlowMonitor
            monitor->CheckForLostPackets();
            Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());
            FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
            double totalThroughput = 0;
            double totalPacketsSent = 0;
            double totalPacketsReceived = 0;
            for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
                double packetsSent = i->second.txPackets;
                double packetsReceived = i->second.rxPackets;
                double throughput = i->second.rxBytes * 8.0 / (simulationDuration * 1000000.0);

                Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
                PrintNodeResults(i->first, t.sourceAddress, t.destinationAddress, packetsSent, packetsReceived, throughput);

                totalThroughput += throughput;
                totalPacketsSent += packetsSent;
                totalPacketsReceived += packetsReceived;
            }

            std::cout << "\n" << "Total throughput: " << totalThroughput << " Mbps\n";
            std::cout << "Total packets sent: " << totalPacketsSent << "\n";
            std::cout << "Total packets received: " << totalPacketsReceived << "\n";

            runsThroughputs[runNumber][testNumber] = totalThroughput;
            runsPacketsSends[runNumber][testNumber] = totalPacketsSent;
            runsPacketsReceiveds[runNumber][testNumber] = totalPacketsReceived;

            Simulator::Destroy();
        }

        std::cout << "\n";
        for (int i = 0; i < numOfTests; i++) {
            std::cout << runsThroughputs[runNumber][i] << "Mbps - " << runsPacketsReceiveds[runNumber][i] << "/" << runsPacketsSends[runNumber][i] << "\n";
        }
    }

    CalculateStandardDeviation(runsThroughputs, numOfRuns, numOfTests, &gnuplotData);

    gnuplot.AddDataset(gnuplotData);
    std::ofstream plotFile ("demolitionDerby-plot.plt");
    gnuplot.GenerateOutput (plotFile);
    plotFile.close ();

    return 0;
}
