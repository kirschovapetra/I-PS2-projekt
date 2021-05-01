#include <bits/stdint-uintn.h>
#include <ns3/animation-interface.h>
#include <ns3/application-container.h>
#include <ns3/boolean.h>
#include <ns3/csma-helper.h>
#include <ns3/double.h>
#include <ns3/flow-classifier.h>
#include <ns3/flow-monitor.h>
#include <ns3/flow-monitor-helper.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-address.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-flow-classifier.h>
#include <ns3/ipv4-global-routing-helper.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/log.h>
#include <ns3/mobility-helper.h>
#include <ns3/names.h>
#include <ns3/net-device-container.h>
#include <ns3/node.h>
#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/object.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/position-allocator.h>
#include <ns3/ptr.h>
#include <ns3/rectangle.h>
#include <ns3/simulator.h>
#include <ns3/ssid.h>
#include <ns3/string.h>
#include <ns3/udp-echo-helper.h>
#include <ns3/uinteger.h>
#include <ns3/vector.h>
#include <ns3/waypoint-mobility-model.h>
#include <ns3/wifi-helper.h>
#include <ns3/wifi-mac-helper.h>
#include <ns3/yans-wifi-helper.h>
#include <iostream>
#include <map>
#include <string>
using namespace ns3;
using namespace std;
NS_LOG_COMPONENT_DEFINE ("cvicenie 3.2");
uint32_t dataSize = 1024;
uint32_t pocetZastavok = 10;
uint32_t pocetElektriciek = 3;
uint32_t velkostUdajov = 1024;
double trvanieSimulacie = 200.0;
double interval = 5.0;
double delay = 3.0;
Time aktualnyCas = Seconds(0.0);
vector<int> cesta{0,1,2,3,7,8,7,6,9,6,5,4,5,2,1,0};
vector<string> nazvyZastavok{"A","B", "C", "D", "E", "F", "G", "H", "I", "J"};
vector<Vector> pozicieZastavok{  Vector(0.0, 0.0, 0.0),        //A
                                 Vector(0.0, 200.0, 0.0),      //B
                                 Vector(200.0, 100.0, 0.0),    //C
                                 Vector(200.0, 200.0, 0.0),    //D
                                 Vector(300.0, 0.0, 0.0),      //E
                                 Vector(300.0, 100.0, 0.0),    //F
                                 Vector(400.0, 100.0, 0.0),    //G
                                 Vector(400.0, 200.0, 0.0),    //H
                                 Vector(400.0, 300.0, 0.0),    //I
                                 Vector(500.0, 100.0, 0.0) };

NodeContainer nodyZastavky, nodyElektricky;
map<int, int> tramPositions;
Time stopLength = Seconds(2.0);


void ScheduleNextStop (Ptr<WaypointMobilityModel> model, map<int, int> tramPositions, int id, Time interval, Time stopLength){

  int nextStop = tramPositions.at (id) + 1;

  if (nextStop == cesta.size () - 1){
      tramPositions.find (id)->second = 0;
      nextStop = 0;
  } else {
      tramPositions.find (id)->second = nextStop;
  }

//  cout << "ELEKTRICKA #" << id << " next stop -> " << nextStop << endl;
  auto pozicia = pozicieZastavok[cesta[nextStop]];
  // dalsia zastavka
  model->AddWaypoint (Waypoint (Simulator::Now () + interval, pozicia));
  // dalsia zastavka druhykrat -> aby elektricka stala
  model->AddWaypoint (Waypoint (Simulator::Now () + stopLength + interval, pozicia));
  // naschedulovanie dalsej zastavky
  Simulator::Schedule (stopLength + interval, &ScheduleNextStop, model, tramPositions, id, interval, stopLength);
}

void ZastavkyConstantPositionModel(MobilityHelper &mobility) {

  Ptr <ListPositionAllocator> pozicieZastavokAlloc = CreateObject<ListPositionAllocator>();
  for( auto& pozicia: pozicieZastavok) {
      pozicieZastavokAlloc -> Add(pozicia);
  }

  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(pozicieZastavokAlloc);
  mobility.Install(nodyZastavky);
}

void ElektrickyWaypointModel(MobilityHelper &mobility) {

  mobility.SetMobilityModel("ns3::WaypointMobilityModel", "LazyNotify",BooleanValue(true));
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

      auto cas = aktualnyCas + Seconds(interval * 0 + i * delay);
      auto pozicia = pozicieZastavok[cesta[0]];
      model->AddWaypoint (Waypoint(cas,pozicia));

      for (int j = 1; j < 2; j++) {
          auto cas = aktualnyCas + Seconds(interval * j + i * delay);
          auto pozicia = pozicieZastavok[cesta[j]];

//          cout << "zastavka: " << cesta[j] << " cas: " << cas  << endl;
          model->AddWaypoint (Waypoint(cas,pozicia));
          model->AddWaypoint (Waypoint(cas + Seconds(2), pozicia));
      }
  }

  for (int i = 0; i < nodyElektricky.GetN(); i++) {
        Ptr<WaypointMobilityModel> model = DynamicCast<WaypointMobilityModel>(nodyElektricky.Get(i)->GetObject<MobilityModel>());
        // Schedulovanie zastavok pre jednotilve elektricky
        Simulator::Schedule (stopLength + Seconds( + interval + (i * delay)), &ScheduleNextStop, model, tramPositions, i, Seconds(interval), stopLength);
  }
}

void SetP2PDevices() {

  // spojenia medzi dvojicami zastavok
  vector<vector<string>> spojenia = { {"A","B"}, {"B","C"}, {"C","D"}, {"C","F"}, {"D","F"}, {"D","H"},
                                      {"F","E"}, {"F","G"}, {"F","H"}, {"G","H"}, {"G","J"}, {"H","I"} };

  PointToPointHelper p2pHelper;
  for (auto &spoj: spojenia)
    p2pHelper.Install(spoj[0], spoj[1]);

}


void CreateNodes() {

  // zastavky
  nodyZastavky.Create(pocetZastavok);
  for (int i = 0; i < pocetZastavok; i++)
      Names::Add (nazvyZastavok[i], nodyZastavky.Get(i));

  // elektricky
  nodyElektricky.Create(pocetElektriciek);
  for (int i = 0; i < pocetElektriciek; i++)
      Names::Add (to_string(i+1), nodyElektricky.Get(i));
}

void Run(){
  CreateNodes();
  SetP2PDevices();
  MobilityHelper mobility;
  ZastavkyConstantPositionModel(mobility);
  ElektrickyWaypointModel(mobility);


  /******************************* animacia ***********************************/

  AnimationInterface anim ("animations/test.xml");
  anim.EnablePacketMetadata();
  anim.SetStartTime(Seconds(0.0));
  anim.SetStopTime(Seconds(trvanieSimulacie));

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

  Simulator::Stop(Seconds(trvanieSimulacie));
  Simulator::Run ();
  Simulator::Destroy ();
}

int main (int argc, char *argv[]) {
  Run();

  return 0;
}
