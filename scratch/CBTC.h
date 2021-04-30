#ifndef CBTC_H
#define CBTC_H

#include <bits/stdint-uintn.h>
#include <ns3/animation-interface.h>
#include <ns3/inet-socket-address.h>
#include <ns3/ipv4-address.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/mobility-helper.h>
#include <ns3/mobility-model.h>
#include <ns3/net-device-container.h>
#include <ns3/waypoint-mobility-model.h>
#include <ns3/node.h>
#include <ns3/node-container.h>
#include <ns3/ptr.h>
#include <ns3/socket.h>
#include <ns3/vector.h>
#include <map>
#include <stack>
#include <string>
#include <vector>


using namespace ns3;
using namespace std;

// callback
void ReceivePacket (Ptr<Socket> socket);

// other
static vector<string> GetElementsFromContext (string& context);
static Ptr <Node> GetNodeFromContext (string& context);

class CBTC: public Object {

public:

  CBTC();

  ~CBTC();

  /*--------------------- ATTRIBUTES -----------------------*/

  // nody
  NodeContainer nodyZastavky, nodyElektricky;

  // CMD line argumenty
  uint32_t pocetZastavok, pocetElektriciek, velkostUdajov, trvanieSimulacie;
  bool ulozAnimaciu/*, logging*/;

  // pohyb elektriciek
  static double interval, delay;
  static Time aktualnyCas, stopLength;
  static vector<int> cesta;
  static vector<string> nazvyZastavok;
  static vector<Vector> pozicieZastavok;
  // zastavovanie
  static map<int, int> tramPositions;

  // id , arrival time
  static map<int, Time> timings;
  static map<int, bool > isStopped;

  vector<Ptr<Socket>> sockets;


  static Vector curpos;

  /*--------------------- METHODS -----------------------*/

  // spustenie programu
  void Run();

  // Config
  void SetCommandLineArgs (int argc, char **argv);

  void CreateNodes();                                             // L1
  void ElektrickyWaypointModel(MobilityHelper &mobility);         // L1
  void ZastavkyConstantPositionModel(MobilityHelper &mobility);   // L1
  void SetP2PDevices();                                           // L2
  NetDeviceContainer SetWifiDevices();                            // L2
  void SetRouting(NetDeviceContainer nicElektricky);              // L3
  void SetApplications();                                         // L4-L7
  AnimationInterface SaveAnimation(bool save);
  static void Stop(Ptr<WaypointMobilityModel> elektricka, int id);


  // callback
  static void ScheduleNextStop (Ptr<WaypointMobilityModel> model, int id);
  static void CheckDistances (NodeContainer nodyElektricky);
  static void CourseChange (string context, Ptr<const MobilityModel> model);
  static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval);
  static void ReceivePacket (Ptr<Socket> socket);

  // ping
  void VytvorSocketyMedziElektrickami ();
  void PingniZoSource(Ptr<Socket> source, Time time);


  //test
  static void Reset(Ptr<WaypointMobilityModel> waypointModel);
};

#endif
