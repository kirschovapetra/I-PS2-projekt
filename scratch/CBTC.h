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

// callbacky
void ReceivePacket (Ptr<Socket> socket);

// other
static vector<string> GetElementsFromContext (string& context);
static Ptr <Node> GetNodeFromContext (string& context);

class CBTC: public Object {

public:

  CBTC();

  ~CBTC();

  /*--------------------- ATTRIBUTES -----------------------*/
  NodeContainer nodyZastavky, nodyElektricky;   // nody
  Ptr<Socket> srcSocket, recSocket;             // sockety
  MobilityHelper mobility;                      // mobility helper
  vector<NetDeviceContainer> nicZastavky;       // CSMA zariadenia
  NetDeviceContainer nicElektricky;             // WIFI zariadenia
  Ipv4InterfaceContainer interfacesElektricky;  // WIFI interfaces
  const InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"), 80);
  const InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny (), 80);


  map<int, int> tramPositions;
  map<int, stack<int>> waypoints;

  // CMD line argumenty
  uint32_t pocetZastavok, pocetElektriciek, velkostUdajov, trvanieSimulacie;
  bool ulozAnimaciu/*, logging*/;


  // pohyb elektriciek
  static double interval, delay;
  static Time aktualnyCas;
  static vector<int> cesta;
  static vector<string> nazvyZastavok;
  static vector<Vector> pozicieZastavok;



  /*--------------------- METHODS -----------------------*/

  void Run(); // spustenie programu

  // Config
  void SetCommandLineArgs (int argc, char **argv);
  void CreateNodes();                   // L1
  void elektrickyWaypointModel();       // L1
  void zastavkyConstantPositionModel(); // L1
  void SetCsmaDevices();                // L2
  void SetWifiDevices();                // L2
  void SetRouting();                    // L3
  void SetApplications();               // L4-L7

  void GenerateAnimation();
  static void Stop(Ptr<Node> elektricka);

  // callback
  static void CourseChange (map<int, int> tramPositions, map<int, stack<int>> waypoints,string context, Ptr<const MobilityModel> model);
  static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval);
};

#endif
