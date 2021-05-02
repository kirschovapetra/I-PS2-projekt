#ifndef CBTC_H
#define CBTC_H

#include <bits/stdint-intn.h>
#include <bits/stdint-uintn.h>
#include <ns3/ipv4-interface-container.h>
#include <ns3/net-device-container.h>
#include <ns3/node-container.h>
#include <ns3/nstime.h>
#include <ns3/ptr.h>
#include <ns3/socket.h>
#include <ns3/traced-value.h>
#include <ns3/vector.h>
#include <ns3/waypoint-mobility-model.h>
#include <ns3/mobility-helper.h>
#include <map>
#include <string>
#include <vector>

using namespace ns3;
using namespace std;


class CBTC: public Object {

public:

  CBTC( uint32_t tramsN, uint32_t packetSize, uint32_t totalTime, bool saveAnim,
        uint32_t protocol, uint32_t packetInterval, bool runtimeIntervalChange, int32_t range);

  ~CBTC();

  /*--------------------- ATTRIBUTES -----------------------*/

  NodeContainer stopNodes, tramNodes;   // nody zastavok, elektriciek
  Ipv4InterfaceContainer interfaces;    // IP interfaces (elektricky)
  vector<Ptr<Socket>> sockets;          // sokety (elektricky)
  uint32_t stopsN = 10;

  // CMD line argumenty
  uint32_t tramsN, packetSize, totalTime, protocol, packetInterval, range;
  bool saveAnim, runtimeIntervalChange;
  TracedValue<int32_t> packetIntervalTrace = 2.0; // trace-ovana zmena intervalu posielania paketov

  // pohyb elektriciek
  static double delay;
  static int32_t collisionCounter;
  static int32_t stopCounter;
  static int32_t sentPacketsCounter;
  static int32_t receivedPacketsCounter;
  static Time stopLength;
  static Time pauseLength;
  static vector<int> path;
  static vector<string> stopNames;
  static vector<Vector> stopPositions;
  static map<int, int> tramPositions;     // zastavovanie
  static map<int, Time> timings;          // {id , cas prichodu na zastavku}
  static map<int, bool > isStopped;       // {id, je/nie je zastavena}
  static map<int, bool > resetStop;       // {id, moze/nemoze pokracovat v ceste}

  /*--------------------- METHODS -----------------------*/

  static TypeId GetTypeId ();

  // spustenie programu
  void Run(int argc, char **argv);

  // config
  void SetCommandLineArgs(int argc, char **argv);

  void CreateNodes();                                        // L1
  void TramsWaypointModel(MobilityHelper &mobility);         // L1
  void StopsConstantPositionModel(MobilityHelper &mobility); // L1
  void SetP2PDevices();                                      // L2
  NetDeviceContainer SetWifiDevices();                       // L2
  void SetRouting(NetDeviceContainer nicTrams);              // L3
  void SetApplications();                                    // L4-L7

  // pohyb elektriciek
  static void StopTramMovement(Ptr<WaypointMobilityModel> elektricka, int id);
  static void ScheduleNextStop(Ptr<WaypointMobilityModel> model, int id);
  static void ResetStop(int id);
  static void CheckDistances (NodeContainer tramNodes);
  // posielanie paketov
  static void GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize, Time pktInterval);
  static void ReceivePacket(Ptr<Socket> socket);

};

// prepisanie intervalu paketov
void RewritePacketInterval(Ptr<CBTC> thisPtr);
// callback na zmenu intervalu paketov
void PacketIntervalChanged (int32_t oldValue, int32_t newValue);
// callback na prijatie paketu
void ReceivePacket (Ptr<Socket> socket);

/***************************************** vytvaranie grafov *********************************************/

#endif
