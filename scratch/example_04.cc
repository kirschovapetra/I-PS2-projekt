#include "ns3/object.h"
#include "ns3/uinteger.h"
#include "ns3/traced-value.h"
#include "ns3/trace-source-accessor.h"

#include <iostream>

using namespace ns3;

class MyObject : public Object {

public:

  TracedValue<int32_t> m_myInt = 0;

  static TypeId GetTypeId (void) {
    static TypeId tid = TypeId ("MyObject")
      .SetParent<Object> ()
      .SetGroupName("Tutorial")
      .AddConstructor<MyObject>()
      .AddTraceSource("MyInteger", "An integer value to trace.",
                      MakeTraceSourceAccessor (&MyObject::m_myInt), // ktory objekt/premennu budem zachytavat
                      "ns3::TracedValueCallback::Int32");           // typ funkcie na zachytenie
    return tid;
  }

  void inc(){m_myInt++;}

  MyObject () {}


};

void CallbackWithoutContextFunc (int32_t oldValue, int32_t newValue) {
  std::cout << "WITHOUT CONTEXT: Traced " << oldValue << " to " << newValue << std::endl;
}

void BoundCallbackWithoutContextFunc (/*premenne*/ int cislo, int32_t oldValue, int32_t newValue) {
  std::cout << "WITHOUT CONTEXT BOUND: cislo: " <<  cislo << " Traced " << oldValue << " to " << newValue << std::endl;
}

void BoundCallbackFunc (/*premenne*/ int cislo, /*kontext*/ std::string context, int32_t oldValue, int32_t newValue) {
  std::cout << "WITH CONTEXT BOUND: cislo: " <<  cislo << " Traced old=" << oldValue << " to new=" << newValue << " context:" << context << std::endl;
}



int main (int argc, char *argv[]) {
  Ptr<MyObject> myObject = CreateObject<MyObject> ();
  // bez kontextu
  myObject->TraceConnectWithoutContext ("MyInteger", MakeCallback (&CallbackWithoutContextFunc));

  myObject->inc(); // vypise sa pre WITHOUT CONTEXT:

  myObject->TraceDisconnectWithoutContext ("MyInteger", MakeCallback (&CallbackWithoutContextFunc)); // odpoji sa
  myObject->inc(); // nic sa nestane

  // bez kontextu, bound
  myObject->TraceConnectWithoutContext ("MyInteger", MakeBoundCallback (&BoundCallbackWithoutContextFunc, 150));
  // s kontextom, bound
  myObject->TraceConnect ("MyInteger", "serus", MakeBoundCallback (&BoundCallbackFunc, 100));

  myObject->inc(); // vypise sa pre WITH a WITHOUT CONTEXT BOUND

}
