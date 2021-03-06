
#ifndef b4r_main_h
#define b4r_main_h
class b4r_timenist;

class b4r_main {
public:

static void initializeProcessGlobals();
static void _appstart();
static void _connecttowifi();
static void _mqtt_connect(Byte _unused);
static void _mqtt_disconnected();
static void _mqtt_messagearrived(B4R::B4RString* _topic,B4R::Array* _payload);
static void _process_globals();
static B4R::Serial* _serial1;
static B4R::B4RESPWiFi* _wifi;
static B4R::WiFiSocket* _wifistr;
static B4R::MqttClient* _mqtt;
static B4R::MqttConnectOptions* _mqttopt;
static B4R::B4RString* _mqttuser;
static B4R::B4RString* _mqttpassword;
static B4R::B4RString* _mqtthostname;
static Int _mqttport;
static B4R::B4RESP8266* _esp;
static B4R::B4RString* _wifissid;
static B4R::B4RString* _wifipassword;
static Double _smokelevel;
static b4r_timenist* _timenist;
static void _readsmoke(Byte _tag);
static void _timeisavailable();
};

#endif