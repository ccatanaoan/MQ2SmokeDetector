#include "B4RDefines.h"

B4R::Serial* b4r_main::_serial1;
B4R::B4RESPWiFi* b4r_main::_wifi;
B4R::WiFiSocket* b4r_main::_wifistr;
B4R::MqttClient* b4r_main::_mqtt;
B4R::MqttConnectOptions* b4r_main::_mqttopt;
B4R::B4RString* b4r_main::_mqttuser;
B4R::B4RString* b4r_main::_mqttpassword;
B4R::B4RString* b4r_main::_mqtthostname;
Int b4r_main::_mqttport;
B4R::B4RESP8266* b4r_main::_esp;
B4R::B4RString* b4r_main::_wifissid;
B4R::B4RString* b4r_main::_wifipassword;
Double b4r_main::_smokelevel;
b4r_timenist* b4r_main::_timenist;
static B4R::Serial be_gann1_3;
static B4R::B4RESPWiFi be_gann2_3;
static B4R::WiFiSocket be_gann3_3;
static B4R::MqttClient be_gann4_3;
static B4R::MqttConnectOptions be_gann5_3;
static B4R::B4RString be_gann6_5;
static B4R::B4RString be_gann7_5;
static B4R::B4RString be_gann8_5;
static B4R::B4RESP8266 be_gann10_3;
static B4R::B4RString be_gann11_5;
static B4R::B4RString be_gann12_5;


 /*******************Demo for MQ-2 Gas Sensor Module V1.0*****************************
  Support:  Tiequan Shao: support[at]sandboxelectronics.com

  Lisence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)

  Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application.

                                                    Sandbox Electronics    2011-04-25
************************************************************************************/

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       A0     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
//normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3, 0.21, -0.47}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float           COCurve[3]  =  {2.3, 0.72, -0.34};  //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float           SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

int temp = 0;

void setup(B4R::Object* o){
  Serial.print("Calibrating...\n");
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air
  //when you perform the calibration
  Serial.print("Calibration is done...\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
}

void ReadSmoke(B4R::Object* o) {
  temp = analogRead(MQ_PIN);
  Serial.print("Voltage:");
  Serial.print(temp);
  Serial.print( "V" );
  Serial.print("    ");
  Serial.print("LPG:");
  //Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG) );
  Serial.print( "ppm" );
  Serial.print("    ");
  Serial.print("CO:");
  //Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO) );
  Serial.print( "ppm" );
  Serial.print("    ");
  Serial.print("SMOKE:");
 //Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE) );
  b4r_main::_smokelevel = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE); 
  Serial.print( "ppm" );
  Serial.print("\n");
  delay(200);
}

 void SetSTA(B4R::Object* o) {
   WiFi.mode(WIFI_STA);
}

/****************** MQResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
  Input:   mq_pin - analog channel
  Output:  Ro of the sensor
  Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}
/*****************************  MQRead *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
  Output:  ppm of the target gas
  Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}

/*****************************  MQGetPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
void b4r_main::_appstart(){
const UInt cp = B4R::StackMemory::cp;
 //BA.debugLineNum = 40;BA.debugLine="Private Sub AppStart";
 //BA.debugLineNum = 41;BA.debugLine="Serial1.Initialize(115200)";
b4r_main::_serial1->Initialize((ULong) (115200));
 //BA.debugLineNum = 42;BA.debugLine="Delay(3000)";
Common_Delay((ULong) (3000));
 //BA.debugLineNum = 43;BA.debugLine="Log(\"AppStart\")";
B4R::Common::LogHelper(1,102,F("AppStart"));
 //BA.debugLineNum = 54;BA.debugLine="RunNative(\"setup\",Null)";
Common_RunNative(setup,Common_Null);
 //BA.debugLineNum = 55;BA.debugLine="ReadSmoke(0)";
_readsmoke((Byte) (0));
 //BA.debugLineNum = 56;BA.debugLine="End Sub";
B4R::StackMemory::cp = cp;
}
void b4r_main::_connecttowifi(){
const UInt cp = B4R::StackMemory::cp;
 //BA.debugLineNum = 92;BA.debugLine="Sub ConnectToWifi";
 //BA.debugLineNum = 93;BA.debugLine="Log(\"Connecting to WiFi\")";
B4R::Common::LogHelper(1,102,F("Connecting to WiFi"));
 //BA.debugLineNum = 94;BA.debugLine="WiFi.Connect2(WiFiSSID, WiFiPassword)";
b4r_main::_wifi->Connect2(b4r_main::_wifissid,b4r_main::_wifipassword);
 //BA.debugLineNum = 96;BA.debugLine="If WiFi.IsConnected Then";
if (b4r_main::_wifi->getIsConnected()) { 
 //BA.debugLineNum = 97;BA.debugLine="Log(\"Connected to \",WiFiSSID,\" network, Local IP";
B4R::Common::LogHelper(4,102,F("Connected to "),101,b4r_main::_wifissid->data,102,F(" network, Local IP "),101,b4r_main::_wifi->getLocalIp()->data);
 //BA.debugLineNum = 99;BA.debugLine="TimeNIST.Start";
b4r_main::_timenist->_start();
 }else {
 //BA.debugLineNum = 101;BA.debugLine="Log(\"Not Connected to WiFi\")";
B4R::Common::LogHelper(1,102,F("Not Connected to WiFi"));
 };
 //BA.debugLineNum = 103;BA.debugLine="End Sub";
B4R::StackMemory::cp = cp;
}
void b4r_main::_mqtt_connect(Byte _unused){
const UInt cp = B4R::StackMemory::cp;
B4R::B4RString be_ann32_4;
B4R::B4RString be_ann33_4;
 //BA.debugLineNum = 58;BA.debugLine="Sub MQTT_Connect(Unused As Byte)";
 //BA.debugLineNum = 59;BA.debugLine="If WiFi.IsConnected = False Then";
if (b4r_main::_wifi->getIsConnected()==Common_False) { 
 //BA.debugLineNum = 60;BA.debugLine="ConnectToWifi";
_connecttowifi();
 };
 //BA.debugLineNum = 63;BA.debugLine="If MQTT.Connect = False Then";
if (b4r_main::_mqtt->Connect()==Common_False) { 
 //BA.debugLineNum = 64;BA.debugLine="Log(\"Connecting to broker\")";
B4R::Common::LogHelper(1,102,F("Connecting to broker"));
 //BA.debugLineNum = 65;BA.debugLine="MQTT.Connect2(MQTTOpt)";
b4r_main::_mqtt->Connect2(b4r_main::_mqttopt);
 //BA.debugLineNum = 66;BA.debugLine="CallSubPlus(\"MQTT_Connect\", 1000, 0)";
B4R::__c->CallSubPlus(_mqtt_connect,(ULong) (1000),(Byte) (0));
 }else {
 //BA.debugLineNum = 68;BA.debugLine="Log(\"Connected to broker\")";
B4R::Common::LogHelper(1,102,F("Connected to broker"));
 //BA.debugLineNum = 69;BA.debugLine="MQTT.Subscribe(\"TempHumid\", 0)";
b4r_main::_mqtt->Subscribe(be_ann32_4.wrap("TempHumid"),(Byte) (0));
 //BA.debugLineNum = 70;BA.debugLine="MQTT.Subscribe(\"MQ7\", 0)";
b4r_main::_mqtt->Subscribe(be_ann33_4.wrap("MQ7"),(Byte) (0));
 };
 //BA.debugLineNum = 72;BA.debugLine="End Sub";
B4R::StackMemory::cp = cp;
}
void b4r_main::_mqtt_disconnected(){
const UInt cp = B4R::StackMemory::cp;
 //BA.debugLineNum = 86;BA.debugLine="Sub mqtt_Disconnected";
 //BA.debugLineNum = 87;BA.debugLine="Log(\"Disconnected from broker\")";
B4R::Common::LogHelper(1,102,F("Disconnected from broker"));
 //BA.debugLineNum = 88;BA.debugLine="MQTT.Close";
b4r_main::_mqtt->Close();
 //BA.debugLineNum = 89;BA.debugLine="MQTT_Connect(0)";
_mqtt_connect((Byte) (0));
 //BA.debugLineNum = 90;BA.debugLine="End Sub";
B4R::StackMemory::cp = cp;
}
void b4r_main::_mqtt_messagearrived(B4R::B4RString* _topic,B4R::Array* _payload){
const UInt cp = B4R::StackMemory::cp;
B4R::Object be_ann37_8;
B4R::B4RString be_ann38_3;
B4R::B4RString be_ann39_3;
B4R::B4RString be_ann40_4;
B4R::B4RString be_ann40_6;
B4R::B4RString be_ann42_3;
 //BA.debugLineNum = 74;BA.debugLine="Sub mqtt_MessageArrived (Topic As String, Payload(";
 //BA.debugLineNum = 75;BA.debugLine="Log(\"Message arrived. Topic=\", Topic, \" Payload=\"";
B4R::Common::LogHelper(4,102,F("Message arrived. Topic="),101,_topic->data,102,F(" Payload="),100,be_ann37_8.wrapPointer(_payload));
 //BA.debugLineNum = 77;BA.debugLine="If Topic = \"TempHumid\" Then";
if ((_topic)->equals(be_ann38_3.wrap("TempHumid"))) { 
 //BA.debugLineNum = 78;BA.debugLine="If Payload = \"Restart controller\" Then";
if ((_payload)->equals((be_ann39_3.wrap("Restart controller"))->GetBytes())) { 
 //BA.debugLineNum = 79;BA.debugLine="MQTT.Publish(\"TempHumid\",\"*Restarting relay by";
b4r_main::_mqtt->Publish(be_ann40_4.wrap("TempHumid"),(be_ann40_6.wrap("*Restarting relay by remote"))->GetBytes());
 //BA.debugLineNum = 80;BA.debugLine="ESP.Restart";
b4r_main::_esp->Restart();
 }else if((_payload)->equals((be_ann42_3.wrap("Read weather"))->GetBytes())) { 
 };
 };
 //BA.debugLineNum = 84;BA.debugLine="End Sub";
B4R::StackMemory::cp = cp;
}

void b4r_main::initializeProcessGlobals() {
     B4R::StackMemory::buffer = (byte*)malloc(STACK_BUFFER_SIZE);
     b4r_main::_process_globals();
b4r_timenist::_process_globals();

   
}
void b4r_main::_process_globals(){
const UInt cp = B4R::StackMemory::cp;
 //BA.debugLineNum = 22;BA.debugLine="Sub Process_Globals";
 //BA.debugLineNum = 25;BA.debugLine="Public Serial1 As Serial";
b4r_main::_serial1 = &be_gann1_3;
 //BA.debugLineNum = 26;BA.debugLine="Private WiFi As ESP8266WiFi";
b4r_main::_wifi = &be_gann2_3;
 //BA.debugLineNum = 27;BA.debugLine="Private WiFiStr As WiFiSocket";
b4r_main::_wifistr = &be_gann3_3;
 //BA.debugLineNum = 28;BA.debugLine="Private MQTT As MqttClient";
b4r_main::_mqtt = &be_gann4_3;
 //BA.debugLineNum = 29;BA.debugLine="Private MQTTOpt As MqttConnectOptions";
b4r_main::_mqttopt = &be_gann5_3;
 //BA.debugLineNum = 30;BA.debugLine="Private MQTTUser As String = \"vynckfaq\"";
b4r_main::_mqttuser = be_gann6_5.wrap("vynckfaq");
 //BA.debugLineNum = 31;BA.debugLine="Private MQTTPassword As String = \"KHSV1Q1qSUUY\"";
b4r_main::_mqttpassword = be_gann7_5.wrap("KHSV1Q1qSUUY");
 //BA.debugLineNum = 32;BA.debugLine="Private MQTTHostName As String = \"m14.cloudmqtt.c";
b4r_main::_mqtthostname = be_gann8_5.wrap("m14.cloudmqtt.com");
 //BA.debugLineNum = 33;BA.debugLine="Private MQTTPort As Int = 11816";
b4r_main::_mqttport = 11816;
 //BA.debugLineNum = 34;BA.debugLine="Private ESP As ESP8266";
b4r_main::_esp = &be_gann10_3;
 //BA.debugLineNum = 35;BA.debugLine="Private WiFiSSID As String = \"RiseAboveThisHome\"";
b4r_main::_wifissid = be_gann11_5.wrap("RiseAboveThisHome");
 //BA.debugLineNum = 36;BA.debugLine="Private WiFiPassword As String = \"SteelReserve\"";
b4r_main::_wifipassword = be_gann12_5.wrap("SteelReserve");
 //BA.debugLineNum = 37;BA.debugLine="Public SmokeLevel As Double";
b4r_main::_smokelevel = 0;
 //BA.debugLineNum = 38;BA.debugLine="End Sub";
}
void b4r_main::_readsmoke(Byte _tag){
const UInt cp = B4R::StackMemory::cp;
 //BA.debugLineNum = 111;BA.debugLine="Sub ReadSmoke(tag As Byte)";
 //BA.debugLineNum = 112;BA.debugLine="RunNative(\"ReadSmoke\",Null)";
Common_RunNative(ReadSmoke,Common_Null);
 //BA.debugLineNum = 113;BA.debugLine="Log(\"Smoke level:\", SmokeLevel)";
B4R::Common::LogHelper(2,102,F("Smoke level:"),7,(double)(b4r_main::_smokelevel));
 //BA.debugLineNum = 114;BA.debugLine="CallSubPlus(\"ReadSmoke\",1000,0)";
B4R::__c->CallSubPlus(_readsmoke,(ULong) (1000),(Byte) (0));
 //BA.debugLineNum = 115;BA.debugLine="End Sub";
B4R::StackMemory::cp = cp;
}
void b4r_main::_timeisavailable(){
const UInt cp = B4R::StackMemory::cp;
 //BA.debugLineNum = 105;BA.debugLine="Public Sub TimeIsAvailable";
 //BA.debugLineNum = 107;BA.debugLine="RunNative(\"setup\",Null)";
Common_RunNative(setup,Common_Null);
 //BA.debugLineNum = 108;BA.debugLine="ReadSmoke(0)";
_readsmoke((Byte) (0));
 //BA.debugLineNum = 109;BA.debugLine="End Sub";
B4R::StackMemory::cp = cp;
}
