#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/String.h>
#include <ArduinoJson.h>

  const char* sensor;
  long time1;
  double latitude;
  double longitude;
std_msgs::String debug_msg;

char buffer[256];


ros::NodeHandle  nh;
ros::Publisher pub_module("debug", &debug_msg);

char parseFailed[25] = "parseObject() failed";

  
void module_cb( const std_msgs::String& json_msg){
//  debug_msg.data=hello; 
//  pub_module.publish(&debug_msg);
  debug_msg.data=json_msg.data;
  pub_module.publish(&debug_msg);
    StaticJsonBuffer<200> jsonBuffer;
    char json[] ="{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";
    JsonObject& root = jsonBuffer.parseObject(json_msg.data);
    
   /* if(!root.success()) {
       debug_msg.data=parseFailed;
       pub_module.publish(&debug_msg);
      return;
    }*/
 
    sensor = root["sensor"];
    time1 = root["time"];
    latitude = root["data"][0];
    longitude = root["data"][1];   
   StaticJsonBuffer<200> jsonBuffer1;
   JsonObject& root1 = jsonBuffer1.createObject();
   root1["sensor2"]=sensor;
   root1["time2"]=time1;
    JsonArray& data = root1.createNestedArray("data2");
    data.add(latitude);
    data.add(longitude);
    root1.printTo(buffer, sizeof(buffer));
    debug_msg.data=buffer;
     pub_module.publish(&debug_msg); 
  
}

ros::Subscriber<std_msgs::String> sub_module("module", module_cb);



void setup(){
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pub_module);
  nh.subscribe(sub_module);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
