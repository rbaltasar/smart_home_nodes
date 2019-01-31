/*
  Home automation - kitchen node

  Data gathering:
    Kitchen status

  Data publishing:
    Kitchen status
    Kitchen warning

  Topic subscription:

  Logic:
    If kitchen is ON more than a defined threshold, publish a warning event

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* Network settings */
const char* ssid = "........";
const char* password = "........";
const char* mqtt_server = "192.168.1.51";

/* Hardware settings */
#define LED_DETECTION_PIN 1
#define LED_INDICARION_PIN 2
#define BUZZER_PIN 2
#define KITCHEN_ON_MAX 3600 //seconds
#define ALARM_DURATION 2000 //milliseconds
#define POLLING_PERIOD 1000 //milliseconds

/* Logical states */
bool kitchen_status_on = false;
bool kitchen_status_on_old = false;
bool warning_triggered = false;
bool warning_status = false;
int start_time = 0;

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
long last_iteration = 0;
char msg[5];
//StaticJsonBuffer<200> jsonBuffer;
//JsonObject& JSONencoder = JSONbuffer.createObject();

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  setup_wifi();
  setup_hardware();
  setup_mqtt();

}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);
  /* Define callback function */
  //Not needed
  /* Subscribe to topics */
  //Not needed
}

/* Connect to the wireless network */
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/* Configure the hardware */
void setup_hardware()
{
 /* Setup pins */

}

/* Reconnect to the MQTT broker */
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/* Handle the network part of the loop */
void network_loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

/* Get the status of the kitchen fires */
void get_kitchen_status()
{
  /* Poll actual sensor */
  int polled_value = digitalRead(LED_DETECTION_PIN);

  /* Save old status for change detection */
  kitchen_status_on_old = kitchen_status_on;

  if(polled_value) kitchen_status_on = true;
  else kitchen_status_on = false;
}


/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get the status of the door */
  get_kitchen_status();
}

/* Make sound with the buzzer */
void trigger_alarm()
{
  int duration = ALARM_DURATION;

  while(duration > 0)
  {
    /* Trigger buzzer */

    delay(1);
    duration--;
  }
}

/* Node-specific logic */
/* If kitchen is ON for more than the defined time --> trigger alarm */
/* If state change OFF->ON --> start timer */
void node_logic()
{

  /* Start timer with state change ON->OFF. Publish information */
  if(kitchen_status_on && !kitchen_status_on_old) start_time = millis(); //TODO: change to seconds

  /* Trigger alarm in case of warning status. Restart the counter */
  if(kitchen_status_on)
  {
    int time_on = millis() - start_time;
    if(time_on > KITCHEN_ON_MAX)
    {
      start_time = 0;
      trigger_alarm();
      warning_status = true;
    }
  }
  /* Switch off warning state when the kitchen is off */
  else
  {
    warning_status = false;
    warning_triggered = false;
  }

}

/* Publish sensor information */
void publish_status()
{
  /* Publish kitchen on information with status update */
  if(kitchen_status_on != kitchen_status_on_old)
  {
    snprintf(msg,75,"%ld",(int)door_status_open);
    client.publish("kitchen_node/kitchen_status", msg);
  }

  /* Publish warning information only once */
  if(warning_status && !warning_triggered)
  {
    client.publish("kitchen_node/warning", 0);
    warning_triggered = true;
  }
}

void loop() {

  /* Connection handling */
  network_loop();

  long now = millis();
  if( (now - last_iteration) > POLLING_PERIOD )
  {
    last_iteration = now;

    /* Sensor polling */
    poll_sensors();

    /* Internal logic */
    node_logic();

    /* Publish sensor information */
    publish_status();
  }

}
