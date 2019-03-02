/*
  Home automation - entrance node

  Data gathering:
    Door status
    Presence

  Data publishing:
    Door status
    Entrance detected

  Topic subscription:
    entrance/warning

  Logic:
    Distinguish between entrance and exit

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* Network settings */
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "192.168.2.118";

/* Debugging */
#define DEBUG_ENABLED 0

/* Hardware settings */
#define DOOR_STATUS_PIN D2
#define PRESSURE_SENSOR_PIN D5
//#define BUZZER_PIN D8
#define PRESENCE_COUNTER_MAX 10
#define ENTRANCE_DETECTION_THRESHOLD 20000 //milliseconds
#define EVENT_MAX_TX_FREQ 10000 //milliseconds
#define EVENT_MAX_PRESSURE_FREQ ENTRANCE_DETECTION_THRESHOLD
#define ALARM_DURATION 1 //repetitions
#define POLLING_PERIOD 500 //milliseconds

/* Logical states */
unsigned long time_since_pressure = 0;
volatile bool door_status_open = false;
bool door_status_open_old = false;
volatile bool pressure_detected = false;
bool pressure_detected_old = false;
bool warning_status = false;
unsigned long last_pressure_detected = 0;
unsigned long last_event_published = 0;
unsigned long last_event_pressure_published = 0;
bool entrance_detected = false;
bool exit_detected = false;

uint16_t disconnect_count = 0;

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);
volatile unsigned long last_iteration = 0;

void setup() {

#if 1
  Serial.begin(115200);
#endif

  attachInterrupt(digitalPinToInterrupt(DOOR_STATUS_PIN), door_detection_isr, RISING);

  setup_wifi();
  setup_hardware();
  setup_mqtt();

  /* Publish a boot event for error tracking and debugging */
  bool publish_succeeded = false;
  while(!publish_succeeded)
  {
    Serial.println("Hello");
    publish_succeeded = client.publish("entrance_node/boot", "");
    if(!publish_succeeded) network_loop();
    yield();
    Serial.println("Hello");
  }

  blink_led(50);
}

/* Subscribe to the MQTT topics */
void setup_mqtt()
{
  /* Define MQTT broker */
  client.setServer(mqtt_server, 1883);

#if DEBUG_ENABLED
  Serial.println("MQTT setup completed");
#endif
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

#if DEBUG_ENABLED
   Serial.println("Wifi setup completed");
#endif
}

/* Configure the hardware */
void setup_hardware()
{
 /* Setup pins */
 pinMode(DOOR_STATUS_PIN, INPUT_PULLUP);
 pinMode(PRESSURE_SENSOR_PIN, INPUT);//INPUT_PULLUP);

 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, HIGH);

#if DEBUG_ENABLED
  Serial.println("Hardware setup completed");
#endif

}

/* Reconnect to the MQTT broker */
void reconnect() {

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("EntranceClient"))
    {
      Serial.println("connected");
    }
    else
    {
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
  /* Enter critical area */
  noInterrupts();

  /* MQTT loop */
  if (!client.connected()) reconnect();
  client.loop();

  /* Exit critical area */
  interrupts();
}

void blink_led(uint16_t delay_ms)
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, LOW);
  delay(delay_ms);
  network_loop();
  digitalWrite(LED_BUILTIN, HIGH);
}

/* Get the status of the door */
void get_door_status()
{
  /* Poll sensor */
  int polled_value = digitalRead(DOOR_STATUS_PIN);

  /* Save old status for change detection */
  door_status_open_old = door_status_open;

  if(polled_value)
  {
    door_status_open = true;
  }
  else door_status_open = false;

 #if DEBUG_ENABLED
  Serial.print("Door detection: ");
  Serial.println(polled_value);
 #endif

}

/* Get the status of the presence sensor and update presence counter */
void get_pressure_status()
{
  /* Save old status for change detection */
  pressure_detected_old = pressure_detected;


  /* Debounce sensor */
  int polled_value;
  uint8_t detection_count = 0;

  polled_value = digitalRead(PRESSURE_SENSOR_PIN);
  delay(50);
  if(polled_value) detection_count++;
  polled_value = digitalRead(PRESSURE_SENSOR_PIN);
  delay(100);
  if(polled_value) detection_count++;

  if(detection_count == 2)
  {
    pressure_detected = true;
    last_pressure_detected = millis();
  }
  else pressure_detected = false;

  #if DEBUG_ENABLED
  Serial.print("Polled value presence: ");
  Serial.println(polled_value);
  Serial.print("Presence detection: ");
  Serial.println(pressure_detected);
  #endif
}

void pressure_detection_isr()
{
  last_iteration = 0;
  #if DEBUG_ENABLED
  Serial.println("Pressure detected via interrupt");
  #endif
}

void door_detection_isr()
{
  last_iteration = 0;
  #if DEBUG_ENABLED
  Serial.println("Door detected via interrupt");
  #endif
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get the status of the door */
  get_door_status();

  /* Get the status of the presence detector */
  get_pressure_status();
}

/* Node-specific logic */
/* If presence has been detected before door opens --> somebody left */
/* If presence has been detected after door opens --> somebody came in */
void node_logic()
{
   /* Distinguish between entrance and exit */
  if(door_status_open && !door_status_open_old) //Only with status update
  {
    time_since_pressure = millis() - last_pressure_detected;
    #if DEBUG_ENABLED
    Serial.print("Time since last pressure detection (seconds): ");
    Serial.println(time_since_pressure / 1000);
    #endif
    if(time_since_pressure < ENTRANCE_DETECTION_THRESHOLD)
    {
      entrance_detected = true;
      #if DEBUG_ENABLED
      Serial.println("Entrance detected");
      #endif
    }
    else
    {
      exit_detected = true;
      #if DEBUG_ENABLED
      Serial.println("Exit detected");
      #endif
    }
  }

  /* Trigger alarm in case of warning status */
  //if(warning_status && door_status_open) trigger_alarm();
}

/* Publish sensor information */
void publish_status()
{

  /* Measure current time only once */
  unsigned long current_time = millis();

  /* Publish door open information with status update */
  if(door_status_open != door_status_open_old)
  {
#if DEBUG_ENABLED
    Serial.print("Change in door status: ");
    Serial.println(door_status_open);
#endif
    client.publish("entrance_node/door_status", String(door_status_open).c_str());
  }

  /* Publish door open information with status update */
  unsigned long time_since_pressure_event = current_time - last_event_pressure_published;
  if((pressure_detected != pressure_detected_old) && (time_since_pressure_event > EVENT_MAX_PRESSURE_FREQ) )
  {
#if DEBUG_ENABLED
    Serial.print("Change in pressure_detected: ");
    Serial.println(pressure_detected);
#endif
    client.publish("entrance_node/pressure_detected", String(pressure_detected).c_str());
    last_event_pressure_published = current_time;
  }

  /* Publish entrance & exit information when detected */
  unsigned long time_since_event = current_time - last_event_published;
  if(entrance_detected && (time_since_event > EVENT_MAX_TX_FREQ))
  {
    #if DEBUG_ENABLED
    Serial.println("Publishing event: entrance detected");
    #endif

    /* Ensure that the critical topics get published */
    bool publish_succeeded = false;
    while(!publish_succeeded)
    {
      publish_succeeded = client.publish("entrance_node/entrance_detected", String(time_since_pressure).c_str());
      if(!publish_succeeded) network_loop();
      yield();
    }
    last_event_published = current_time;
    time_since_event = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }

  if(exit_detected && (time_since_event > EVENT_MAX_TX_FREQ))
  {
    #if DEBUG_ENABLED
    Serial.println("Publishing event: exit detected");
    #endif

    /* Ensure that the critical topics get published */
    bool publish_succeeded = false;
    while(!publish_succeeded)
    {
      publish_succeeded = client.publish("entrance_node/exit_detected", String(exit_detected).c_str());
      if(!publish_succeeded) network_loop();
      yield();
    }
    /* After an exit event, sleep 3 seconds to prevent false entrance events */
    last_event_published = current_time;
    time_since_event = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }

  /* Show with the LED the period where no events are published */
  if(time_since_event > EVENT_MAX_TX_FREQ) digitalWrite(LED_BUILTIN, HIGH);

  /* Delete entrance & exit states after publish */
  entrance_detected = false;
  exit_detected = false;
}

/* Loop with node-specific stuff */
/* This shall include polling the sensors, doing any node-specifi logic and
 * publish the data.
 * This loop is executed atomically
 */
void node_specific_loop()
{
#if DEBUG_ENABLED
  /* Performance measurement */
  unsigned long current_time = millis();
#endif

  /* Disable interrupts */
  noInterrupts();

  /* Sensor polling */
  poll_sensors();

  /* Internal logic */
  node_logic();

  /* Publish sensor information */
  publish_status();

  /* Leave critical area */
  interrupts();

#if DEBUG_ENABLED
  current_time = millis() - current_time;
  Serial.print("Duration node loop: ");
  Serial.println(current_time);
#endif
}

void loop() {

  /* Connection handling */
  network_loop();

  long now = millis();
  if( (now - last_iteration) > POLLING_PERIOD )
  {
    /* Node specific loop */
    node_specific_loop();

    /* Update last iteration time */
    last_iteration = now;
  }
}
