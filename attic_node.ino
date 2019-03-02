/*
  Home automation - attic node

  Data gathering:
    Temperature & humidity
    Light amount

  Data publishing:
    Temperature & humidity
    Light amount

  Topic subscription:

  Node description:
    The node gathers and publishes temperature, humidity and light amount
    The node keeps track of the state of the door (interrupts + polling)
    On a door interrupt, the node inmediatelly enters the node_specific_loop.
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <Wire.h>
#include <BH1750.h>

/* Network settings */
const char* ssid = "";
const char* password = "";
const char* mqtt_server = "192.168.2.118";

/* Debugging */
#define DEBUG_ENABLED 0

/* Hardware settings */
#define TEMPERATURE_SENSOR_PIN D1
#define DOOR_SENSOR_PIN D2
#define DHTTYPE DHT11   // DHT 11
#define LIGHT_SENSOR_PIN_1 D6
#define LIGHT_SENSOR_PIN_2 D7
#define MEASUREMENT_DELTA 0.01 //Minimum change in measurements (%) to publish the data

/* Ensure a minimum publish rate */
#define MINIMUM_PUBLISH_RATE 600000 //Publish each topic at least every 10 minutes
#define NUM_PUBLISH_TOPICS 3
#define DEFAULT_POLLING_PERIOD 20000 //20 seconds
#define WARNING_POLLING_PERIOD 20000 //20 seconds. Currently not used
#define SLEEP_TIME 1000

/* Logical states */
/* Signal smoothing filters */
/*
 * The signal smoothing filter is implemented as a circular buffer.
 * For simplicity, the last position of the buffer contains always the index
 * of the oldest element in the buffer.
 */
 #define BUFFER_SIZE 20
float temperature_buffer[BUFFER_SIZE+1];
float humidity_buffer[BUFFER_SIZE+1];

/* Logical states */
/* Door status tracking */
bool door_status_open = false;
bool door_status_open_old = false;

/* Light measurement (no averaging needed) */
float light_amount = 0;
float light_amount_old = 99999;

/* Temperature measurement (average after filtering) */
//Improvement: create a struct
float temperature = 0;
float temperature_old = 99999;

/* Humidity measurement (average after filtering) */
//Improvement: create a struct
float humidity = 0;
float humidity_old = 99999;

/* Period to enter the node_specific_loop */
uint32_t polling_period = DEFAULT_POLLING_PERIOD;
long last_iteration = 0;

/* Ensure a minimum publish rate */
unsigned long last_publish[NUM_PUBLISH_TOPICS];

/* Communication settings */
WiFiClient espClient;
PubSubClient client(espClient);

/* Temperature and humidity sensor */
DHTesp temperature_sensor;
/* Light sensor */
BH1750 lightMeter;


/* Setup */
void setup()
{
#if 1
  Serial.begin(115200);
#endif

  /* Attach door interrupt callback */
  attachInterrupt(digitalPinToInterrupt(DOOR_SENSOR_PIN), door_sensor_isr, RISING);

  setup_wifi();
  setup_hardware();
  setup_mqtt();

  /* Ensure that the circular buffer starts at the index 0 */
  temperature_buffer[BUFFER_SIZE] = 0;
  humidity_buffer[BUFFER_SIZE] = 0;

  /* Publish a boot event for error tracking and debugging */
  bool publish_succeeded = false;
  while(!publish_succeeded)
  {
    publish_succeeded = client.publish("attic_node/boot", "");
    if(!publish_succeeded) network_loop();
    yield();
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
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  wifi_set_sleep_type(LIGHT_SLEEP_T);

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

  /* In-built LED */
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(LED_BUILTIN, HIGH); //This means led OFF

 /* Setup pins */
 temperature_sensor.setup(TEMPERATURE_SENSOR_PIN, DHTesp::DHT11);

 /* Door sensor pin as PULLUP (avoid undefined state) */
 pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);

 /* Initialize I2C bus */
  Wire.begin(LIGHT_SENSOR_PIN_1,LIGHT_SENSOR_PIN_2);
  lightMeter.begin();

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
    if (client.connect("AtticClient"))
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
/* The network stuff works atomically */
void network_loop()
{
#if DEBUG_ENABLED
  unsigned long current_time = millis();
#endif

  /* Disable interrupts */
  noInterrupts();

  /* MQTT loop */
  if (!client.connected()) reconnect();
  client.loop();

  /* Enable interrupts */
  interrupts();

#if DEBUG_ENABLED
  current_time = millis() - current_time;
  Serial.print("Duration network_loop: ");
  Serial.println(current_time);
 #endif
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

/* Average an array. Remove oldest element. Add newest element. Filter out NaN values */
float smoothing_filter(float* measurement_buffer, float measurement)
{
  float avg = 0;
  uint8_t measurement_counter = 0;

  //Get index of the oldest element, stored in position 0 of the buffer
  uint8_t oldest_idx = (uint8_t)(measurement_buffer[BUFFER_SIZE]);

  //Update index of the oldest element
  uint8_t next_idx;
  if( (oldest_idx+1) >= BUFFER_SIZE ) next_idx = 0;
  else next_idx = oldest_idx + 1;
  measurement_buffer[BUFFER_SIZE] = next_idx;

  //Replace oldest element by newest in the FIFO
  measurement_buffer[oldest_idx] = measurement;

  //Average the buffer skipping NaN values, unless ALL are NaNs
  for(uint8_t i=0; i<BUFFER_SIZE; i++)
  {
    float val = measurement_buffer[i];
    if(!isnan(val) && (val != 0))
    {
      avg += val;
      measurement_counter++;
    }
  }

  if(measurement_counter == 0) avg = NAN;
  else avg /= measurement_counter;

  //Return averaged value
#if DEBUG_ENABLED
  Serial.print("Averaged value: ");
  Serial.println(avg);
  Serial.print("Filtered values: ");
  Serial.println(measurement_counter);
#endif
  return avg;
}

/* Get the amount of light */
void get_light_amount()
{
  light_amount = lightMeter.readLightLevel();

#if DEBUG_ENABLED
  Serial.print("Light amount: ");
  Serial.println(light_amount);
#endif
}

/* Get temperature and humidity */
void get_temperature()
{
  /* Instant measurement */
  float tmp_humidity = temperature_sensor.getHumidity();
  float tmp_temperature = temperature_sensor.getTemperature();

  /* Filter and average */
  temperature = smoothing_filter(temperature_buffer,tmp_temperature);
  humidity = smoothing_filter(humidity_buffer,tmp_humidity);

#if DEBUG_ENABLED
  Serial.print("Instant temperature: ");
  Serial.println(tmp_temperature);
  Serial.print("Averaged temperature: ");
  Serial.println(temperature);
  Serial.print("Instant humidity: ");
  Serial.println(tmp_humidity);
  Serial.print("Averaged humidity: ");
  Serial.println(humidity);
#endif
}

/* Interrupt service request for door sensor */
void door_sensor_isr()
{
  /* Force the execution of the node_specific_loop in the next loop() iteration */
  /* Same result as polling but with maximum responsibity */
  last_iteration = 0;
  #if DEBUG_ENABLED
  Serial.println("Door detected via interrupt");
  #endif
}

/* Update the polling period depending on the status of the system */
/* If the average measurement after filtering is NaN, reduce the polling period */
/* This feature is currently not used */
void update_polling_period()
{
  if(isnan(temperature) || isnan(humidity))
  {
    #if DEBUG_ENABLED
    Serial.println("Polling period in WARNING mode");
    #endif
    polling_period = WARNING_POLLING_PERIOD;
  }
  else
  {
    #if DEBUG_ENABLED
    Serial.println("Polling period in DEFAULT mode");
    #endif
    polling_period = DEFAULT_POLLING_PERIOD;
  }
}

/* Poll all the sensors and store their value in local variables */
void poll_sensors()
{
  /* Get temperature and humidity */
  get_temperature();

  /* Get the amount of light (only if request received) */
  get_light_amount();

  /* Get door status */
  get_door_status();

  /* Update polling period */
  //update_polling_period();
}

/* Get the status of the door */
void get_door_status()
{
  /* Poll actual sensor */
  int polled_value = digitalRead(DOOR_SENSOR_PIN);

  /* Save old status for change detection */
  door_status_open_old = door_status_open;

  if(polled_value) door_status_open = true;
  else door_status_open = false;

  #if DEBUG_ENABLED
  Serial.print("Polled value: ");
  Serial.println(polled_value);
  #endif
}

/* Node-specific logic */
/* Empty. Keep for future improvements/features */
void node_logic()
{

}

/* Publish sensor information */
void publish_status()
{
#if DEBUG_ENABLED
  Serial.println("Publishing messages");
#endif

  /* Compute time since last publish for each topic */
  unsigned long time_since_publish;
  bool publish_this[NUM_PUBLISH_TOPICS];
  for(uint8_t i=0; i<NUM_PUBLISH_TOPICS; i++)
  {
    time_since_publish = millis() - last_publish[i];
    if(time_since_publish > MINIMUM_PUBLISH_RATE) publish_this[i] = true;
    else publish_this[i] = false;
  }

  /* Publish temperature*/
  if( (abs(temperature - temperature_old) > abs(MEASUREMENT_DELTA*temperature)) || publish_this[0] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in temperature larger than 1%. Publishing temperature");
    #endif
    client.publish("attic_node/temperature", String(temperature).c_str() );
    temperature_old = temperature;
    last_publish[0] = millis();
  }

  /* Publish humidity */
  if( (abs(humidity - humidity_old) > abs(MEASUREMENT_DELTA*humidity)) || publish_this[1] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in humidity larger than 1%. Publishing humidity");
    #endif
    client.publish("attic_node/humidity", String(humidity).c_str() );
    humidity_old = humidity;
    last_publish[1] = millis();
  }

  /* Publish light amount */
  if( (abs(light_amount - light_amount_old) > abs(MEASUREMENT_DELTA*light_amount)) || publish_this[2] )
  {
    #if DEBUG_ENABLED
    Serial.println("Detected change in light amount larger than 1%. Publishing light amount");
    #endif
    client.publish("attic_node/light", String(light_amount).c_str() );
    light_amount_old = light_amount;
    last_publish[2] = millis();
  }

  /* Publish door open information with status update */
  if(door_status_open != door_status_open_old)
  {
#if DEBUG_ENABLED
    Serial.print("Change in door status: ");
    Serial.println(door_status_open);
#endif

    /* Ensure that the critical topics get published */
    bool publish_succeeded = false;
    while(!publish_succeeded)
    {
      publish_succeeded = client.publish("attic_node/door_status", String(door_status_open).c_str());
      if(!publish_succeeded) network_loop();
      yield();
    }
  }
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

  /* Enable interrupts */
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

  /* Enter the node_specific_loop with the specified frequency */
  long now = millis();
  if( (now - last_iteration) > polling_period )
  {
    /* Node specific loop */
    node_specific_loop();

    /* Update last iteration time */
    last_iteration = now;
  }

  /* Enter light sleep - reduce power consumption */
  //delay(SLEEP_TIME);
}
