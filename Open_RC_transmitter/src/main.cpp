#include <arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OpenRC.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);
OpenRC openrc;// initialize openRC
Preferences preferences;
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//ESPNOW setup
esp_now_peer_info_t slave;
#define CHANNEL 3
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 1

#define JOYA 33
#define BUTA 13

///////function prototypes

int AdctoAngle (int adc, int cal);
int calibrate (int pin);
void InitESPNow() ;
bool ScanForSlave() ;
bool manageSlave() ;
void deletePeer() ;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void sendData();
void OpenRCloop();
void GetData();
void SaveData();
bool lookforSlave();
void Menu();
void SavedSlave();
///////////////
void setup() { //Setup loop
Serial.begin(115200);

 // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
   
//SETUP GPIO
pinMode(14,INPUT_PULLUP);
pinMode(12,INPUT_PULLUP);
pinMode(19,INPUT_PULLUP);
pinMode(13,INPUT_PULLUP);
  ////////Wifi stuff


//openrc.direction[0] = 1; // Direction change 0 default 1 reverse
//openrc.dualrate[0] = 10; //dualrate only positive 0-180
//openrc.trim[0] = -10; // trim positive or negative of -90 - +90 degree
////////////////
display.clearDisplay(); display.setCursor(0,0);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Open RC");
display.display();



/////////

//SaveData();
GetData();
  mpu6050.begin();
  
Menu ();
WiFi.mode(WIFI_STA);
    // Init ESPNow with a fallback logic
InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
esp_now_register_send_cb(OnDataSent);
//ScanForSlave();
//openrc.Calibration();
SavedSlave();
OpenRCloop();
delay(1000);
display.clearDisplay();
display.display();

}

void loop() {
 sendData();
/*   mpu6050.update();
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ()); */

  display.clearDisplay(); 

  display.setCursor(1,1);
  
  //display.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success          " : "Delivery Fail        ");
    int values[16];  //gettng the array values into local array so we can send it out might be better ways to do it
      for(int x=0; x<=5; x++)
    {
      values[x] = openrc.AdctoAngle(x);//read all ADC convert to degrees and get ready to send
      
    }
     for(int x=6; x<16; x++)
    {
      values[x] = openrc.IOtoAngle(x-6);//
    }

 for(int z=0; z<=16; z++){
display.print(values[z]);
display.print(",");
 }

  display.display();

delay(50);
}




void GetData(){
preferences.begin("my-app", false);
preferences.getBytes("Calibration", &openrc.calibration, 6);
preferences.getBytes("Direction", &openrc.direction, 6);
preferences.getBytes("Dualrate", &openrc.dualrate, 6);
// Close the Preferences
  preferences.end();

}

void SaveData(){

preferences.begin("my-app", false);
preferences.putBytes("Calibration", &openrc.calibration, 6);
preferences.putBytes("Direction", &openrc.direction, 6);
preferences.putBytes("Dualrate", &openrc.dualrate, 6);

// Close the Preferences
  preferences.end();

}

void OpenRCloop(){
 // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
bool isPaired = manageSlave(); 
//bool isPaired = lookforSlave(); //do not add new
    if (isPaired) {
      // pair success or already paired
      // Send data to device
     
      sendData(); //Send Data for all 16 chanels
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
      
    }
  }
  else {
    // No slave found to process
 //ScanForSlave(); // look for one
//oldScanForSlave();
  }

}

////////////////
// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}


void SavedSlave() {
    memset(&slave, 0, sizeof(slave));
preferences.begin("my-app", false);
preferences.getBytes("mac", &slave.peer_addr, 6);
preferences.end();
        
       

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

}

// Scan for slaves in AP mode
bool ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
preferences.begin("my-app", false);
preferences.putBytes("mac", &slave.peer_addr, 6);
preferences.end();

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }
  WiFi.scanDelete();
  if (slaveFound) {
    Serial.println("Slave Found, processing..");
    return 1;
  } else {
    Serial.println("Slave Not Found, trying again.");
    return 0;
  }

  // clean up ram

}

// Check if the slave is already paired with the master.
// If not, pair the slave with master



bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    const esp_now_peer_info_t *peer = &slave;
    const uint8_t *peer_addr = slave.peer_addr;
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(peer);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  const esp_now_peer_info_t *peer = &slave;
  const uint8_t *peer_addr = slave.peer_addr;
  esp_err_t delStatus = esp_now_del_peer(peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}


// send data
void sendData( ) {
  
  uint8_t values[16];  //gettng the array values into local array so we can send it out might be better ways to do it
      for(int x=0; x<=5; x++)
    {
      values[x] = openrc.AdctoAngle(x);//read all ADC convert to degrees and get ready to send
      
    }
     for(int x=6; x<16; x++)
    {
      values[x] = openrc.IOtoAngle(x-6);//
    }

  const uint8_t *peer_addr = slave.peer_addr;
  Serial.print("Sending: "); 
  //esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
  esp_err_t result = esp_now_send(peer_addr, (uint8_t*) &values, sizeof(values));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
/////////////////
 
  
}
//////////////////ugly menu system///////////////////
void Menu (){
  int level =0;
  int step = 0;
 // int substep =0;
  int exit = 1;

  //int okpin = 4;
  display.clearDisplay();
    display.setTextColor(WHITE); //setting the color
    display.setTextSize(1); //set the font size
  while (exit == 1){
     while (level == 0){
      /////////////////
    if (analogRead(JOYA) == 0)  { step++; delay(200);  }
    if (analogRead(JOYA) > 4000){ step--; delay(200);  }

switch (step) {

      case 0:
              display.clearDisplay(); display.setCursor(0,0);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("   OpenRC Start      ");
              display.setTextColor(WHITE);
              display.print("Calibrate            ");
              display.print("Configure            ");
              display.print("Bind                 ");
              display.display();
              if (digitalRead(BUTA) == LOW){
              exit = 0;
              level = 999;
              delay(200);
              }

      break;

      case 1:
              display.clearDisplay(); display.setCursor(0,0);
              display.print("   OpenRC Start      ");
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Calibrate            ");
              display.setTextColor(WHITE);
              display.print("Configure            ");
              display.print("Bind                 ");
              display.display();
              if (digitalRead(BUTA) == LOW){
               level = 1;
               step = 0;
               delay(200);
              }
      break;

      case 2:
              display.clearDisplay(); display.setCursor(0,0);
              display.print("   OpenRC Start      ");
              display.print("Calibrate            ");
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Configure            ");
              display.setTextColor(WHITE);
              display.print("Bind                 ");
              display.display();
      break;

      case 3:
              display.clearDisplay(); display.setCursor(0,0);
              display.print("   OpenRC Start      ");
              display.print("Calibrate            ");
              display.print("Configure            ");
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Bind                 ");
              display.setTextColor(WHITE);
              display.display();
               if (digitalRead(BUTA) == LOW){
               level = 3;
               step = 0;
               delay(200);
              }
      break;
      default:
        step = 0;
       

}
  }
  while (level == 1){
     if (analogRead(JOYA) == 0)  { step++; delay(200);  }
    if (analogRead(JOYA) > 4000){ step--; delay(200);  }
  switch (step) {
      case 0:
              display.clearDisplay(); display.setCursor(0,0);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Start  Calibration   ");
              display.setTextColor(WHITE);
              display.print("Exit                 ");
              display.print("                     ");
                   display.display();      
              if (digitalRead(BUTA) == LOW){
              openrc.Calibration(); //calibrate default readings to 90 degree angle
                  //print 3 calibrations
                  /////
                  mpu6050.calcGyroOffsets(true);
                Serial.println("Calibration");
              Serial.println(openrc.calibration[0]);
              Serial.println(openrc.calibration[1]);
              Serial.println(openrc.calibration[2]);
              Serial.println(openrc.calibration[3]);
              Serial.println("dualrate");
              Serial.println(openrc.dualrate[0]);
              Serial.println(openrc.dualrate[1]);
              Serial.println(openrc.dualrate[2]);
              Serial.println(openrc.dualrate[3]);
                Serial.println("direction");
              Serial.println(openrc.direction[0]);
              Serial.println(openrc.direction[1]);
              Serial.println(openrc.direction[2]);
              Serial.println(openrc.direction[3]);
              SaveData();

             step = 5;
             delay(200);
              }
            
              
      break;

      case 1:
              display.clearDisplay(); display.setCursor(0,0);
              display.print("Start  Calibration   ");
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Exit                 ");
              display.setTextColor(WHITE);
              display.print("                     ");
              display.print("                     ");
              display.display();
              if (digitalRead(BUTA) == LOW){
               level = 0;
               step = 0;
               delay(200);
              }
      break;

      case 5:
              display.clearDisplay(); display.setCursor(0,0);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Start  Calibration   ");
              display.setTextColor(WHITE);
              display.print("Exit                 ");
               display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Calibration Finished ");
              display.setTextColor(WHITE);
              display.display();
      break;
      default:
        step = 0;
       

}
  }
  
   while (level == 3){/// binding
     if (analogRead(JOYA) == 0)  { step++; delay(200);  }
    if (analogRead(JOYA) > 4000){ step--; delay(200);  }
      switch (step) {
      case 0:
              display.clearDisplay(); display.setCursor(0,0);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Bind a new device    ");
              display.setTextColor(WHITE);
              display.print("Exit                 ");
              display.print("                     ");
              display.print("                     ");
              display.display();      
              if (digitalRead(BUTA) == LOW){
              // level = 0;
               //step = 5;
               delay(200);
                // scan and add new device
                bool found = ScanForSlave();
                 if (found == 0)step = 10;
                  if (found == 1)step = 5;
                

              }
      break;
      case 1:
              display.clearDisplay(); display.setCursor(0,0);
              display.print("Bind a new device    ");
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Exit                 ");
              display.setTextColor(WHITE);
              display.print("                     ");
              display.print("                     ");
              display.display();      
              if (digitalRead(BUTA) == LOW){
               level = 0;
               step = 0;
               delay(200);
              }
      break;

      case 5:

              display.clearDisplay(); display.setCursor(0,0);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Bind a new device    ");
              display.setTextColor(WHITE);
              display.print("Exit                 ");
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("New Device paired    ");
              display.print("                     ");
              display.setTextColor(WHITE);
              display.display();      
      break;

       case 10:

              display.clearDisplay(); display.setCursor(0,0);
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("Bind a new device    ");
              display.setTextColor(WHITE);
              display.print("Exit                 ");
              display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
              display.print("No Device Found      ");
              display.print("                     ");
              display.setTextColor(WHITE);
              display.display();      
      break;

      default:
        step = 0;
        }
     }

}
  }