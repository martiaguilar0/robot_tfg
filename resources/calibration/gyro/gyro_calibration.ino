#include <Wire.h>
#include "ICM_20948.h" 

ICM_20948_I2C myICM; 

void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("--- Sistema Iniciado ---"); 

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println("Buscando IMU...");
  myICM.begin(Wire, 1); 

  if (myICM.status != ICM_20948_Stat_Ok) {
    Serial.println("Fallo en 0x69. Reintentando en 0x68 (AD0 a GND)...");
    myICM.begin(Wire, 0); 
    
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.print("Error final: ");
      Serial.println(myICM.statusString());
      while (1); 
    }
  }

  Serial.println("¡IMU Conectada!");

  ICM_20948_fss_t myFSS;
  myFSS.g = dps250; 
  myICM.setFullScale(ICM_20948_Internal_Gyr, myFSS);

  Serial.println("sample,gx,gy,gz");
}

void loop() {
  static int count = 0;
  if (count < 10000) {
    if (myICM.dataReady()) {
      myICM.getAGMT(); 
      Serial.print(count); Serial.print(",");
      Serial.print(myICM.gyrX(), 4); Serial.print(",");
      Serial.print(myICM.gyrY(), 4); Serial.print(",");
      Serial.println(myICM.gyrZ(), 4);
      count++;
      delay(10); 
    }
  }
}