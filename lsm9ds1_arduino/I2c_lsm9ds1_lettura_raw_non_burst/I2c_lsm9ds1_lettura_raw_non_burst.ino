#include <Wire.h>

// Indirizzi I2C
#define LSM9DS1_AG_ADDR 0x6B  // Indirizzo accelerometro e giroscopio
#define LSM9DS1_MAG_ADDR 0x1E // Indirizzo magnetometro

// Indirizzi dei registri
#define OUT_X_L_G 0x18       // Giroscopio asse X byte basso
#define OUT_X_L_XL 0x28      // Accelerometro asse X byte basso
#define OUT_X_L_M 0x28       // Magnetometro asse X byte basso

// Registri di controllo
#define CTRL_REG1_G 0x10     // Registro di controllo giroscopio
#define CTRL_REG6_XL 0x20    // Registro di controllo accelerometro
#define CTRL_REG1_M 0x20     // Registro di controllo magnetometro 1
#define CTRL_REG2_M 0x21     // Registro di controllo magnetometro 2
#define CTRL_REG3_M 0x22     // Registro di controllo magnetometro 3

void setup() {
    Wire.begin();
    Serial.begin(9600);

    // Inizializza le impostazioni LSM9DS1
    initializeLSM9DS1();
}

void loop() {
    // Leggi i dati dall'accelerometro
    int16_t accelX = read16BitData(LSM9DS1_AG_ADDR, OUT_X_L_XL);
    int16_t accelY = read16BitData(LSM9DS1_AG_ADDR, OUT_X_L_XL + 2);
    int16_t accelZ = read16BitData(LSM9DS1_AG_ADDR, OUT_X_L_XL + 4);

    // Leggi i dati dal giroscopio
    int16_t gyroX = read16BitData(LSM9DS1_AG_ADDR, OUT_X_L_G);
    int16_t gyroY = read16BitData(LSM9DS1_AG_ADDR, OUT_X_L_G + 2);
    int16_t gyroZ = read16BitData(LSM9DS1_AG_ADDR, OUT_X_L_G + 4);

    // Leggi i dati dal magnetometro
    int16_t magX = read16BitData(LSM9DS1_MAG_ADDR, OUT_X_L_M);
    int16_t magY = read16BitData(LSM9DS1_MAG_ADDR, OUT_X_L_M + 2);
    int16_t magZ = read16BitData(LSM9DS1_MAG_ADDR, OUT_X_L_M + 4);

    // Stampa i dati sul monitor seriale
    Serial.print("Accel: ");
    Serial.print("X = "); Serial.print(accelX);
    Serial.print(", Y = "); Serial.print(accelY);
    Serial.print(", Z = "); Serial.println(accelZ);

    Serial.print("Gyro: ");
    Serial.print("X = "); Serial.print(gyroX);
    Serial.print(", Y = "); Serial.print(gyroY);
    Serial.print(", Z = "); Serial.println(gyroZ);

    Serial.print("Mag: ");
    Serial.print("X = "); Serial.print(magX);
    Serial.print(", Y = "); Serial.print(magY);
    Serial.print(", Z = "); Serial.println(magZ);

    delay(500); // Ritardo per leggibilità
}

// Funzione per inizializzare il sensore LSM9DS1 con le impostazioni corrette dei registri
void initializeLSM9DS1() {
    // Imposta il giroscopio su 245 dps a scala completa, 119 Hz ODR
    writeRegister(LSM9DS1_AG_ADDR, CTRL_REG1_G, 0x50); // ODR_G = 119 Hz, FS_G = 245 dps

    // Imposta l'accelerometro su ±2 g a scala completa, 119 Hz ODR
    writeRegister(LSM9DS1_AG_ADDR, CTRL_REG6_XL, 0x20); // ODR_XL = 119 Hz, FS_XL = ±2g

    // Imposta il magnetometro su modalità continua, bassa potenza, 10 Hz ODR
    writeRegister(LSM9DS1_MAG_ADDR, CTRL_REG1_M, 0x10); // DO_M = 10 Hz, OM = bassa potenza
    writeRegister(LSM9DS1_MAG_ADDR, CTRL_REG2_M, 0x00); // FS_M = ±4 gauss (predefinito)
    writeRegister(LSM9DS1_MAG_ADDR, CTRL_REG3_M, 0x00); // Modalità di conversione continua
}

// Funzione per scrivere un byte in un registro
void writeRegister(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.write(data);
    Wire.endTransmission();
}

// Funzione per leggere i dati a 16 bit da registri consecutivi
int16_t read16BitData(uint8_t deviceAddress, uint8_t regAddress) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.endTransmission(false); // Invia un restart invece di stop
    Wire.requestFrom(deviceAddress, (uint8_t)2); // Richiedi 2 byte

    int16_t data = 0;
    if (Wire.available() >= 2) {
        data = Wire.read();         // Byte basso
        data |= Wire.read() << 8;   // Byte alto
    }
    return data;
}
