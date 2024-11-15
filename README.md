# lsm9ds1
Il datasheet dell'imu può essere reperito al seguente link
[LSM9DS1 Datasheet - STMicroelectronics](https://www.st.com/en/mems-and-sensors/lsm9ds1.html)

Inoltre l'imu in nostro possesso è montata sulla breakout board di Sparkfun e possono essere scaricate le
rispettive librerie per Arduino. Nello specifico utilizzeremo la comunicazione I2C 


15/11/24 PRIMO COMMIT
Il codice al primo commit consta di una cartella dedicata ai test su Arduino, in cui andremo a leggere i dati raw dai sensori, senza computare
effettivamente l'imbardata. Inoltre la lettura in modalità burst non funziona, restituendo "0" per gli assi del giroscopio e per adesso non
è committata.

Codice non burst:
La comunicazione I2C è chiaramente spiegata nel datasheet.
Gli indirizzi di slave I2C
sono 0x6B (accelerometro e giroscopio) e 0x1E (magnetometro) dove il bit SA0 è alto standard (da come è montata l'imu sulla board Sparkfun).
Con le impostazioni sui rispettivi registri di controllo, leggeremo andando a richiedere ogni volta i 3 assi per accelerometro, giroscopio, magnetometro.

Codice burst (e auto-increment)
Andando a settare appositamente i registri di controllo, andiamo a chiedere una sola volta i dati che ci verranno dati a blocchi di 16 bit per asse automaticamente 
(auto increment senza chiedere il registro basso e alto) e ciclicamente (burst) giroscopio e accelerometro, come a pagina 21 del datasheet.
Tuttavia questa parte non funziona e non è committata al primo commit 





