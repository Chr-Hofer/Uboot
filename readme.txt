//

//TODO:
  //Pressuresensor:
    //Linearisierung implementieren
    //Senden der Werte im Sensor-Protokoll
    //Tauchtiefe limitieren

  //Errorprotokoll:
    //Error für Wassersensoren
    //Error für Pressuresensor
    //Error für Batterielevel

  //Webserver/Videostream

  //Schaltpläne entwerfen
  
  //Batteriemessung implementieren
  
  //? Anomalie bei Transmission beheben ?

//


//

//Code ist in einzelne Module geteilt
//Was parallel auf verschiedenen Kernen läuft, ist nicht in einer Codedatei; betrifft eigentlich eh nur die Bildverarbeitung (core1) und die Steuerverarbeitung (core0) des ESP32 in der Boje
//füge ich später zusammen, wenn wir das müssen, da das Debugging sonst unnötig schwer wird

//
