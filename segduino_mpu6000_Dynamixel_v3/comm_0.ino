/*
 * protocolo de comunicación basado en el Proyecto Butiá 1.0 (2010)
 * http://www.fing.edu.uy/inco/proyectos/butia/prototipos1.html
 * (c) MINA GROUP + Pablo Gindel
 */

#define MY_ID  200

///////////////////////////////////////////////////////////////////////////////////////////////////////
//                                               COMM                                                //
///////////////////////////////////////////////////////////////////////////////////////////////////////

// nota: SYNC y ESCAPE deben ser ambos > 127
// nota: el largo no puede ser == SYNC ni == ESCAPE (todo lo demás SÍ puede ser == SYNC o == ESCAPE)

#define SYNC            254                  // byte de comienzo de los mensajes seriales
#define ESCAPE          253                  // byte de desambiguación en los mensajes seriales
#define BROADCAST       255                  // broadcast

// estados de la máquina de recepción
enum ESTADO { BUSCAR, LEER_LARGO, LEER_MSG, CHECKSUM }; 

byte msgBuffer [256];        // buffer fijo para todo el sistema de mensajes

/*
 * máquina de estados que recibe y decodifica los mensajes Seriales
 */
void leerSerial () {                   
  
  // variables de control
  static byte largo, cont, checksum;
  static byte anterior = 0;    
  static ESTADO estado = BUSCAR;
 
  while (Serial1.available() > 0) {                                                 // si hay al menos 1 byte en el Serial...
    byte b = Serial1.read();                                                            // ...lo lee
    if (b==SYNC && ((anterior!=ESCAPE && estado==BUSCAR) || anterior<128)) {           // chequea la presencia de un "TRUE SYNC"
      estado = LEER_LARGO; 
    } else {
      switch (estado) {
        case LEER_LARGO:
          largo = b;                                                        // largo no puede ser > 255 
          checksum = 0;                                                     // se prepara para leer el cuerpo del mensaje
          cont = 0;
          estado = LEER_MSG; 
          break;
        case LEER_MSG:
          msgBuffer [cont++] = b;                                          // va llenando el buffer
          checksum += b;                                                    // va efectuando la suma comprobatoria
          if (cont == largo) {                                              // ...hasta que termina el cuerpo del mensaje
            checksum &= 127;                                                // "recorta" el checksum
            estado = CHECKSUM;
          }
          break;
        case CHECKSUM:
          if (b == checksum) {                                              // comprueba el checksum                                    
            // saca los ESCAPEs y llama a parse_msg
            // el parámetro es el nuevo largo
            parseMsg (unescape (largo));
          } else {
            // TODO: handle checksum error
            // error ("checksum");
          }
          estado = BUSCAR;                                                  // estado inicial
          break;  
      }
    }
    anterior = b;                                                           // se prepara para procesar el siguiente byte
  } 
}

/*
 * formatea, encapsula y manda los mensajes al Serial (largo es el largo de la data solamente)
 */
void sendMsg (byte destId, byte opcode, byte *data, int largo) {      
  
  // primera parte: formatea el mensaje
  byte mensaje [largo+3];
  memcpy (mensaje+3, data, largo);
  largo += 3;
  mensaje [0] = MY_ID;
  mensaje [1] = destId;
  mensaje [2] = opcode;
    
  // segunda parte: escapea  
  byte salida [largo*2];                                         // crea un array suficientemente largo para contener el string de salida
  byte pos = 0;                                                  // posición en el array de salida
  for (byte i=0; i<largo; i++) {                                 // recorre el array de entrada
    if (mensaje[i] != ESCAPE && mensaje[i] != SYNC) {            // si no es un byte especial...
      salida[pos++] = mensaje[i];                                // ...lo mete como viene en el array de salida e incrementa posición
    } else {                                                     // de lo contrario...
      salida[pos++] = ESCAPE;                                    // mete primero un ESCAPE e incrementa
      salida[pos++] = mensaje[i];                                // y luego mete el byte e incrementa
    }
  }   
  
  // tercera parte: encapsula y manda al Serial
  Serial1.write (SYNC);                                  // escribe un SYNC
  Serial1.write (pos);                                   // escribe el largo
  byte checksum = 0;                                    // inicializa el acumulador de suma comprobatoria
  for (byte i=0; i<pos; i++) {                          // recorre el array de salida
    Serial1.write (salida[i]);                           // escribe el byte
    checksum += salida[i];                              // suma
  }
  Serial1.write (checksum & 127);                        // escribe la suma comprobatoria recortada
 
}

/*
 * saca los ESCAPES y devuelve el mensaje original
 */
byte unescape (byte largo) {                         
  byte i=0, pos=0;                                       // inicializa posición del array de entrada y salida (que son el mismo)
  do {
    if (msgBuffer [i] == ESCAPE) {i++;}                 // si encuentra un ESCAPE, lo saltea
    msgBuffer [pos++] = msgBuffer [i++];               // copia el byte correcto a la nueva posición
  } while (i < largo);
  return pos;                                            // devuelve el nuevo largo
}



