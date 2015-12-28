/*
  ax12.cpp - arbotiX Library for AX-12 Servos
  Copyright (c) 2008,2009 Michael E. Ferguson.  All right reserved.
  Modificada el 15/11/09 por Pablo Gindel.
  versión 2.0 - 10/02/10
  versión 2.1 - 27/06/10
  versión 2.2 - 19/10/10
  versión 2.3 - 30/12/10
	versión 2.31 - 2/3/11
*/

/**** ATENCION: si se cambia el orden de los #includes, sigue compilando pero no funciona! ****/

#include "ax12.h"
#include "arduino.h"           // we need this for the serial port defines. 
#include <string.h>
#include <avr/interrupt.h>


#define DIRECTION_PIN   65


// some utils definition
int makeInt (byte *dir, byte reglength);
byte lengthRead (byte registr);
byte lengthWrite (byte registr);

/******************************************************************************
 * Hardware Serial Level, this uses the same stuff as Serial, therefore
 *  you should not use the Arduino Serial library.
 ******************************************************************************/

/** helper functions to emulate half-duplex */
void AX12::setTX () {
    digitalWrite (DIRECTION_PIN, HIGH);
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
    bitClear(UCSR0B, RXCIE0);    // deshabilita la interrupción de recepción
    bitClear(UCSR0B, RXEN0);     // deshabilila la recepción
    bitSet(UCSR0B, TXEN0);       // habilita la trasmisión
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)   
    bitClear(UCSR2B, RXCIE2);    // deshabilita la interrupción de recepción
    bitClear(UCSR2B, RXEN2);     // deshabilila la recepción
    bitSet(UCSR2B, TXEN2);       // habilita la trasmisión
#elif defined (__AVR_ATmega8__)
    bitClear(UCSRB, RXCIE);    // deshabilita la interrupción de recepción
    bitClear(UCSRB, RXEN);     // deshabilila la recepción
    bitSet(UCSRB, TXEN);       // habilita la trasmisión
#endif    
}

void AX12::setRX () {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
    //bitClear(TIMSK0, TOIE0);   // deshabilita la interrupción del timer0 (nota: esto es sólo para entornos Arduino)
    bitClear(UCSR0B, TXEN0);   // deshabilita la trasmisión
    bitSet(UCSR0B, RXEN0);     // habilita la recepción
    bitSet(UCSR0B, RXCIE0);    // habilita la interrupción de recepción
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
    //bitClear(TIMSK0, TOIE0);   // deshabilita la interrupción del timer0 (nota: esto es sólo para entornos Arduino)
    // linea dudosa
    while (bit_is_clear(UCSR2A, UDRE2)) ;
    // fin linea dudosa
    bitClear(UCSR2B, TXEN2);   // deshabilita la trasmisión
    bitSet(UCSR2B, RXEN2);     // habilita la recepción
    bitSet(UCSR2B, RXCIE2);    // habilita la interrupción de recepción
#elif defined (__AVR_ATmega8__)
    //bitClear(TIMSK0, TOIE0); // deshabilita la interrupción del timer0 (nota: esto es sólo para entornos Arduino)
    bitClear(UCSRB, TXEN);   // deshabilita la trasmisión
    bitSet(UCSRB, RXEN);     // habilita la recepción
    bitSet(UCSRB, RXCIE);    // habilita la interrupción de recepción 
#endif    
    delayMicroseconds (150);                                // esto es reconrtra dudoso, es un parche para el MAX485
    digitalWrite (DIRECTION_PIN, LOW);
    ax_rx_Pointer = 0;         // resetea el puntero del buffer
}

void AX12::setNone () {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
    bitClear(UCSR0B, RXCIE0);    // deshabilita la interrupción de recepción
    bitClear(UCSR0B, RXEN0);     // deshabilila la recepción
    bitClear(UCSR0B, TXEN0);     // deshabilita la trasmisión
    //bitSet(TIMSK0, TOIE0);          // rehabilita la interrupción del timer0 (nota: esto es sólo para entornos Arduino)
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
    bitClear(UCSR2B, RXCIE2);    // deshabilita la interrupción de recepción
    bitClear(UCSR2B, RXEN2);     // deshabilila la recepción
    bitClear(UCSR2B, TXEN2);     // deshabilita la trasmisión
    //bitSet(TIMSK0, TOIE0);          // rehabilita la interrupción del timer0 (nota: esto es sólo para entornos Arduino)
#elif defined (__AVR_ATmega8__)
    bitClear(UCSRB, RXCIE);    // deshabilita la interrupción de recepción
    bitClear(UCSRB, RXEN);     // deshabilila la recepción
    bitClear(UCSRB, TXEN);     // deshabilita la trasmisión 
    //bitSet(TIMSK0, TOIE0);        // rehabilita la interrupción del timer0 (nota: esto es sólo para entornos Arduino)
#endif    
}

/** Sends a character out the serial port */
byte AX12::writeByte (byte data) {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
    while (bit_is_clear(UCSR0A, UDRE0));    // espera que el micro esté pronto para trasmitir
    UDR0 = data;                            // escribe el byte a trasmitir
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
    while (bit_is_clear(UCSR2A, UDRE2));    // espera que el micro esté pronto para trasmitir
    UDR2 = data;                            // escribe el byte a trasmitir
#elif defined (__AVR_ATmega8__)
    while (bit_is_clear(UCSRA, UDRE));      // espera que el micro esté pronto para trasmitir
    UDR = data;                             // escribe el byte a trasmitir  
#endif     
    return data;
}

/** We have a one-way receive buffer, which is reset after each packet is receieved.
    A wrap-around buffer does not appear to be fast enough to catch all bytes at 1Mbps. */
volatile byte AX12::ax_rx_buffer[AX12_BUFFER_SIZE];
volatile byte AX12::ax_rx_Pointer;

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
ISR (USART_RX_vect) {
    AX12::ax_rx_buffer[(AX12::ax_rx_Pointer++)] = UDR0;    // esta es la rutina de interrupción de recepción
}                                                          
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
ISR (USART2_RX_vect) {
    AX12::ax_rx_buffer[(AX12::ax_rx_Pointer++)] = UDR2;    // esta es la rutina de interrupción de recepción
}                                                          // lo que hace es meter el byte recibido en el buffer
#elif defined (__AVR_ATmega8__)
SIGNAL (SIG_UART_RECV) {
    AX12::ax_rx_buffer[(AX12::ax_rx_Pointer++)] = UDR;    // esta es la rutina de interrupción de recepción
}                                                         // lo que hace es meter el byte recibido en el buffer
#endif    

/** initializes serial transmit at baud, 8-N-1 */
// nota: el AX12 internamente usa el Double Speed Operation (U2Xn=1). Ciertos baudrates no coinciden si no se usa este seteo
void AX12::init (long baud) {
    unsigned int reg = F_CPU / (baud * 8) - 1;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
    bitSet (UCSR0A, U2X0); 
    UBRR0H = reg >> 8;                 // setea la velocidad del USART
    UBRR0L = reg & 0xFF; 
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
    bitSet (UCSR2A, U2X2); 
    UBRR2H = reg >> 8;
    UBRR2L = reg & 0xFF;               // setea la velocidad del USART
#elif defined (__AVR_ATmega8__)
    bitSet (UCSRA, U2X); 
    UBRRH = reg >> 8;
    UBRRL = reg & 0xFF;                // setea la velocidad del USART
#endif    
    ax_rx_Pointer = 0;
    // RX-24F add-on
    pinMode (DIRECTION_PIN, OUTPUT);
    // deshabilita tanto recepción como trasmisión
    setNone();
}

/******************************************************************************
 * Initialization
 ******************************************************************************/

byte AX12::autoDetect (byte* list_motors, byte num_motors) {                // escanea todos los ID hasta el 253 mandando PINGs
                                                                            // devuelve una lista con los ID que respondieron
    byte counter = 0;      
    byte *data;
    for (byte i=0; i<254; i++) {
      sendPacket (i, 0, AX_PING, data);
      byte index = readPacket ();
      byte id = ax_rx_buffer [index];
      byte int_error = ax_rx_buffer [index+1];
      if (int_error==0 && id==i) {
        list_motors[counter++] = i;
        if (counter == num_motors) {break;}
      }
    }
    return counter;
}

/** constructors */
AX12::AX12 (long baud, byte motor_id, bool inv) {
    id = motor_id;
    inverse = inv;
    SRL = RETURN_ALL;
    init (baud);
}

AX12::AX12 (byte motor_id, bool inv) {
    id = motor_id;
    inverse = inv;
    SRL = RETURN_ALL;
}

AX12::AX12 (long baud, byte motor_id) {
    id = motor_id;
    inverse = false;
    SRL = RETURN_ALL;
    init (baud);
}

AX12::AX12 (byte motor_id) {
    id = motor_id;
    inverse = false;
    SRL = RETURN_ALL;
}

AX12::AX12 () {
    id = BROADCAST_ID;
    inverse = false;
    SRL = RETURN_NONE;
}

/* para mandar mensajes broadcast, definir un motor broadcast, de la siguiente manera:
   AX12 broadcast = AX12();
   Nota: al definir un motor de esta manera y luego asignarle un id, tener en cuenta que 
   el SRL (status return level) va a quedar seteado en RETURN_NONE.
*/

/******************************************************************************
 * Packet Level
 ******************************************************************************/

/** send instruction packet */
void AX12::sendPacket (byte _id, byte datalength, byte instruction, byte* data) {
    byte checksum = 0;
    setTX();
    writeByte (0xFF);
    writeByte (0xFF);
    checksum += writeByte (_id);
    checksum += writeByte (datalength + 2);
    checksum += writeByte (instruction);
    for (byte f=0; f<datalength; f++) {     // data = parámetros
      checksum += writeByte (data[f]);
    }
    // checksum =
    writeByte (~checksum);
    setRX();
}

/** read status packet
/** retorna la posición en el buffer a partir de la cual se lee lo siguiente:
    posición [0] = status_id
    posición [1] = internal error (0 = OK)
    posición [2] = status_error
    posición [3,4,...] = status_data        */
byte AX12::readPacket () {
    unsigned long ulCounter;
    byte timeout, error, status_length, checksum, offset, bcount;
    // primero espera que llegue toda la data
    offset = 0; timeout = 0; bcount = 0;
    while (bcount < 13) {       // 13 es el largo máximo que puede tener un packet
        ulCounter = 0;
        while ((bcount + offset) == ax_rx_Pointer) {
            if (ulCounter++ > 1200L) {                   // acá hay 2 errores claros, 1) faltan #defines, 2) el timeout no es relativo al baudrate
                timeout = 1;
                break;
            }
        }
        if (timeout) break;
        if ((bcount == 0) && (ax_rx_buffer[offset] != 0xff)) offset++;
        else bcount++;
    }
    setNone();
    // ahora decodifica el packet
    // corrección de cabecera
    error = 0;                                             // código interno de error
    do {
        error++;
        offset++;
        bcount--;
    } while (ax_rx_buffer[offset] == 255);
    if (error > 1) error = 0;                               // prueba de cabecera
    // offset = primer byte del mensaje (sin cabecera)
    // bcount = largo del mensaje leido (sin cabecera)
    status_length = 2 + ax_rx_buffer[offset+1];            // largo del mensaje decodificado
    if (bcount != status_length) error+=2;                 // prueba de coherencia de data
    checksum = 0;                                          // cálculo de checksum
    for (byte f=0; f<status_length; f++)
        checksum += ax_rx_buffer[offset+f];
    if (checksum != 255) error+=4;                          // prueba de checksum
    ax_rx_buffer[offset+1] = error;
    return offset;   
}

/******************************************************************************
 * Instruction Level
 ******************************************************************************/

/** ping */
int AX12::ping () {
  byte* data;
  sendPacket (id, 0, AX_PING, data);
  return returnData (RETURN_NONE).error;
}

/** reset */
int AX12::reset () {
  byte* data;
  sendPacket (id, 0, RESET, data);
  return returnData (RETURN_ALL).error;
}

/** action */
int AX12::action () {
  byte *data;
  sendPacket (id, 0, ACTION, data);
  return returnData (RETURN_ALL).error;
}

/** read data */
AX12data AX12::readData (byte start, byte length) {
  byte data [2];
  data [0] = start; data [1] = length;
  sendPacket (id, 2, READ_DATA, data);
  return returnData (RETURN_READ);
}

/** write data + reg write */
// seteando a "true" el parámetro adicional (isReg) se transforma en un reg write
int AX12::writeData (byte start, byte length, byte* values, bool isReg) {
    byte data [length+1];
    data [0] = start; 
    memcpy (&data[1], values, length);
    if (isReg) {
      sendPacket (id, length+1, REG_WRITE, data);
    } else {
      sendPacket (id, length+1, WRITE_DATA, data);
    }
    int error = returnData (RETURN_ALL).error;
    if (start < 23) {delayMicroseconds (5000);}       // si la operación de escritura es en la EEPROM, este delay previene el embotellamiento
                                                   // (las operaciones en la EEPROM no suelen ser real-time)
    return error;
}

/** sync write */
void AX12::syncWrite (byte start, byte length, byte targetlength, byte* targets, byte** valuess) {
    byte rowlength = length + 1;
    byte superlength = rowlength*targetlength + 2; 
    byte data [superlength];
    data [0] = start; 
    data [1] = length;
    byte index = 2;
    for (byte f=0; f<targetlength; f++) {
      data [index++] = targets[f];                 // pongo el ID
      memcpy (&data[index], valuess[f], length);   // copio los valores
      index += length;
    }
    sendPacket (BROADCAST_ID, superlength, SYNC_WRITE, data);
    setNone();   
}


/******************************************************************************
 * Register Level
 ******************************************************************************/

/** "intelligent" read data */
AX12info AX12::readInfo (byte registr) {
    byte reglength = lengthRead (registr);
    AX12info returninfo;
    returninfo.error = -2;
    if (reglength == 0) {return returninfo;}
    AX12data returndata = readData (registr, reglength);
    returninfo.error = returndata.error;
    returninfo.value = makeInt (returndata.data, reglength);    
    processValue (registr, &returninfo.value);
    return returninfo;
}

/** "intelligent" write data + reg write */
// seteando a "true" el parámetro adicional se transforma en un reg write
int AX12::writeInfo (byte registr, int value, bool isReg) {
    byte reglength = lengthWrite (registr);
    if (reglength==0) {return -2;}
    processValue (registr, &value);
    byte values [reglength];
    values [0] = lowByte(value);
    if (reglength > 1) {values[1] = highByte(value);}
    return writeData (registr, reglength, values, isReg);
}

/** "intelligent" sync write */
void AX12::syncInfo (byte registr, byte targetlength, byte* targets, int* values) {
    byte reglength = lengthWrite (registr);
    if (reglength==0) {return;}
    byte valuess [targetlength][reglength];
    byte * pointers [targetlength];
    for (byte f=0; f<targetlength; f++) {
      valuess [f][0] = lowByte(values[f]);
      if (reglength > 1) {valuess[f][1] = highByte(values[f]);}
      pointers[f] = &valuess[f][0];
    }    
    //nota: la sync write no respeta la propiedad "inverse"
    syncWrite (registr, reglength, targetlength, targets, pointers);
}


/******************************************************************************
 * Macro Level
 ******************************************************************************/

void AX12::setEndlessTurnMode (bool endless) {           // prende o apaga el modo "endless turn"
    writeInfo (CW_ANGLE_LIMIT, 0);
    if (endless) {
      writeInfo (CCW_ANGLE_LIMIT, 0);
    } else {
      writeInfo (CCW_ANGLE_LIMIT, 4095);
    }
}

void AX12::endlessTurn (int velocidad) {                    // setea la velocidad, en el modo "endless turn"
    bool direccion = sign2bin (velocidad);
    writeInfo (MOVING_SPEED, abs(velocidad)|((direccion^inverse)<<10));
}

int AX12::presentPSL (int* PSL) {                                // lee position, speed & load de una sola vez
    AX12data data = readData (PRESENT_POSITION, 6);
    for (byte f=0; f<3; f++) {
      PSL[f] = makeInt (&data.data[2*f], 2);  
      processValue (PRESENT_POSITION + 2*f, &PSL[f]);
    }
    return data.error;
}

// nota: si no coincide el SRL declarado con el del motor, los mensajes de respuesta son malinterpretados
void AX12::setSRL (byte _srl) {
  SRL = _srl;
  writeInfo (STATUS_RETURN_LEVEL, SRL);
}

byte AX12::changeID (byte newID) {
  if (newID > 253) {return id;}
  writeInfo (ID, newID);
  id = newID;
  return id;
}
 
int AX12::setPosVel (int pos, int vel) {
    processValue (GOAL_POSITION, &pos);
    byte values [4];
    values [0] = lowByte(pos);
    values[1] = highByte(pos);
    values [2] = lowByte(vel);
    values[3] = highByte(vel);
    return writeData (GOAL_POSITION, 4, values);
}

void AX12::setMultiPosVel (byte targetlength, byte* targets, int* posvalues, int* velvalues) {
    byte valuess [targetlength][4];
    byte * pointers [targetlength];
    for (byte f=0; f<targetlength; f++) {
      valuess [f][0] = lowByte(posvalues[f]);
      valuess[f][1] = highByte(posvalues[f]);
      valuess [f][2] = lowByte(velvalues[f]);
      valuess[f][3] = highByte(velvalues[f]);
      pointers[f] = &valuess[f][0];
    }    
    //nota: la sync write no respeta la propiedad "inverse"
    syncWrite (GOAL_POSITION, 4, targetlength, targets, pointers);  
}


/******************************************************************************
 * Misc.
 ******************************************************************************/

int makeInt (byte *dir, byte reglength) {          // transforma 2 bytes en un int (según la lógica AX12)
  if (reglength > 1) {
    return (dir[1] << 8) | dir[0];
  } else {
    return dir[0];
  }
}

byte lengthRead (byte registr) {
    byte reglength = 0;
    switch (registr) {
      case VERSION: case ID: case BAUD_RATE: case RETURN_DELAY_TIME: 
      case LIMIT_TEMPERATURE: case DOWN_LIMIT_VOLTAGE: case UP_LIMIT_VOLTAGE: 
      case STATUS_RETURN_LEVEL: case ALARM_LED: case ALARM_SHUTDOWN: case 19: case TORQUE_ENABLE: case LED: 
      case CW_COMPLIANCE_MARGIN: case CCW_COMPLIANCE_MARGIN: case CW_COMPLIANCE_SLOPE: case CCW_COMPLIANCE_SLOPE: 
      case PRESENT_VOLTAGE: case PRESENT_TEMPERATURE: case REGISTERED_INSTRUCTION: case MOVING: case LOCK: reglength = 1; break;
      case MODEL_NUMBER: case CW_ANGLE_LIMIT: case CCW_ANGLE_LIMIT: 
      case MAX_TORQUE: case DOWN_CALIBRATION: case UP_CALIBRATION: 
      case GOAL_POSITION: case MOVING_SPEED: case TORQUE_LIMIT: 
      case PRESENT_POSITION: case PRESENT_SPEED: case PRESENT_LOAD: case PUNCH: reglength = 2; break;
    }
    return reglength;
}

byte lengthWrite (byte registr) {
    byte reglength = 0;
    switch (registr) {
      case ID: case BAUD_RATE: case RETURN_DELAY_TIME: 
      case LIMIT_TEMPERATURE: case DOWN_LIMIT_VOLTAGE: case UP_LIMIT_VOLTAGE: 
      case STATUS_RETURN_LEVEL: case ALARM_LED: case ALARM_SHUTDOWN: case 19: 
      case TORQUE_ENABLE: case LED: case CW_COMPLIANCE_MARGIN: case CCW_COMPLIANCE_MARGIN: 
      case CW_COMPLIANCE_SLOPE: case CCW_COMPLIANCE_SLOPE: case REGISTERED_INSTRUCTION: case LOCK: reglength = 1; break;
      case CW_ANGLE_LIMIT: case CCW_ANGLE_LIMIT: 
      case MAX_TORQUE: case GOAL_POSITION: 
      case MOVING_SPEED: case TORQUE_LIMIT: case PUNCH: reglength = 2; break;
    }
    return reglength;
}

AX12data AX12::returnData (byte _srl) {
  AX12data returndata;
  if (SRL >= _srl) {
    byte index = readPacket ();
    byte status_id = ax_rx_buffer [index];
    byte int_error = ax_rx_buffer [index+1];
    byte status_error = ax_rx_buffer [index+2];
    returndata.error = (int_error<<7) | status_error | ((status_id != id)<<10);       // genera el mensaje de error, combinación de error interno con error ax12
    returndata.data = (byte*) &(ax_rx_buffer [index+3]);
  } else {
    setNone();
    returndata.error = -1;
  }
  return returndata;
}

// Atención: modificada a lo bestia para los MX28
void AX12::processValue (byte registr, int* value) {                           // procesa el valor para la salida segun la propiedad "inverse"
  switch (registr) {
    case PRESENT_POSITION: case GOAL_POSITION:
      if (inverse) {*value = 1023 - *value;} 
      break;
    case PRESENT_SPEED: case PRESENT_LOAD:
      *value = ((*value)&0x03FF) * bin2sign(((*value)>0x03FF)^inverse); 
      break;
  } 
}
