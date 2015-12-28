
#include "types.h"

void parseMsg (int largo) {
  
  byte fromId = msgBuffer [0];
  byte toId = msgBuffer [1];
  byte opcode = msgBuffer [2];
  
  if (toId == MY_ID || toId == BROADCAST) {
    handle (fromId, opcode, &msgBuffer[3], largo-3);  
  }
  
}

// opcodes 
#define OP_PING               0              
#define OP_ERROR              1 
#define OP_GETNAME            2
#define OP_LEDS               3
#define OP_GETGYRO            4
#define OP_GETACCEL           5 
#define OP_GETINCLINATION     6
#define OP_GETPID             7
#define OP_SETSETPOINT        8

#define FPN_SIZE  sizeof(FixedPointNumber)

void handle (byte fromId, byte opcode, byte* parameters, int largo) {
  switch (opcode) {
    case OP_PING: { 
      sendMsg (fromId, opcode, NULL, 0);
      break;  
    }
    case OP_GETNAME: { 
      char name[] = "Segduino_MPU6000";
      sendMsg (fromId, opcode, (byte*)name, strlen(name)+1);
      break;  
    }
    case OP_LEDS: { 
      prenderLeds (parameters[0]);
      sendMsg (fromId, opcode, NULL, 0);                 // ACK
      break;  
    }
    case OP_GETGYRO: {                                   // lo manda como fixed point number
      double G[3];
      imu.getGyro(G);    
      FixedPointNumber fpn [3]; 
      for (int axis=0; axis<3; axis++) {
        fpn[axis].setValue (G[axis]);
      } 
      sendMsg (fromId, opcode, (byte*)&fpn, FPN_SIZE*3);
      break;  
    }
    case OP_GETACCEL: {                                  // lo manda como fixed point number
      double A[3];
      imu.getAcc(A);    
      FixedPointNumber fpn [3]; 
      for (int axis=0; axis<3; axis++) {
        fpn[axis].setValue (A[axis]);
      } 
      sendMsg (fromId, opcode, (byte*)&fpn, FPN_SIZE*3);
      break;  
    }
    case OP_GETINCLINATION: {                          // lo manda como fixed point number
      FixedPointNumber fpn [3]; 
      fpn[0].setValue (imu.getAccInclination()); 
      fpn[1].setValue (imu.getGyroInclination());
      fpn[2].setValue (imu.getEstimatedInclination());
      sendMsg (fromId, opcode, (byte*)&fpn, FPN_SIZE*3);
      break;  
    }
    case OP_GETPID: {                                  // lo manda como fixed point number
      FixedPointNumber fpn [3]; 
      fpn[0].setValue (angleSetpoint); 
      fpn[1].setValue (angleInput);
      fpn[2].setValue (angleOutput);
      sendMsg (fromId, opcode, (byte*)&fpn, FPN_SIZE*3);
      break;  
    }
    case OP_SETSETPOINT: {                                    // lo manda como fixed point number
      FixedPointNumber fpn {parameters[0], parameters[1]};
      float value = THETA_CENTER + fpn.getValue();
      if (value>=THETA_MIN && value<=THETA_MAX) {             
        angleSetpoint = value;
        sendMsg (fromId, opcode, NULL, 0);                    // ACK
      } else {
        error (fromId, "set point fuera de rango");
      }
      break;
    }
    default: {
      error (fromId, "unknown opcode");
      break;
    }
  }
}


// general responses

void error (byte destId, char* text) {
  sendMsg (destId, OP_ERROR, (byte*)text, strlen(text)+1);   
}
