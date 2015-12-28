/* 
 - RingBuffer, recursive filters, fixed point number type & Timer class
 (c) Pablo Gindel
*/

#ifndef __TYPES_H__
#define __TYPES_H__

#include <arduino.h>

///////////////////////////////////////////////
//         ring buffer (con template)        //
///////////////////////////////////////////////

template <typename T, int S>
struct RingBuffer { 
  RingBuffer(): pos(0) {
    for (int i=0; i<S; i++) {
      buffer[i] = 0;
    }
  }
  
  void store (T value) {    
    pos = (pos+S-1) % S;
    buffer [pos] = value;
  }
  
  T get (int index) {
    return buffer [(pos+index)%S];
  }
  
  private:
    T buffer [S];
    int pos;
};

typedef RingBuffer <float, 3> RBF3;


////////////////////////////////////////////////
//                  filtros                   //
////////////////////////////////////////////////

enum FilterType {LPF, HPF};

struct RecursiveFilter {
  
  float a0, a1, b1;                      // coeficientes
  RBF3 input_, output_;                  // ring buffers
  
  RecursiveFilter (FilterType type, float cutoff) {           // cutoff entre 0 y 0.5    
    // calcula los coeficientes
    b1 = exp (-2*PI*cutoff);
    switch (type) {
      case LPF:
        a0 = 1-b1;
        a1 = 0;
        break;
      case HPF:
        a0 = (1+b1)/2.0;
        a1 = -a0;
        break;
    }
  }
  
  float update (float value) {
    input_.store (value);
    output_.store (a0*input_.get(0) + a1*input_.get(1) + b1*output_.get(0));
    return output_.get(0);
  }
  
};


////////////////////////////////////////////////
//          fixed point number type           //
////////////////////////////////////////////////

struct FixedPointNumber {
  uint8_t   units;
  uint8_t   cents;
  inline float getValue () {
    return units + cents/100.0 - 128.0;
  }
  inline void setValue (float value) {
    value += 128.0;
    units = int(value);
    cents = (value-(float)units)*100.0;
  }
};


////////////////////////////////////////////////
//                Timer class                 //
////////////////////////////////////////////////

// clase universal para timers (con millis())
template <typename T=void>
class Timer {
  private:
    unsigned long last_time;
    long periodo;
    void (*runtime)();
    void (*runtime2)(T*);
  public:
    Timer (long periodo_, void (*runtime_)());
    Timer (long periodo_, void (*runtime_)(T*));
    void setPause (unsigned long millis_);
    void update ();
    void update (T*);
};

template <typename T>
Timer<T>::Timer (long periodo_, void (*runtime_)()) : periodo (periodo_), runtime (runtime_) {
  last_time = millis ();
}

template <typename T>
Timer<T>::Timer (long periodo_, void (*runtime_)(T*)) : periodo (periodo_), runtime2 (runtime_) {
  last_time = millis ();
}

template <typename T>
void Timer<T>::update () {
  unsigned long now = millis ();
  if (now > last_time && now-last_time > periodo) {
    last_time = now;
    (*runtime) ();
  }   
}

template <typename T>
void Timer<T>::update (T* param) {
  unsigned long now = millis ();
  if (now > last_time && now-last_time > periodo) {
    last_time = now;
    (*runtime2) (param);
  }   
}

template <typename T>
void Timer<T>::setPause (unsigned long millis_) {
  last_time += millis_;
}


///////////////////////////////////////////////
//                super misc.                //
///////////////////////////////////////////////

enum {NEGRO, ROJO, AMARILLO, ROJOAMARILLO, AZUL, AZULROJO, AZULAMARILO, AZULAMARILLOROJO};


#endif



