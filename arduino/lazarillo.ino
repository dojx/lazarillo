#include <ros.h>
#include <math.h> 
#include <std_msgs/Bool.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

// Interrupts
#define ED1 20 
#define EI1 18 
#define ED2 21 
#define EI2 19 

// PWM
#define LDerecha 6
#define LIzquierda 5 
#define atrasD 8
#define atrasI 7
#define adelanteD 10
#define adelanteI 9

// Botones
#define rosa 14 // botón 1
#define azul 15 // botón 2
#define naranja 16 // botón 3
#define morado 17 // botón 4

volatile long pulsosD = 0, pulsosI = 0;
unsigned long t_actual = 0, t_pasado = 0;
float wD = 0, wI = 0; // Velocidades ángulares en rad/s
const short ppv = 2716; // Pulsos por vuelta (completa) del motorreductor
const float resolucion = (2.0 * 3.141592) / float(ppv); // Radianes que gira el eje del motorreductor por cada pulso del encoder
const short td_m = 200; // Tiempo (milisegundos)
const float td = td_m / 1000.0; // Tiempo (segundos)
int coma = 0, wDn, wIn;
float xp = 0, yp = 0, thp = 0, x = 0, y = 0, th = 0;
float L = 0.19, R = 0.02205;

void update_pwm(const geometry_msgs::Quaternion& msg) {
    if (msg.z) {
      digitalWrite(adelanteI, HIGH);
      digitalWrite(atrasI, LOW);
    } else {
      digitalWrite(adelanteI, LOW);
      digitalWrite(atrasI, HIGH);
    }
    if (msg.w) {
      digitalWrite(adelanteD, HIGH);
      digitalWrite(atrasD, LOW);
    } else {
      digitalWrite(adelanteD, LOW);
      digitalWrite(atrasD, HIGH);
    }
    
    analogWrite(LIzquierda, msg.x); // Velocidad a escribir en la llanta izquierda
    analogWrite(LDerecha, msg.y); // Velocidad a escribir en la llanta derecha

//    pulsosD=0; pulsosI=0; //Se resetea la cuenta
//    t_pasado=millis(); //Se actualiza el valor
}

ros::NodeHandle nh;

geometry_msgs::Twist odom_msg;
geometry_msgs::Quaternion btn_msg;

ros::Subscriber<geometry_msgs::Quaternion> pwm_sub("/lazarillo/llantas_vel", &update_pwm);
ros::Publisher odom_pub("/lazarillo/odom", &odom_msg);
ros::Publisher btn_pub("/lazarillo/buttons", &btn_msg);

void setup() {
  pinMode(ED1, INPUT_PULLUP);
  pinMode(EI1, INPUT_PULLUP);
  pinMode(ED2, INPUT_PULLUP);
  pinMode(EI2, INPUT_PULLUP);
  pinMode(LDerecha, OUTPUT);
  pinMode(LIzquierda, OUTPUT);
  pinMode(adelanteD, OUTPUT);
  pinMode(adelanteI, OUTPUT);
  pinMode(atrasD, OUTPUT);
  pinMode(atrasI, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ED1), cuentaD1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ED2), cuentaD2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EI1), cuentaI1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EI2), cuentaI2, CHANGE);
  digitalWrite(adelanteD,LOW);
  digitalWrite(atrasD,LOW);
  digitalWrite(adelanteI,LOW);
  digitalWrite(atrasI,LOW);

  // Botones
  pinMode(rosa, INPUT_PULLUP);
  pinMode(azul, INPUT_PULLUP);
  pinMode(naranja, INPUT_PULLUP);
  pinMode(morado, INPUT_PULLUP);

  analogWrite(LDerecha,0);
  analogWrite(LIzquierda,0);
  digitalWrite(adelanteD,HIGH);
  digitalWrite(adelanteI,HIGH);
  
  /*"Velocidades" iniciales*/
  analogWrite(LDerecha, 0);
  analogWrite(LIzquierda, 0);

  cli(); // stop interrupts

  // set timer5 interrupt at 5Hz
  TCCR5A = 0; // set entire TCCR5A register to 0
  TCCR5B = 0; // same for TCCR5B
  TCNT5  = 0; // initialize counter value to 0
  // set compare match register for 1hz increments
  OCR5A = 15624/5; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR5B |= (1 << WGM52);
  // Set CS50 and CS52 bits for 1024 prescaler
  TCCR5B |= (1 << CS50) | (1 << CS52);  
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE5A);
  
  sei(); // allow interrupts

  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(btn_pub);
  nh.subscribe(pwm_sub); 
}

ISR(TIMER5_COMPA_vect) {
  // Cálculo de las velocidades ángulares de las llantas
  wD = resolucion * float(pulsosD) / td; // Resolución*(pulsos/tiempo)
  wI = resolucion * float(pulsosI) / td;
  
  xp = (R / 2) * (wD + wI) * cos(th);
  yp = (R / 2) * (wD + wI) * sin(th);
  thp = (R / L) * (wD - wI);
  
  x += xp * td;
  y += yp * td;
  th += thp * td;

  odom_msg.linear.x = x;
  odom_msg.linear.y = y;
  odom_msg.linear.z = th;

  odom_msg.angular.x = xp;
  odom_msg.angular.y = yp;
  odom_msg.angular.z = thp;

  odom_pub.publish(&odom_msg);

  pulsosD = 0; pulsosI = 0; //Se resetea la cuenta
}

void loop() {
  if(!digitalRead(rosa) || !digitalRead(azul) || !digitalRead(naranja) || !digitalRead(morado)){
    delay(20); // Tiempo para eliminar el deboucing
    btn_msg.x = !digitalRead(rosa);
    btn_msg.y = !digitalRead(azul);
    btn_msg.z = !digitalRead(naranja);
    btn_msg.w = !digitalRead(morado);
    btn_pub.publish(&btn_msg);  
    //Mientras se siga presionando
    while(!digitalRead(rosa) || !digitalRead(azul) || !digitalRead(naranja) || !digitalRead(morado)){} 
    delay(20); // Tiempo para eliminar el deboucing
  } 
  
  nh.spinOnce();
}

void cuentaD1() { // Llanta derecha. Encoder 1
  if (digitalRead(ED1)==HIGH) { // Se lee un flanco de subida
    if (digitalRead(ED2)==HIGH) { // Gira hacia adelante
      pulsosD++;
    }
    else {//gira hacia atras
      pulsosD--;
    }
  }
  else { // Se lee un flanco de bajada
    if (digitalRead(ED2)==LOW) { // Gira hacia adelante
      pulsosD++;
    }
    else { // Gira hacia atras
      pulsosD--;
    }
  }
}

void cuentaD2() { // Llanta derecha. Encoder 2
  if (digitalRead(ED2) == HIGH) { // Se lee un flanco de subida
    if (digitalRead(ED1) == LOW) { // Gira hacia adelante
      pulsosD++;
    }
    else { // Gira hacia atras
      pulsosD--;
    }
  }
  else{ // Se lee un flanco de bajada
    if(digitalRead(ED1)==HIGH){ // Gira hacia adelante
      pulsosD++;
    }
    else{ // Gira hacia atras
      pulsosD--;
    }
  }
}

void cuentaI1() { // Llanta izquierda. Encoder 1
  if (digitalRead(EI1) == HIGH){ // Se lee un flanco de subida
    if (digitalRead(EI2) == LOW){ // Gira hacia adelante
      pulsosI++;
    }
    else { // Gira hacia atras
      pulsosI--;
    }
  }
  else { // Se lee un flanco de bajada
    if (digitalRead(EI2) == HIGH){ // Gira hacia adelante
      pulsosI++;
    }
    else { // Gira hacia atras
      pulsosI--;
    }
  }
}

void cuentaI2() { // Llanta izquierda. Encoder 2
  if (digitalRead(EI2) == HIGH) { // Se lee un flanco de subida
    if (digitalRead(EI1) == HIGH) { // Gira hacia adelante
      pulsosI++;
    }
    else {//gira hacia atras
      pulsosI--;
    }
  }
  else { // Se lee un flanco de bajada
    if (digitalRead(EI1) == LOW){ // Gira hacia adelante
      pulsosI++;
    }
    else { // Gira hacia atras
      pulsosI--;
    }
  }
}
