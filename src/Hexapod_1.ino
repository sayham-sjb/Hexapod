#include "Servo.h"

Servo servo_Lijevi_Gornji_Zglob_1;        // LEFT UP JOINT
Servo servo_Lijevi_Gornji_Zglob_2;        //                      ________________________________________
Servo servo_Lijevi_Gornji_Zglob_3;       // Pin- 22,23,24    |                                        |   25,26,27
                                          //                     |                                        |
Servo servo_Desni_Gornji_Zglob_1;         // RIGHT UP  JOINT                  |                                        |
Servo servo_Desni_Gornji_Zglob_2;         //                     |                                        |
Servo servo_Desni_Gornji_Zglob_3;         //                     |                                        |
                                          //                     |                                        |
Servo servo_Lijevi_Srednji_Zglob_1;       //  LEFT MIDDLE  JOINT                   |                                        |
Servo servo_Lijevi_Srednji_Zglob_2;       //                     |                                        |
Servo servo_Lijevi_Srednji_Zglob_3;      // Pin - 28,29,30    |                HEXAPOD                 |   31,32,33
                                          //                     |                                        |
Servo servo_Desni_Srednji_Zglob_1;        // RIGHT MIDDLE JOINT                    |                                        |
Servo servo_Desni_Srednji_Zglob_2;        //                     |                                        |
Servo servo_Desni_Srednji_Zglob_3;        //                     |                                        |
                                          //                     |                                        |
Servo servo_Lijevi_Donji_Zglob_1;         //  LEFT DOWN JOINT                  |                                        |
Servo servo_Lijevi_Donji_Zglob_2;         //                     |                                        |
Servo servo_Lijevi_Donji_Zglob_3;        // Pin - 34,35,36    |                                        |   37,38,39
                                          //                     |________________________________________|
Servo servo_Desni_Donji_Zglob_1;          // RIGHT DOWN JOINT
Servo servo_Desni_Donji_Zglob_2;          //
Servo servo_Desni_Donji_Zglob_3; 




int val;

int svijetla1 = 11;


int rasvjeta = 0;
int brightness = 0;

int Sklop = 11; 

void setup()
{
  pinMode(svijetla1, OUTPUT);
  
  servo_Lijevi_Gornji_Zglob_1.attach(39);
  servo_Lijevi_Gornji_Zglob_2.attach(38);
  servo_Lijevi_Gornji_Zglob_3.attach(37);
  servo_Desni_Gornji_Zglob_1.attach(36);
  servo_Desni_Gornji_Zglob_2.attach(35);
  servo_Desni_Gornji_Zglob_3.attach(34);
  servo_Lijevi_Srednji_Zglob_1.attach(33);
  servo_Lijevi_Srednji_Zglob_2.attach(32);
  servo_Lijevi_Srednji_Zglob_3.attach(31);
  servo_Desni_Srednji_Zglob_1.attach(30);
  servo_Desni_Srednji_Zglob_2.attach(29);
  servo_Desni_Srednji_Zglob_3.attach(28);  
  servo_Lijevi_Donji_Zglob_1.attach(27);
  servo_Lijevi_Donji_Zglob_2.attach(26);
  servo_Lijevi_Donji_Zglob_3.attach(25);
  servo_Desni_Donji_Zglob_1.attach(24);
  servo_Desni_Donji_Zglob_2.attach(23);
  servo_Desni_Donji_Zglob_3.attach(22);


  servo_Lijevi_Gornji_Zglob_1.write(78);
  servo_Desni_Gornji_Zglob_1.write(77);
  servo_Lijevi_Srednji_Zglob_1.write(108);
  servo_Desni_Srednji_Zglob_1.write(76);
  servo_Lijevi_Donji_Zglob_1.write(75);
  servo_Desni_Donji_Zglob_1.write(83);

  servo_Lijevi_Gornji_Zglob_2.write(120);
  servo_Desni_Gornji_Zglob_2.write(62);
  servo_Lijevi_Srednji_Zglob_2.write(116);
  servo_Desni_Srednji_Zglob_2.write(35);
  servo_Lijevi_Donji_Zglob_2.write(62);
  servo_Desni_Donji_Zglob_2.write(132);

  servo_Lijevi_Gornji_Zglob_3.write(41);
  servo_Desni_Gornji_Zglob_3.write(32);
  servo_Lijevi_Srednji_Zglob_3.write(178);
  servo_Desni_Srednji_Zglob_3.write(6);
  servo_Lijevi_Donji_Zglob_3.write(34);
  servo_Desni_Donji_Zglob_3.write(15);


  delay(1000);
  servo_Lijevi_Gornji_Zglob_3.write(45);
  servo_Desni_Gornji_Zglob_3.write(35);
  servo_Lijevi_Srednji_Zglob_3.write(178);
  servo_Desni_Srednji_Zglob_3.write(10);
  servo_Lijevi_Donji_Zglob_3.write(40);
  servo_Desni_Donji_Zglob_3.write(20);


  Serial.begin(9600);  

  delay(5000);
}
int stanje = 0;

void loop() 
{ 
  
  if (Serial.available())
  {
    
    char val = Serial.read();
    switch(val)
    {
    case 'F': 
      {
        if(stanje = 1)
        {
        Naprijed();
        }
        break;
      }
    case 'W':
      {
         stanje = 1;
        USTANI();
       
        break;
      }
    case 'w': 
      {
        stanje = 0;
        SJEDNI();
        
        break;
      }
    case 'B':
      {
        if(stanje = 1)
        {
        Nazad();
        }
        break;
      }
    case 'L': 
      {
        if(stanje = 1)
        {
        Lijevo();
        }
        break;
      }
    case 'R':
      {
        if(stanje = 1)
        {
        Desno();
        }
        break;
      }
      case 'U':
      {
       
      RasvjetaON();
        break;
      }
      case 'u':
      {
      RasvjetaOFF();
        break;
      }
    }

  }
} 
void RasvjetaON()
{
 
    digitalWrite(svijetla1, HIGH);

}
void RasvjetaOFF()
{
   digitalWrite(svijetla1, LOW);  
}

void USTANI()
{
   servo_Lijevi_Gornji_Zglob_1.write(78);
  servo_Desni_Gornji_Zglob_1.write(77);
  servo_Lijevi_Srednji_Zglob_1.write(108);
  servo_Desni_Srednji_Zglob_1.write(76);
  servo_Lijevi_Donji_Zglob_1.write(75);
  servo_Desni_Donji_Zglob_1.write(83);

  delay(1000);
  servo_Lijevi_Gornji_Zglob_3.write(45);
  servo_Desni_Gornji_Zglob_3.write(35);
  servo_Lijevi_Srednji_Zglob_3.write(178);
  servo_Desni_Srednji_Zglob_3.write(10);
  servo_Lijevi_Donji_Zglob_3.write(40);
  servo_Desni_Donji_Zglob_3.write(20);

  delay(400);
  servo_Lijevi_Gornji_Zglob_2.write(179);
  servo_Desni_Gornji_Zglob_2.write(114);
  servo_Lijevi_Srednji_Zglob_2.write(51);
  servo_Desni_Srednji_Zglob_2.write(88);
  servo_Lijevi_Donji_Zglob_2.write(121);
  servo_Desni_Donji_Zglob_2.write(76);
  delay(500);
  servo_Lijevi_Gornji_Zglob_3.write(61);
  servo_Desni_Gornji_Zglob_3.write(52);
  servo_Lijevi_Srednji_Zglob_3.write(160);
  servo_Desni_Srednji_Zglob_3.write(26);
  servo_Lijevi_Donji_Zglob_3.write(54);
  servo_Desni_Donji_Zglob_3.write(35);
stanje = 1;
}



void SJEDNI()
{
  delay(1000);
  servo_Lijevi_Gornji_Zglob_1.write(78);
  servo_Desni_Gornji_Zglob_1.write(77);
  servo_Lijevi_Srednji_Zglob_1.write(108);
  servo_Desni_Srednji_Zglob_1.write(76);
  servo_Lijevi_Donji_Zglob_1.write(75);
  servo_Desni_Donji_Zglob_1.write(83);
  
  servo_Lijevi_Gornji_Zglob_2.write(90); //Povecati
  servo_Desni_Gornji_Zglob_2.write(95);   // Smanjiti
  servo_Lijevi_Srednji_Zglob_2.write(75);  // Povecati
  servo_Desni_Srednji_Zglob_2.write(65);   // Smanjiti
  servo_Lijevi_Donji_Zglob_2.write(85);    // Smanjiti
  servo_Desni_Donji_Zglob_2.write(100);    // Povecati
  delay(100);
  servo_Lijevi_Gornji_Zglob_2.write(95); //Povecati
  servo_Desni_Gornji_Zglob_2.write(90);   // Smanjiti
  servo_Lijevi_Srednji_Zglob_2.write(80);  // Povecati
  servo_Desni_Srednji_Zglob_2.write(60);   // Smanjiti
  servo_Lijevi_Donji_Zglob_2.write(80);    // Smanjiti
  servo_Desni_Donji_Zglob_2.write(105);    // Povecati
  delay(100);
  servo_Lijevi_Gornji_Zglob_2.write(107); //Povecati
  servo_Desni_Gornji_Zglob_2.write(82);   // Smanjiti
  servo_Lijevi_Srednji_Zglob_2.write(87);  // Povecati
  servo_Desni_Srednji_Zglob_2.write(52);   // Smanjiti
  servo_Lijevi_Donji_Zglob_2.write(72);    // Smanjiti
  servo_Desni_Donji_Zglob_2.write(117);    // Povecati
  delay(100);
  servo_Lijevi_Gornji_Zglob_2.write(115); //Povecati
  servo_Desni_Gornji_Zglob_2.write(75);   // Smanjiti
  servo_Lijevi_Srednji_Zglob_2.write(95);  // Povecati
  servo_Desni_Srednji_Zglob_2.write(45);   // Smanjiti
  servo_Lijevi_Donji_Zglob_2.write(65);    // Smanjiti
  servo_Desni_Donji_Zglob_2.write(125);    // Povecati
  delay(500);

  servo_Lijevi_Gornji_Zglob_2.write(120);
  servo_Desni_Gornji_Zglob_2.write(50);
  servo_Lijevi_Srednji_Zglob_2.write(108);
  servo_Desni_Srednji_Zglob_2.write(30);
  servo_Lijevi_Donji_Zglob_2.write(60);
  servo_Desni_Donji_Zglob_2.write(133);

  servo_Lijevi_Gornji_Zglob_3.write(51);
  servo_Desni_Gornji_Zglob_3.write(37);
  servo_Lijevi_Srednji_Zglob_3.write(165);
  servo_Desni_Srednji_Zglob_3.write(19);
  servo_Lijevi_Donji_Zglob_3.write(53);
  servo_Desni_Donji_Zglob_3.write(35);

stanje = 0;
}



void Naprijed()
{
  servo_Lijevi_Gornji_Zglob_2.write(152); 
  servo_Desni_Srednji_Zglob_2.write(42);  
  servo_Lijevi_Donji_Zglob_2.write(93); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(98);  
  servo_Desni_Srednji_Zglob_1.write(56);
  servo_Lijevi_Donji_Zglob_1.write(54); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_2.write(179);
  servo_Desni_Srednji_Zglob_2.write(88);  
  servo_Lijevi_Donji_Zglob_2.write(122); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(78);
  servo_Desni_Srednji_Zglob_1.write(76);
  servo_Lijevi_Donji_Zglob_1.write(75);
  delay(100);
  
  servo_Desni_Gornji_Zglob_2.write(70);
  servo_Lijevi_Srednji_Zglob_2.write(88);
  servo_Desni_Donji_Zglob_2.write(110);
  delay(100);
  servo_Desni_Gornji_Zglob_1.write(63);
  servo_Lijevi_Srednji_Zglob_1.write(123);
  servo_Desni_Donji_Zglob_1.write(95);
  delay(100);
  servo_Desni_Gornji_Zglob_2.write(114);
  servo_Lijevi_Srednji_Zglob_2.write(51);
  servo_Desni_Donji_Zglob_2.write(76);
  delay(100);
  servo_Desni_Gornji_Zglob_1.write(77);
  servo_Lijevi_Srednji_Zglob_1.write(108);
  servo_Desni_Donji_Zglob_1.write(83);
  delay(100);

}


void Nazad()
{
 servo_Lijevi_Gornji_Zglob_2.write(152); 
  servo_Desni_Srednji_Zglob_2.write(42);  
  servo_Lijevi_Donji_Zglob_2.write(93); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(63);  
  servo_Desni_Srednji_Zglob_1.write(91);
  servo_Lijevi_Donji_Zglob_1.write(89); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_2.write(179);
  servo_Desni_Srednji_Zglob_2.write(88);  
  servo_Lijevi_Donji_Zglob_2.write(122); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(78);
  servo_Desni_Srednji_Zglob_1.write(76);
  servo_Lijevi_Donji_Zglob_1.write(75);
  delay(100);

  servo_Desni_Gornji_Zglob_2.write(70);
  servo_Lijevi_Srednji_Zglob_2.write(88);
  servo_Desni_Donji_Zglob_2.write(110);
  delay(100);
  servo_Desni_Gornji_Zglob_1.write(97);
  servo_Lijevi_Srednji_Zglob_1.write(95);
  servo_Desni_Donji_Zglob_1.write(67);
  delay(100);
  servo_Desni_Gornji_Zglob_2.write(112);
  servo_Lijevi_Srednji_Zglob_2.write(56);
  servo_Desni_Donji_Zglob_2.write(80);
  delay(100);
   servo_Desni_Gornji_Zglob_1.write(77);
  servo_Lijevi_Srednji_Zglob_1.write(108);
  servo_Desni_Donji_Zglob_1.write(83);
  delay(100);
}




void Lijevo()
{
   servo_Lijevi_Gornji_Zglob_2.write(150); 
  servo_Desni_Srednji_Zglob_2.write(37);  
  servo_Lijevi_Donji_Zglob_2.write(85); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(66);  
  servo_Desni_Srednji_Zglob_1.write(51);
  servo_Lijevi_Donji_Zglob_1.write(88); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_2.write(178); 
  servo_Desni_Srednji_Zglob_2.write(85);  
  servo_Lijevi_Donji_Zglob_2.write(125); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(78);
  servo_Desni_Srednji_Zglob_1.write(76);
  servo_Lijevi_Donji_Zglob_1.write(75);
  delay(100);

  servo_Desni_Gornji_Zglob_2.write(70);
  servo_Lijevi_Srednji_Zglob_2.write(88);
  servo_Desni_Donji_Zglob_2.write(113);
  delay(100);
  servo_Desni_Gornji_Zglob_1.write(56);
  servo_Lijevi_Srednji_Zglob_1.write(95);
  servo_Desni_Donji_Zglob_1.write(100);
  delay(100);
  servo_Desni_Gornji_Zglob_2.write(114);
  servo_Lijevi_Srednji_Zglob_2.write(51);
  servo_Desni_Donji_Zglob_2.write(76);
  delay(100);
   servo_Desni_Gornji_Zglob_1.write(77);
  servo_Lijevi_Srednji_Zglob_1.write(108);
  servo_Desni_Donji_Zglob_1.write(83);
  delay(100);

}




void Desno()
{ 

  servo_Lijevi_Gornji_Zglob_2.write(150); 
  servo_Desni_Srednji_Zglob_2.write(37);  
  servo_Lijevi_Donji_Zglob_2.write(85); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(95);
  servo_Desni_Srednji_Zglob_1.write(91);  
  servo_Lijevi_Donji_Zglob_1.write(63); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_2.write(178); 
  servo_Desni_Srednji_Zglob_2.write(85);  
  servo_Lijevi_Donji_Zglob_2.write(125); 
  delay(100);
  servo_Lijevi_Gornji_Zglob_1.write(78);
  servo_Desni_Srednji_Zglob_1.write(76);
  servo_Lijevi_Donji_Zglob_1.write(75);
  delay(100);


  servo_Desni_Gornji_Zglob_2.write(70);
  servo_Lijevi_Srednji_Zglob_2.write(88);
  servo_Desni_Donji_Zglob_2.write(113);
  delay(100);
  servo_Desni_Gornji_Zglob_1.write(98);
  servo_Lijevi_Srednji_Zglob_1.write(122);
  servo_Desni_Donji_Zglob_1.write(64);
  delay(100);
  servo_Desni_Gornji_Zglob_2.write(112);
  servo_Lijevi_Srednji_Zglob_2.write(56);
  servo_Desni_Donji_Zglob_2.write(78);
  delay(100);    
   servo_Desni_Gornji_Zglob_1.write(77);
  servo_Lijevi_Srednji_Zglob_1.write(108);
  servo_Desni_Donji_Zglob_1.write(83);
  delay(100);

}