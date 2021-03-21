
#include <Servo.h>

Servo myservo; // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 75;       // variable to store the servo position
int pos_turn = 145; // variable to store the servo position
int straight_pin = 4;
int right_pin = 5;
int left_pin = 6;
int turn_pin = 7;

void setup()
{
    //Serial.begin(9600);
    myservo.attach(3); // attaches the servo on pin 9 to the servo object
    pinMode(straight_pin, INPUT);
    pinMode(right_pin, INPUT);
    pinMode(left_pin, INPUT);
    pinMode(turn_pin, INPUT);
}

void loop()
{
    int straight = digitalRead(straight_pin); // read the input pin
    int right = digitalRead(right_pin);       // read the input pin
    int left = digitalRead(left_pin);         // read the input pin
    int turn = digitalRead(turn_pin);         // read the input pin
    int sum = straight + right + left + turn;

    if (sum < 2)
    {
        if (straight == 1)
        {
            myservo.write(pos); // tell servo to go to position in variable 'pos'
            delay(100);
        }
        else if (right == 1)
        {
            myservo.write(pos + 3); // tell servo to go to position in variable 'pos'
            delay(100);
        }
        else if (left == 1)
        {
            myservo.write(pos - 3); // tell servo to go to position in variable 'pos'
            delay(100);
        }
        else if (turn == 1)
        {
            myservo.write(pos_turn); // tell servo to go to position in variable 'pos'
            delay(100);
        }
    }
}