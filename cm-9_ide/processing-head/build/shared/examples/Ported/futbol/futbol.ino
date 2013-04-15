#include "stdint.h"
//#include "CM9_BC.h"
#include "bplf.h"

BioloidController bioloid;

uint16_t servo_ids[] = {12,7,8,9,10,11,12,13,14,15,16,17,18};

void setup()
{
	Dxl.begin(1);
	SerialUSB.begin();

    pinMode(BOARD_LED_PIN, OUTPUT);

    // Waits 5 seconds for you to open the console (open too quickly after
    //   downloading new code, and you will get errors
    delay(5000);
    SerialUSB.print("Send any value to continue...\n");
    while(!SerialUSB.available())
    {
        delay(1000);
        digitalWrite(BOARD_LED_PIN, LOW);
        SerialUSB.print("Send any value to continue...\n");
        delay(1000);
        digitalWrite(BOARD_LED_PIN, HIGH);
    }


	delay(1000);
	bioloid.setup(servo_ids[0]);


	int iter;
	for (iter=0; iter<servo_ids[0]; iter++)
	{
		bioloid.setId(iter, servo_ids[iter+1]);
	}

	bioloid.playSeq(bplf_Init);
	while(bioloid.playing > 0)
	{
		bioloid.play();
		delay(3);
	}
	delay(500);

	SerialUSB.println("Am I still alive?");
}

void loop()
{

}
