/* 
  ArbotiX Test Program for use with PyPose 0015
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "CM9_BC.h"

// Using USB debug output requires CM-900 be connected to PC via USB
//#define USB_DEBUG_OUTPUT

BioloidController bioloid;

#define ARB_SIZE_POSE   7  // also initializes
#define ARB_LOAD_POSE   8
#define ARB_LOAD_SEQ    9
#define ARB_PLAY_SEQ    10
#define ARB_LOOP_SEQ    11
#define ARB_TEST        25
#define MAX_SERVOS        30

//#define INST_READ           0x02
//#define INST_WRITE          0x03

int mode = 0;                   // where we are in the frame

unsigned int id = 0;           // id of this frame
unsigned int length = 0;       // length of this frame
unsigned int ins = 0;          // instruction of this frame

unsigned int params[143];      // parameters (match RX-64 buffer size)
unsigned int iter = 0;        // index in param buffer

int checksum;                   // checksum

typedef struct{
    unsigned int pose;         // index of pose to transition to 
    int time;                   // time for transition
} sp_trans_t;

//  pose and sequence storage
int poses[30][MAX_SERVOS]; // poses [iter][servo_id-1]
sp_trans_t sequence[50];        // sequence
int seqPos;                     // step in current sequence

void setup(){
    Serial2.begin(38400);
#ifdef USB_DEBUG_OUTPUT
    SerialUSB.begin();
#endif
    Dxl.begin(1);
	bioloid.setup(MAX_SERVOS);

    pinMode(BOARD_LED_PIN, OUTPUT);


#ifdef USB_DEBUG_OUTPUT
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
#endif

}

/* 
 * packet: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix
 *
 * ID = 253 for these special commands!
 * Pose Size = 7, followed by single param: size of pose
 * Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
 * Seq Size = 9, followed by single param: size of seq
 * Load Seq = A, followed by index/times (# of parameters = 3*seq_size) 
 * Play Seq = B, no params
 * Loop Seq = C, 
 */
    
void loop(){
    int i;
    
    // process messages
    while(Serial2.available() > 0){
        // We need to 0xFF at start of packet
        if(mode == 0){         // start of new packet
            if(Serial2.read() == 0xff){
#ifdef USB_DEBUG_OUTPUT
                SerialUSB.print("\n0xff,");
#endif
                mode = 2;
//                digitalWrite(0,HIGH-digitalRead(0));
            }
//        }else if(mode == 1){   // another start byte
//            if(Serial2.read() == 0xff){
#ifdef USB_DEBUG_OUTPUT
//                SerialUSB.print("0xff,");
#endif
//                mode = 2;
//            }
            else{
#ifdef USB_DEBUG_OUTPUT
                SerialUSB.print(";blah;\n\n");
#endif
                mode = 0;
            }
        }else if(mode == 2){   // next byte is index of servo
            id = Serial2.read();
            if(id != 0xff){
#ifdef USB_DEBUG_OUTPUT
                SerialUSB.print(id);
                SerialUSB.print(",");
#endif
                mode = 3;
            }
            else{
#ifdef USB_DEBUG_OUTPUT
                SerialUSB.print("0xff,");
#endif
            }
        }else if(mode == 3){   // next byte is length
            length = Serial2.read();
#ifdef USB_DEBUG_OUTPUT
            SerialUSB.print(length);
            SerialUSB.print(",");
#endif
            checksum = id + length;
            mode = 4;
        }else if(mode == 4){   // next byte is instruction
            ins = Serial2.read();
#ifdef USB_DEBUG_OUTPUT
            SerialUSB.print(ins);
            SerialUSB.print(",");
#endif
            checksum += ins;
            iter = 0;
            mode = 5;
        }else if(mode == 5){   // read data in 
            params[iter] = Serial2.read();
#ifdef USB_DEBUG_OUTPUT
            SerialUSB.print(params[iter]);
            SerialUSB.print(",");
#endif
            checksum += (int) params[iter];
            iter++;
            if(iter + 1 == length){  // we've read params & checksum
                mode = 0;
                if((checksum%256) != 255){ 
                    // return a packet: FF FF id Len Err params=None check
                    Serial2.write(0xff);
                    Serial2.write(0xff);
                    Serial2.write(id);
                    Serial2.write(2);
                    Serial2.write(64);
                    Serial2.write(255-((66+id)%256));

#ifdef USB_DEBUG_OUTPUT
                    SerialUSB.print("\n\n0xff,");
                    SerialUSB.print("0xff,");
                    SerialUSB.print(id);
                    SerialUSB.print(",2,");
                    SerialUSB.print("64,");
                    SerialUSB.println(255-((66+id)%256));
#endif
                }else{
                    if(id == 253){
                        // return a packet: FF FF id Len Err params=None check
                        Serial2.write(0xff);
                        Serial2.write(0xff);
                        Serial2.write(id);
                        Serial2.write(2);
                        Serial2.write((unsigned char)0);
                        Serial2.write(255-((2+id)%256));
                        
#ifdef USB_DEBUG_OUTPUT
                        SerialUSB.print("\n\n0xff,");
                        SerialUSB.print("0xff,");
                        SerialUSB.print(id);
                        SerialUSB.print(",2,");
                        SerialUSB.print("0,");
                        SerialUSB.println(255-((2+id)%256));
#endif
                        // special ArbotiX instructions
                        // Pose Size = 7, followed by single param: size of pose
                        // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size+1)
                        // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size) 
                        // Play Seq = A, no params
                        if(ins == ARB_SIZE_POSE){
                            bioloid.poseSize = params[0];
                            bioloid.readPose();    
                            //Serial.println(bioloid.poseSize);
                        }else if(ins == ARB_LOAD_POSE){
                            int i;    
                            Serial2.print("New Pose:");
                            for(i=0; i<bioloid.poseSize; i++){
                                poses[params[0]][i] = params[(2*i)+1]+(params[(2*i)+2]<<8); 
                                Serial2.print(poses[params[0]][i]);
                                Serial2.print(",");     
                            } 
                            Serial2.println("");
                        }else if(ins == ARB_LOAD_SEQ){
                            int i;
                            for(i=0;i<(length-2)/3;i++){
                                sequence[i].pose = params[(i*3)];
                                sequence[i].time = params[(i*3)+1] + (params[(i*3)+2]<<8);
                                Serial2.print("New Transition:");
                                Serial2.print((int)sequence[i].pose);
                                Serial2.print(" in ");
                                Serial2.println(sequence[i].time);      
                            }
                        }else if(ins == ARB_PLAY_SEQ){
                            seqPos = 0;
                            while(sequence[seqPos].pose != 0xff){
                                int i;
                                int p = sequence[seqPos].pose;
                                // are we HALT?
                                if(Serial2.read() == 'H'){
#ifdef USB_DEBUG_OUTPUT
                                    SerialUSB.println("H");
#endif
                                    return;
                                }
                                // load pose
                                for(i=0; i<bioloid.poseSize; i++){
                                    bioloid.setNextPose(i+1,poses[p][i]);
                                } 
                                // interpolate
                                bioloid.interpolateSetup(sequence[seqPos].time);
                                while(bioloid.interpolating)
                                    bioloid.interpolateStep();
                                // next transition
                                seqPos++;
                            }
                        }else if(ins == ARB_LOOP_SEQ){
                            while(1){
                                seqPos = 0;
                                while(sequence[seqPos].pose != 0xff){
                                    int i;
                                    int p = sequence[seqPos].pose;
                                    // are we HALT?
                                    if(Serial2.read() == 'H'){
#ifdef USB_DEBUG_OUTPUT
                                        SerialUSB.println("H");
#endif
                                        return;
                                    }
                                    // load pose
                                    for(i=0; i<bioloid.poseSize; i++){
                                        bioloid.setNextPose(i+1,poses[p][i]);
                                    } 
                                    // interpolate
                                    bioloid.interpolateSetup(sequence[seqPos].time);
                                    while(bioloid.interpolating)
                                        bioloid.interpolateStep();
                                    // next transition
                                    seqPos++;
                                }
                            }
                        }else if(ins == ARB_TEST){
                            int i;
/*                            // Test Digital I/O
                            for(i=0;i<8;i++){
                                // test digital
                                pinMode(i,OUTPUT);
                                digitalWrite(i,HIGH);  
                                // test analog
                                pinMode(31-i,OUTPUT);
                                digitalWrite(31-i,HIGH);
                                
                                delay(500);
                                digitalWrite(i,LOW);
                                digitalWrite(31-i,LOW);
                            }
*/
                            // Test Ax-12
                            for(i=452;i<552;i+=20){
                                Dxl.writeWord(1,AXM_GOAL_POSITION_L,i);
                                delay(200);
                            }
/*                            // Test Analog I/O
                            for(i=0;i<8;i++){
                                // test digital
                                pinMode(i,OUTPUT);
                                digitalWrite(i,HIGH);  
                                // test analog
                                pinMode(31-i,OUTPUT);
                                digitalWrite(31-i,HIGH);
                                
                                delay(500);
                                digitalWrite(i,LOW);
                                digitalWrite(31-i,LOW);
                            }
*/                      }   
                    }else{
                        int i;
                        // pass thru
                       if(ins == INST_READ){
                            int i;
                            if (params[1]==1)
                                Dxl.readByte(id, params[0]);//, params[1]);
                            else if (params[1]==2)
                                Dxl.readWord(id, params[0]);
                            else{
                                Dxl.setTxPacketId(id);
                                Dxl.setTxPacketLength(length);
                                Dxl.setTxPacketInstruction(ins);
                                Dxl.setTxPacketParameter(0, params[0]);
                                Dxl.setTxPacketParameter(1, params[1]);
								Dxl.txrxPacket();
                            }
                            // return a packet: FF FF id Len Err params check
							int status = Dxl.getResult();
							if (status == COMM_RXSUCCESS){
								if(Dxl.getRxPacketLength() > 0){
#ifdef USB_DEBUG_OUTPUT
									SerialUSB.print("\n  RX:");
#endif
									for(i=0;i<Dxl.getRxPacketLength()+4;i++){
									// index offset by PARAMETER==5, so subtract 5 to get full packet
										int temp_data = Dxl.getRxPacketParameter(i-5);
										Serial2.write(temp_data);
#ifdef USB_DEBUG_OUTPUT
										SerialUSB.print(temp_data);
										SerialUSB.print(",");
#endif
									}
								}
                            }
							else{
#ifdef USB_DEBUG_OUTPUT
								SerialUSB.print("\n  RX Failed: ");
								SerialUSB.print(status);
#endif
							}
                        }else if(ins == INST_WRITE){
							if(length == 4){
                                Dxl.writeByte(id, params[0], params[1]);
                            }else{
                                int x = params[1] + (params[2]<<8);
                                Dxl.writeWord(id, params[0], x);
                            }
							
							int status = Dxl.getResult();
							if (status == COMM_RXSUCCESS){
								if(Dxl.getRxPacketLength() > 0){
#ifdef USB_DEBUG_OUTPUT
									SerialUSB.print("\n  RX:");
#endif
									for(i=0;i<Dxl.getRxPacketLength()+4;i++){
									// index offset by PARAMETER==5, so subtract 5 to get full packet
										int temp_data = Dxl.getRxPacketParameter(i-5);
										Serial2.write(temp_data);
#ifdef USB_DEBUG_OUTPUT
										SerialUSB.print(temp_data);
										SerialUSB.print(",");
#endif
									}
								}
                            }
							else{
#ifdef USB_DEBUG_OUTPUT
								SerialUSB.print("\n  RX Failed: ");
								SerialUSB.print(status);
#endif
							}
							/*
                            // return a packet: FF FF id Len Err params check
                            Serial2.write(0xff);
                            Serial2.write(0xff);
                            Serial2.write(id);
                            Serial2.write(2);
                            Serial2.write((unsigned char)0);
                            Serial2.write(255-((2+id)%256));

#ifdef USB_DEBUG_OUTPUT
                            SerialUSB.print("\n\n0xff,");
                            SerialUSB.print("0xff,");
                            SerialUSB.print(id);
                            SerialUSB.print(",2,");
                            SerialUSB.print("0,");
                            SerialUSB.println(255-((2+id)%256));
#endif
							*/
                        }
                    }
                }
            }
        }
    }
    
    // update joints
    bioloid.interpolateStep();
}

