/*
 * test_packetSending.c
 *
 *  Created on: May 2, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *
 */

#include "test_packetSending.h"


void sendPacketLoop() {
	while(1) {
		roboData debugRoboData;
		roboAckData debugRoboAckData;
		uint8_t debugRoboPacket[ROBOPKTLEN];

		debugRoboData.id = 10; //robot 10
		debugRoboData.rho = 1;
		debugRoboData.theta = 2;
		debugRoboData.driving_reference = 0;
		debugRoboData.use_cam_info = 0;
		debugRoboData.velocity_angular = 4;
		debugRoboData.debug_info = 1;
		debugRoboData.do_kick = 1;
		debugRoboData.do_chip = 0;
		debugRoboData.kick_chip_forced = 1;
		debugRoboData.kick_chip_power = 0;
		debugRoboData.velocity_dribbler = 15;
		debugRoboData.geneva_drive_state = 3;
		debugRoboData.cam_position_x = 1;
		debugRoboData.cam_position_y = 2;
		debugRoboData.cam_rotation = 3;
		robotDataToPacket(&debugRoboData, debugRoboPacket);

		//let's overwrite that...

		//debugRoboPacket[0] = 0x12;
		//debugRoboPacket[0] = (uint8_t)   							// aaaaabbb
						//(0b11111000 & (10 << 3));                  // aaaaa000   5 bits; bits  4-0 to 7-3

/*		debugRoboPacket[1] = 0x60;
		debugRoboPacket[2] = 0x70;
		debugRoboPacket[3] = 0x80;
		debugRoboPacket[4] = 0x90;
		debugRoboPacket[5] = 0xA0;
		debugRoboPacket[6] = 0xB0;
		debugRoboPacket[7] = 0xC0;
		debugRoboPacket[8] = 0x55;
		debugRoboPacket[9] = 0x66;
		debugRoboPacket[10] = 0x77;
		debugRoboPacket[11] = 0x88;
		debugRoboPacket[12] = 0x99;*/


		unsigned int retransmissionSum = 0;
		uint16_t lostpackets = 0;
		uint16_t emptyack = 0;
		uint16_t packetsToTransmit = 100;
		uint8_t verbose = 1;
		uint16_t interpacketdelay = 1000; //delay in milliseconds between packets

		for(uint16_t i = 0; i<packetsToTransmit; i++) {
			sendPacket(debugRoboPacket);
			HAL_Delay(5);

			uint8_t ack_payload[32];
			uint8_t payload_length;
			int8_t returncode;

			do {
				returncode = getAck(ack_payload, &payload_length);
				//TextOut("No Interrupt yet\n");
			} while(returncode == -1);
			uint8_t retr = readReg(OBSERVE_TX)&0x0f;

			if(returncode == -2) {
				lostpackets++;
				if(verbose) {
					sprintf(smallStrBuffer, "%i. Packet lost.\n", (i+1));
					TextOut(smallStrBuffer);
				}

			} else if(returncode == 1) {
				if(payload_length >= SHORTACKPKTLEN)
					ackPacketToRoboAckData(ack_payload, payload_length, &debugRoboAckData);
				if(verbose) {
					sprintf(smallStrBuffer, "%i. Packet delivered with %i retransmissions.\n", (i+1), retr);
					TextOut(smallStrBuffer);
					sprintf(smallStrBuffer, "Received payload length: %i\n", payload_length);
					TextOut(smallStrBuffer);
					//sprintf(smallStrBuffer, "ACK Data: RoboID: %i\n", debugRoboAckData.roboID);
					sprintf(smallStrBuffer, "ACK Data: ");
					TextOut(smallStrBuffer);
					for(uint8_t i=0; i < payload_length; i++) {
						sprintf(smallStrBuffer, "%02x ", ack_payload[i]);
						TextOut(smallStrBuffer);
					}
					TextOut("\n");
					sprintf(smallStrBuffer, "Robot ID: %i\n",debugRoboAckData.roboID);
					TextOut(smallStrBuffer);
					sprintf(smallStrBuffer, "xPos: %i\n",debugRoboAckData.xPosRobot);
					TextOut(smallStrBuffer);
					sprintf(smallStrBuffer, "BallSensor: %i\n",debugRoboAckData.ballSensor);
					TextOut(smallStrBuffer);

					//TextOut(smallStrBuffer);
					TextOut("\n\n");
				}
				retransmissionSum += retr;
			} else if(returncode == 0) {
				emptyack++;
				if(verbose) {
					sprintf(smallStrBuffer, "%i. Packet delivered with empty ACK!\n", (i+1));
					TextOut(smallStrBuffer);
				}
			}
			clearInterrupts();
			HAL_Delay(interpacketdelay);

		}
		uint8_t packetloss = (int)(lostpackets*100.0)/(packetsToTransmit*1.0);
		uint8_t emptyackprocent = (int) (emptyack*100.0)/(packetsToTransmit*1.0);
		sprintf(smallStrBuffer, "Packets TX'd: %i with a delay of %i ms, delivered: %i with retransmissions: %u, packet loss: %i %% (empty ack: %i %%) \n\n", packetsToTransmit, interpacketdelay,(packetsToTransmit-lostpackets), retransmissionSum, packetloss, emptyackprocent);
		TextOut(smallStrBuffer);
		continue; //skip to the next loop iteration
	}
}
