/*
 * test_packing.c
 *
 *  Created on: Apr 12, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *			Just functions to test if the conversion functions
 *			of packing.c are working properly.
 */
#include "TextOut.h"
#include "packing.h"


void testPacking() {

	/*
	 * Test case:
	 * create a roboData Struct
	 * fill it with pseudo-random data
	 * convert it to a packet
	 * convert the packet back to roboData
	 * compare the original roboData with the output.
	 */
	roboData robInput;
	roboData robOutput;

	uint8_t roboPkt[32]; //larger than it needs to be..


	while(1) {

		//filling struct with pseudo-random data
		robInput.id = (uint8_t) HAL_GetTick()&0x1f;
		robInput.rho = (uint16_t) HAL_GetTick()&0x7ff;
		robInput.theta = (uint16_t) HAL_GetTick()&0x7ff;
		robInput.driving_reference = (uint8_t) HAL_GetTick()&1;
		robInput.use_cam_info = (uint8_t) HAL_GetTick()&1;
		robInput.velocity_angular = (int16_t) HAL_GetTick()&0x1ff;
		robInput.debug_info = (uint8_t) HAL_GetTick()&1;
		robInput.do_kick = (uint8_t) HAL_GetTick()&1;
		robInput.do_chip = (uint8_t) HAL_GetTick()&1;
		robInput.kick_chip_forced = (uint8_t) HAL_GetTick()&1;
		robInput.kick_chip_power = (uint8_t) HAL_GetTick()&0xff;
		robInput.velocity_dribbler = (uint8_t) HAL_GetTick()&0xff;
		robInput.geneva_drive_state = (uint8_t) HAL_GetTick()&7;
		robInput.cam_position_x = (uint16_t) HAL_GetTick()&0x1fff;
		robInput.cam_position_y = (uint16_t) HAL_GetTick()&0x1fff;
		robInput.cam_rotation = (uint16_t) HAL_GetTick()&0x7ff;



		//converting struct to packet
		robotDataToPacket(&robInput, roboPkt);

		//converting back to struct
		packetToRoboData(roboPkt, &robOutput);

		//comparing input with output
		uint8_t mismatch = 0;
		if(robInput.id != robOutput.id) {
			sprintf(smallStrBuffer,"Mismatch in id. Input: %i, Output: %i\n", robInput.id, robOutput.id);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.rho != robOutput.rho) {
			sprintf(smallStrBuffer,"Mismatch in rho. Input: %i, Output: %i\n", robInput.rho, robOutput.rho);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.theta != robOutput.theta) {
			sprintf(smallStrBuffer,"Mismatch in theta. Input: %i, Output: %i\n", robInput.theta, robOutput.theta);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.driving_reference != robOutput.driving_reference) {
			sprintf(smallStrBuffer,"Mismatch in driving_reference. Input: %i, Output: %i\n", robInput.driving_reference, robOutput.driving_reference);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.use_cam_info != robOutput.use_cam_info) {
			sprintf(smallStrBuffer,"Mismatch in use_cam_info. Input: %i, Output: %i\n", robInput.use_cam_info, robOutput.use_cam_info);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.velocity_angular != robOutput.velocity_angular) {
			sprintf(smallStrBuffer,"Mismatch in velocity_angular. Input: %i, Output: %i\n", robInput.velocity_angular, robOutput.velocity_angular);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.debug_info != robOutput.debug_info) {
			sprintf(smallStrBuffer,"Mismatch in debug_info. Input: %i, Output: %i\n", robInput.debug_info, robOutput.debug_info);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.do_kick != robOutput.do_kick) {
			sprintf(smallStrBuffer,"Mismatch in do_kick. Input: %i, Output: %i\n", robInput.do_kick, robOutput.do_kick);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.do_chip != robOutput.do_chip) {
			sprintf(smallStrBuffer,"Mismatch in do_chip. Input: %i, Output: %i\n", robInput.do_chip, robOutput.do_chip);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.kick_chip_forced != robOutput.kick_chip_forced) {
			sprintf(smallStrBuffer,"Mismatch in kick_chip_forced. Input: %i, Output: %i\n", robInput.kick_chip_forced, robOutput.kick_chip_forced);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.kick_chip_power != robOutput.kick_chip_power) {
			sprintf(smallStrBuffer,"Mismatch in kick_chip_power. Input: %i, Output: %i\n", robInput.kick_chip_power, robOutput.kick_chip_power);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.velocity_dribbler != robOutput.velocity_dribbler) {
			sprintf(smallStrBuffer,"Mismatch in velocity_dribbler. Input: %i, Output: %i\n", robInput.velocity_dribbler, robOutput.velocity_dribbler);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.geneva_drive_state != robOutput.geneva_drive_state) {
			sprintf(smallStrBuffer,"Mismatch in geneva_drive_state. Input: %i, Output: %i\n", robInput.geneva_drive_state, robOutput.geneva_drive_state);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.cam_position_x != robOutput.cam_position_x) {
			sprintf(smallStrBuffer,"Mismatch in cam_position_x. Input: %i, Output: %i\n", robInput.cam_position_x, robOutput.cam_position_x);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.cam_position_y != robOutput.cam_position_y) {
			sprintf(smallStrBuffer,"Mismatch in cam_position_y. Input: %i, Output: %i\n", robInput.cam_position_y, robOutput.cam_position_y);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.cam_rotation != robOutput.cam_rotation) {
			sprintf(smallStrBuffer,"Mismatch in cam_rotation. Input: %i, Output: %i\n", robInput.cam_rotation, robOutput.cam_rotation);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}

		if(mismatch == 0) {
			TextOut("Alright. Everything matches. :)");
		}


		TextOut("\n\n\n");

		HAL_Delay(1000);
	}
}

void testAckPacking() {


	while(1) {
		roboAckData robInput;
		roboAckData robOutput;

		uint8_t roboAckPkt[23]; //larger than it needs to be..

		//filling struct with pseudo-random data
		robInput.roboID = (uint8_t) HAL_GetTick();
		robInput.wheelLeftFront = (uint8_t) HAL_GetTick();
		robInput.wheelRightFront = (uint8_t) HAL_GetTick();
		robInput.wheelLeftBack = (uint8_t) HAL_GetTick();
		robInput.wheelRightBack = (uint8_t) HAL_GetTick();
		robInput.genevaDriveState = (uint8_t) HAL_GetTick();
		robInput.xPosRobot = (int16_t) HAL_GetTick();
		robInput.yPosRobot = (int16_t) HAL_GetTick();
		robInput.rho = (int16_t) HAL_GetTick();
		robInput.theta = (int16_t) HAL_GetTick();
		robInput.orientation = (int16_t) HAL_GetTick();
		robInput.angularVelocity = (int16_t) HAL_GetTick();
		robInput.batteryState = (uint8_t) HAL_GetTick();
		robInput.ballSensor = (uint8_t) HAL_GetTick();
		robInput.xAcceleration = HAL_GetTick();
		robInput.yAcceleration = HAL_GetTick();
		robInput.angularRate = HAL_GetTick();

		//converting struct to packet
		roboAckDataToPacket(&robInput, roboAckPkt);

		//converting back to struct
		ackPacketToRoboAckData(roboAckPkt, 23, &robOutput);

		//comparing input with output
		uint8_t mismatch = 0;
		if(robInput.roboID != robOutput.roboID) {
			sprintf(smallStrBuffer,"Mismatch in roboID. Input: %i, Output: %i\n", robInput.roboID, robOutput.roboID);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.wheelLeftFront != robOutput.wheelLeftFront) {
			sprintf(smallStrBuffer,"Mismatch in wheelLeftFront. Input: %i, Output: %i\n", robInput.wheelLeftFront, robOutput.wheelLeftFront);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.wheelRightFront != robOutput.wheelRightFront) {
			sprintf(smallStrBuffer,"Mismatch in wheelRightFront. Input: %i, Output: %i\n", robInput.wheelRightFront, robOutput.wheelRightFront);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.wheelLeftBack != robOutput.wheelLeftBack) {
			sprintf(smallStrBuffer,"Mismatch in wheelLeftBack. Input: %i, Output: %i\n", robInput.wheelLeftBack, robOutput.wheelLeftBack);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.wheelRightBack != robOutput.wheelRightBack) {
			sprintf(smallStrBuffer,"Mismatch in wheelRightBack. Input: %i, Output: %i\n", robInput.wheelRightBack, robOutput.wheelRightBack);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.genevaDriveState != robOutput.genevaDriveState) {
			sprintf(smallStrBuffer,"Mismatch in genevaDriveState. Input: %i, Output: %i\n", robInput.genevaDriveState, robOutput.genevaDriveState);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.xPosRobot != robOutput.xPosRobot) {
			sprintf(smallStrBuffer,"Mismatch in xPosRobot. Input: %i, Output: %i\n", robInput.xPosRobot, robOutput.xPosRobot);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.yPosRobot != robOutput.yPosRobot) {
			sprintf(smallStrBuffer,"Mismatch in yPosRobot. Input: %i, Output: %i\n", robInput.yPosRobot, robOutput.yPosRobot);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.rho != robOutput.rho) {
			sprintf(smallStrBuffer,"Mismatch in xVel. Input: %i, Output: %i\n", robInput.rho, robOutput.rho);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.theta != robOutput.theta) {
			sprintf(smallStrBuffer,"Mismatch in yVel. Input: %i, Output: %i\n", robInput.theta, robOutput.theta);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.orientation != robOutput.orientation) {
			sprintf(smallStrBuffer,"Mismatch in orientation. Input: %i, Output: %i\n", robInput.orientation, robOutput.orientation);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.angularVelocity != robOutput.angularVelocity) {
			sprintf(smallStrBuffer,"Mismatch in angularVelocity. Input: %i, Output: %i\n", robInput.angularVelocity, robOutput.angularVelocity);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.batteryState != robOutput.batteryState) {
			sprintf(smallStrBuffer,"Mismatch in batteryState. Input: %i, Output: %i\n", robInput.batteryState, robOutput.batteryState);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.ballSensor != robOutput.ballSensor) {
			sprintf(smallStrBuffer,"Mismatch in ballSensor. Input: %i, Output: %i\n", robInput.ballSensor, robOutput.ballSensor);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.xAcceleration != robOutput.xAcceleration) {
			sprintf(smallStrBuffer,"Mismatch in xAcceleration. Input: %u, Output: %u\n", (unsigned int) robInput.xAcceleration, (unsigned int) robOutput.xAcceleration);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.yAcceleration != robOutput.yAcceleration) {
			sprintf(smallStrBuffer,"Mismatch in yAcceleration. Input: %u, Output: %u\n", (unsigned int) robInput.yAcceleration, (unsigned int) robOutput.yAcceleration);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}
		if(robInput.angularRate != robOutput.angularRate) {
			sprintf(smallStrBuffer,"Mismatch in angularRate. Input: %u, Output: %u\n", (unsigned int) robInput.angularRate, (unsigned int) robOutput.angularRate);
			TextOut(smallStrBuffer);
			mismatch = 1;
		}

		if(mismatch == 0) {
			TextOut("Alright. Everything matches. :)");
		}


		TextOut("\n\n\n");

		HAL_Delay(100);


	}
}

