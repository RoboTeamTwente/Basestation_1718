/*
 * packing.h
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 *      (Co-)Author: Ulf Stottmeister, April 2018
 */

#ifndef PACKING_H_
#define PACKING_H_

#include <inttypes.h>







/*
 * do we have a use for this here?
typedef struct dataPacket {
  uint8_t robotID; // 0 to 15
  uint16_t robotVelocity; //between 0 and 4095mm/s
  uint16_t movingDirection; // resolution: 2pi/512 radians
  uint8_t rotationDirection; //0 = cw; 1 = ccw;
  uint16_t angularVelocity; //0 to 2047 deg/s
  uint8_t kickForce; // 0 to 255
  uint8_t forced; // 0 = normal kick; 1 = forced kick
  uint8_t chipper; // 0 = kicker; 1 = chipper
  uint8_t kick; // 0 = do not kick; 1 = kick according to forced and chipper
  uint8_t driblerDirection; // 0 = cw; 1 = ccw;
  uint8_t driblerSpeed; // between 0 and 7
} dataPacket;

struct ackPacket {
  uint8_t robotID;
  uint8_t succes;
};

extern dataPacket dataStruct;
*/

/*
 * A data struct which is easy to work with
 * when accessing variables.
 * It needs to be converted before it can be
 * transmitted, though.
 */

typedef struct roboData{
   uint8_t id:5;
   int16_t rho:11;
   int16_t theta:11;
   uint8_t driving_reference:1;
   uint8_t use_cam_info:1;
   int16_t velocity_angular:9;
   uint8_t debug_info:1;
   uint8_t do_kick:1;
   uint8_t do_chip:1;
   uint8_t kick_chip_forced:1;
   uint8_t kick_chip_power:8;
   uint8_t velocity_dribbler:8;
   uint8_t geneva_drive_state:3;
   uint16_t cam_position_x:13;
   uint16_t cam_position_y:13;
   uint16_t cam_rotation:11;
} roboData;

//between 11 and 23 Bytes, ideally
typedef struct roboAckData{
	//regular fields: 11 Bytes
	uint8_t roboID:4;
	uint8_t wheelLeftFront:1;
	uint8_t wheelRightFront:1;
	uint8_t wheelLeftBack:1;
	uint8_t wheelRightBack:1;
	uint8_t batteryState:1;
	uint8_t genevaDriveState:1;
	uint8_t rotatingDirection:1;
	int16_t xPosRobot:13;
	int16_t yPosRobot:13;
	int16_t xVel:11;
	int16_t yVel:11;
	int16_t orientation:11;
	uint16_t angularVelocity:10;
	uint8_t ballSensor:8;

	//extra fields (add 12 Bytes)
	float xAcceleration;
	float yAcceleration;
	float angularRate;

} roboAckData;


void createRobotPacket(int id, int robot_vel, int ang, uint8_t rot_cclockwise, int w_vel, uint8_t kick_force, uint8_t do_kick, uint8_t chip, uint8_t forced, uint8_t dribble_cclockwise, uint8_t dribble_vel, uint8_t* byteArr);
void robotDataToPacket(roboData input, uint8_t output[13]);


#endif /* PACKING_H_ */

