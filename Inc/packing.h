/*
 * packing.h
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 */

#ifndef PACKING_H_
#define PACKING_H_



#endif /* PACKING_H_ */

#include "stm32f3xx_hal.h"
#include "TextOut.h"


void createRobotPacket(int id, int robot_vel, int ang, uint8_t rot_cclockwise, int w_vel, uint8_t kick_force, uint8_t do_kick, uint8_t chip, uint8_t forced, uint8_t dribble_cclockwise, uint8_t dribble_vel, uint8_t* byteArr);

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
