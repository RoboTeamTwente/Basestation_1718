/*
 * test_packetSending.h
 *
 *  Created on: May 2, 2018
 *      Author: Ulf Stottmeister
 *
 * Description:
 *
 */

#ifndef TEST_PACKETSENDING_H_
#define TEST_PACKETSENDING_H_

#include "packing.h"
#include "myNRF24.h"
#include "TextOut.h"
#include "basestationNRF24.h"

//debug function to send packets in a loop. check the source for details. (doh!)
void sendPacketLoop();

#endif /* TEST_PACKETSENDING_H_ */
