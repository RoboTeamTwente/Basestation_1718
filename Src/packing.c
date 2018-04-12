/*
 * packing.c
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 */

#include "packing.h"
#include <stdio.h>
#include <string.h> //we actually need that for memcpy -- not for strings or anything
/*
 * A new packet format
 * (not yet documented)
 * */


/*
 * Convert a struct with roboData to a Bytearray, which can be transmitted by the nRF module.
 * You will only use this function for creating packets for debugging purposes on the basestation.
 * In the final version, the computer already sends a read-to-transmit byte array.
 *
 */
void robotDataToPacket(roboData *input, uint8_t output[13]) {

	output[0] = (uint8_t) (  							// aaaaabbb
		(0b11111000 & (input->id << 3))                  // aaaaa000   5 bits; bits  4-0 to 7-3
	  | (0b00000111 & (input->rho >> 8))                 // 00000bbb  11 bits; bits 10-8 to 2-0
	);

	output[1] = (uint8_t) (  							// bbbbbbbb
		input->rho                                       // bbbbbbbb  11 bits; bits  7-0 to 7-0
	);

	output[2] = (uint8_t) (  							// cccccccc
		input->theta >> 3                                // cccccccc 11 bits; bits 10-8 to 7-0
	);

	output[3] = (uint8_t) (  							// cccde00g
		(0b11100000 & (input->theta << 5)) |             // ccc00000 11 bits; bits  2-0 to 7-5
		(0b00010000 & (input->driving_reference << 4)) | // 000d0000  1 bit ; bit     0 to   4
		(0b00001000 & (input->use_cam_info) << 3) |      // 0000e000  1 bit ; bit     0 to   3
		(0b00000001 & (input->velocity_angular >> 8))    // 0000000g  9 bits; bit     8 to   0
	);

	output[4] = (uint8_t) (  							// gggggggg
		input->velocity_angular                          // gggggggg  8 bits; bits  7-0 to 7-0
	);

	output[5] = (uint8_t) (								// 0000hijk
		(0b00001000 & (input->debug_info << 3)) |        // 0000h000  1 bit ; bit     0 to   3
		(0b00000100 & (input->do_kick << 2)) |           // 00000i00  1 bit ; bit     0 to   2
		(0b00000010 & (input->do_chip << 1)) |           // 000000j0  1 bit ; bit     0 to   1
		(0b00000001 & (input->kick_chip_forced))    // 0000000k  1 bit ; bit     0 to   0
	);

	output[6] = (uint8_t) (  							// mmmmmmmm
		input->kick_chip_power                           // mmmmmmmm  8 bits; bits  7-0 to 7-0
	);

	output[7] = (uint8_t) (  							// nnnnnnnn
		input->velocity_dribbler                         // nnnnnnnn  8 bits; bits  7-0 to 7-0
	);

	output[8] = (uint8_t) ( 							// pppqqqqq
		(0b11100000 & (input->geneva_drive_state << 5)) |// ppp00000  3 bits; bits  2-0 to 7-5
		(0b00011111 & (input->cam_position_x >> 8 ))     // 000qqqqq 13 bits; bits 12-8 to 4-0
	);

	output[9] = (uint8_t) (  							// qqqqqqqq
		input->cam_position_x                            // qqqqqqqq 13 bits; bits  7-0 to 7-0
	);

	output[10] = (uint8_t) ( 							// rrrrrrrr
		input->cam_position_y >> 5                       // rrrrrrrr 13 bits; bits 12-5 to 7-0
	);

	output[11] = (uint8_t) (							// rrrrrsss
		(0b11111000 & (input->cam_position_y << 3)) |    // rrrrr000 13 bits; bits  4-0 to 7-3
		(0b00000111 & (input->cam_rotation >> 8))        // 00000sss 11 bits; bits 10-8 to 2-0
	);

	output[12] = (uint8_t) ( 							// ssssssss
		input->cam_rotation                              // ssssssss 11 bits; bits  7-0 to 7-0
	);
}

/*
 * Create a roboData structure from a given Bytearray.
 * This is used by the robot to convert a received nRF packet into a struct with named variables.
 */
void packetToRoboData(uint8_t input[13], roboData *output) {
	/*
	output[0] aaaaabbb
	output[1] bbbbbbbb
	output[2] cccccccc
	output[3] cccde00g
	output[4] gggggggg
	output[5] 0000hijk
	output[6] mmmmmmmm
	output[7] nnnnnnnn
	output[8] pppqqqqq
	output[9] qqqqqqqq
	output[10] rrrrrrrr
	output[11] rrrrrsss
	output[12] ssssssss
	 */
	output->id = input[0]>>3; //a
	output->rho = (input[0]&0b111)<<8; //b
	output->rho |= input[1]; //b
	output->theta = input[2]<<3; //c
	output->theta |= (input[3]>>5)&0b111; //c
	output->driving_reference = (input[3]>>4)&1; //d
	output->use_cam_info = (input[3]>>3)&1; //e
	output->velocity_angular = (input[3]&1) << 8; //g
	output->velocity_angular |= input[4]; //g
	output->debug_info = (input[5]>>3)&1; //h
	output->do_kick = (input[5]>>2)&1; //i
	output->do_chip = (input[5]>>1)&1; //j
	output->kick_chip_forced = input[5]&1; //k
	output->kick_chip_power = input[6]; //m
	output->velocity_dribbler = input[7]; //n
	output->geneva_drive_state = (input[8]>>5)&0b111; //p
	output->cam_position_x = (input[8]&0b11111)<<8; //q
	output->cam_position_x |= input[9]; //q
	output->cam_position_y = input[10] << 5; //r
	output->cam_position_y |= (input[11]>>3)&0b11111; //r
	output->cam_rotation = (input[11]&0b111) << 8; //s
	output->cam_rotation |= input[12]; //s

}


/*
 * For the Robot ACKs we use the following packet definition
 *

		Character   Description                 Values          Represented values    Units       Bits    Comment
		a           Robot ID                    [0,15]          [0,15]                -              4    -
		b           Left front wheel state      [0,1]           {true,false}          -              1    Indicates whether the left front wheel functions
		c           Right front wheel state     [0,1]           {true,false}          -              1    Indicates whether the right front wheel functions
		d           Left back wheel state       [0,1]           {true,false}          -              1    Indicates whether the left back wheel functions
		e           Right back wheel state      [0,1]           {true,false}          -              1    Indicates whether the right back wheel functions
		f           Battery state               [0,1]           {true,false}          -              1    States whether the battery is nearing depletion
		g           Geneva drive state          [0,1]           {true,false}          -              1    Indicates whether the Geneva drive functions
		h           Rotating direction          [0,1]           {true,false}          -              1    True means clockwise, false means counterclockwise
		i           x position robot            [-4096,4095]    [-1024,1023]          0.25cm        13    -
		j           y position robot            [-4096,4095]    [-1024,1023]          0.25cm        13    -
		k           x velocity robot            [-1024,1023]    [-5120,5115]          2.5mm/s       11    See units: multiply value by 2.5 for the speed in mm/s
		m           y velocity robot            [-1024,1023]    [-5120,5115]          2.5mm/s       11    See units: multiply value by 2.5 for the speed in mm/s
		n           Orientation                 [-1024,1023]    [-pi,pi>              0.00307rad    11    2048 possible angles. Intervals of ~0.00307 rad
		p           Angular velocity            [0,1023]        [0,8*2pi]             0.049rad/s    10    -
		q           Ball sensor                 [0,255]         {true,false}          -              8    Can be used to specify where on the dribbler the ball is located. For now a non-zero value represents the presence of the ball

		Extra
		r           Acceleration x              See float32     See float32           m/s/s         32    -
		s           Acceleration y              See float32     See float32           m/s/s         32    -
		t           Angular rate                See float32     See float32           m/s/s         32    -


	===== Packet received from the robot =====
		Byte      Config
		 0        aaaabcde
		 1        fghiiiii
		 2        iiiiiiii
		 3        jjjjjjjj
		 4        jjjjjkkk
		 5        kkkkkkkk
		 6        mmmmmmmm
		 7        mmmnnnnn
		 8        nnnnnnpp
		 9        pppppppp
		10        qqqqqqqq

		Extra
		11        rrrrrrrr
		12        rrrrrrrr
		13        rrrrrrrr
		14        rrrrrrrr
		15        ssssssss
		16        ssssssss
		17        ssssssss
		18        ssssssss
		19        tttttttt
		20        tttttttt
		21        tttttttt
		22        tttttttt

 */




/*
 * First, fill the fields on a roboAckData struct.
 * Then convert that struct into a Bytearray by using this function.
 * The result can be used as an ACK payload to transmit it over air.
 */

void roboAckDataToPacket(roboAckData *input, uint8_t output[23]) {
	output[0]  = (uint8_t) (input->roboID&0b1111)<<4; //a
	output[0] |= (uint8_t) (input->wheelLeftFront&1)<<3; //b
	output[0] |= (uint8_t) (input->wheelRightFront&1)<<2; //c
	output[0] |= (uint8_t) (input->wheelLeftBack&1)<<1; //d
	output[0] |= (uint8_t) (input->wheelRightBack&1); //e

	output[1]  = (uint8_t) (input->batteryState&1)<<7; //f
	output[1] |= (uint8_t) (input->genevaDriveState&1)<<6; //g
	output[1] |= (uint8_t) (input->rotatingDirection&1)<<5; //h
	output[1] |= (uint8_t) (input->xPosRobot>>8); //i

	output[2]  = (uint8_t) (input->xPosRobot&0xff); //i

	output[3]  = (uint8_t) (input->yPosRobot>>5); //j

	output[4]  = (uint8_t) (input->yPosRobot&0b11111)<<3; //j
	output[4] |= (uint8_t) (input->xVel>>8)&0b111; //k

	output[5]  = (uint8_t) (input->xVel&0xff); //k

	output[6]  = (uint8_t) (input->yVel>>3); //m

	output[7]  = (uint8_t) (input->yVel&0xff)<<5; //m
	output[7]  = (uint8_t) (input->orientation>>6)&0b11111; //n

	output[8]  = (uint8_t) (input->orientation&0b111111)<<2; //n
	output[8]  = (uint8_t) (input->angularVelocity>>8)&0b11; //p

	output[9]  = (uint8_t) (input->angularVelocity&0xff); //p

	output[10]  = (uint8_t) input->ballSensor; //q

/*
 * So, the following code is rather hacky.
 * I think, the only way to make it more reader-friendly is by using unions
 * which define a byte array and a float in the same memory range.
 *
 * I will describe the memcpy solution:
 * Let's take this line as an example:
 *    memcpy(&output[11], (void*)(&input.xAcceleration), 4); //r
 *
 * You might remember memcpy from copying strings. It copies one string to another string (actually: char arrays).
 * It does the copy process byte per byte (char per char -- it's the same).
 * The arguments of memcpy() are: memcpy(output, input, amountOfBytes);
 * Output and Input are pointers to memory sections which contain several bytes of data.
 * In our case we chose output[11] which is the 12th byte of our output[] byte array.
 * As input we use a rather weird construction: (void*)(&input.xAcceleration).
 * Since fragment of code does the following: we take the address of the float variable input.xAcceleration.
 * That float variable is then type casted to "void pointer".
 * With a cast to void we invoke that the float will be interpreted as a floating point variable.
 * Instead we are accessing the raw bytes of the 32-Bit float variable as it is in memory.

 * With the last argument we tell the memcpy() function to copy 4 bytes.
 * When it has copied one byte, it will advance both addresses (input and output) by one byte
 * for the following copy operation, which will then copy the next 8 Bits from the float to output[12] and so on..
 *
 * And that's pretty much it. Three lines of code, 33 lines of comments. Hope you like it. ~ Ulf Stottmeister
 */
    memcpy(&output[11], (void*)(&(input->xAcceleration)), 4); //r
    memcpy(&output[15], (void*)(&(input->yAcceleration)), 4); //s
    memcpy(&output[19], (void*)(&(input->angularRate)), 4); //t


}

/*
 * We would actually just pass the raw ack packet to the computer and let the computer handle the unwrapping.
 * But for completeness and for debug purposes it is nice to unwrap the ACK packet on the Basestation board
 * itself. In that way we can test and debug with just the Basestation and a serial monitor.
 *
 * ACK packets can be either 11 Bytes or 23 Bytes long. That depends on whether the Basestation requested
 * additional fields buy setting the debug_info flag in an earlier robo packet.
 */
void ackPacketToRoboAckData(uint8_t input[23], uint8_t packetlength, roboAckData *output) {
	//input is now specified as an array of size 23. Note that there are also ACK packets of the length 11.
	//You need to use packetlength to know which range of the array contains useful information.
	//The attempt of accessing input[11] to input[22] for a short ACK packet will yield garbage data.

	output->roboID = input[0]>>4; //a
	output->wheelLeftFront = (input[0]>>3)&1; //b
	output->wheelRightFront = (input[0]>>2)&1; //c
	output->wheelLeftBack = (input[0]>>1)&1; //d
	output->wheelRightBack = input[0]&1; //e

	output->batteryState = (input[1]>>7)&1; //f
	output->genevaDriveState = (input[1]>>6)&1; //g
	output->rotatingDirection = (input[1]>>5)&1;  //h
	output->xPosRobot = (input[1]&0b11111)<<8; //i

	output->xPosRobot |= input[2]; //i

	output->yPosRobot = input[3]<<3; //j

	output->yPosRobot |= input[4]>>3; //j
	output->xVel = (input[4]&0b111)<<8; //k

	output->xVel |= input[5]; //k

	output->yVel = input[6]<<5; //m

	output->yVel |= input[7]>>5; //m
	output->orientation = (input[7]&0b11111)<<6; //n

	output->orientation |= (input[8]>>2); //n
	output->angularVelocity = (input[8]&0b11)<<8; //p

	output->angularVelocity |= input[9]; //p

	output->ballSensor = input[10]; //q

	if(packetlength < 23)
		return;

	//converting 4 Bytes to 32 Bit floats. See documentation in roboAckDataToPacket().
    memcpy((void*)(&(output->xAcceleration)), &input[11], 4); //r
    memcpy((void*)(&(output->yAcceleration)), &input[15], 4); //s
    memcpy((void*)(&(output->angularRate)), &input[19], 4); //t

}
