/*
 * Copyright (c) 2013, Federico Ferri
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *   Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 *   Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 *   Neither the name of the {organization} nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.github.rosnxt.proxy;

/**
 * Constants used for communication between the ROS-NXT proxy and
 * the ROS-responder running on the NXT brick.
 *
 * @author Federico Ferri
 *
 */
public abstract class NXTProtocolConstants {
	public static final byte CMD_SETUP_PORT    = 1;
	public static final byte CMD_START_POLLING = 2;
	public static final byte CMD_STOP_POLLING  = 3;
	public static final byte CMD_WRITE         = 4;
	public static final byte CMD_SHUT_DOWN     = 5;

	public static final byte PORT_MISC         = 0;
	public static final byte PORT_1            = 1;
	public static final byte PORT_2            = 2;
	public static final byte PORT_3            = 3;
	public static final byte PORT_4            = 4;
	public static final byte PORT_A            = 5;
	public static final byte PORT_B            = 6;
	public static final byte PORT_C            = 7;
	
	public static final byte TYPE_COLOR        = 1;
	public static final byte TYPE_COMPASS      = 2;
	public static final byte TYPE_IMU          = 3;
	public static final byte TYPE_IRLINK       = 4;
	public static final byte TYPE_LIGHT        = 5;
	public static final byte TYPE_MUXTOUCH     = 6;
	public static final byte TYPE_SOUND        = 7;
	public static final byte TYPE_TOUCH        = 8;
	public static final byte TYPE_ULTRASONIC   = 9;
	public static final byte TYPE_DIAGNOSTICS  = 10;
	public static final byte TYPE_MOTOR        = 11;
	
	public static final byte HDR_DATA          = 101;
	
	public static final byte MOTOR_ROTATE_BY = 1;
	public static final byte MOTOR_ROTATE_TO = 2;
	public static final byte MOTOR_FLOAT = 3;
	public static final byte MOTOR_STOP = 4;
	public static final byte MOTOR_SET_SPEED = 5;
	public static final byte MOTOR_SET_ACCEL = 6;
	public static final byte MOTOR_SET_STALL_TRESHOLD = 7;
	public static final byte MOTOR_FORWARD = 8;
	public static final byte MOTOR_BACKWARD = 9;
	
	public static final byte IRLINK_COMBO_DIRECT = 1;
	public static final byte IRLINK_COMBO_PWM = 2;
	public static final byte IRLINK_EXTENDED = 3;
	public static final byte IRLINK_SINGLE_CST = 4;
	public static final byte IRLINK_SINGLE_PWM = 5;
}
