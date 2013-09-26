package com.github.rosnxt.proxy;

public abstract class NXTProtocolConstants {
	// Commands (byte 0)
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
