package com.github.rosnxt.proxy;

public class ProtocolConstants {
	public static final byte DEV_DUMMY =        0;
	public static final byte DEV_DIAGNOSTICS =  1;
	public static final byte DEV_MOTOR =        2;
	public static final byte DEV_TOUCH =        3;
	public static final byte DEV_SOUND =        4;
	public static final byte DEV_LIGHT =        5;
	public static final byte DEV_COLOR =        6;
	public static final byte DEV_ULTRASONIC =   7;
	public static final byte DEV_TOUCHMUX =     8;
	public static final byte DEV_IRLINK =       9;
	public static final byte DEV_DIMU =        10;
	public static final byte DEV_DCOMPASS =    11;
	
	public static final byte PORT_INTERNAL = 0;
	public static final byte PORT_S1 =       1; 
	public static final byte PORT_S2 =       2;
	public static final byte PORT_S3 =       3;
	public static final byte PORT_S4 =       4;
	public static final byte PORT_A =        5;
	public static final byte PORT_B =        6;
	public static final byte PORT_C =        7;
	
	public static final byte CMD_MOTOR_ROTATE_BY = 0;
	public static final byte CMD_MOTOR_ROTATE_TO = 1;
	public static final byte CMD_MOTOR_ROTATE = 2;
	public static final byte CMD_MOTOR_FLT = 3;
	public static final byte CMD_MOTOR_STOP = 4;
	public static final byte CMD_MOTOR_SET_SPEED = 5;
	public static final byte CMD_MOTOR_SET_ACCEL = 6;
	public static final byte CMD_MOTOR_SET_POWER = 7;
	public static final byte CMD_MOTOR_SET_STALL_THRESHOLD = 8;
	
	public static final byte CMD_IRLINK_SEND_COMBO_DIRECT = 0;
	public static final byte CMD_IRLINK_SEND_COMBO_PWM = 1;
	public static final byte CMD_IRLINK_SEND_EXTENDED = 2;
	public static final byte CMD_IRLINK_SEND_SINGLE_CST = 3;
	public static final byte CMD_IRLINK_SEND_SINGLE_PWM = 4;
	
	public static final byte DATA_DIAGNOSTICS_BATTERY_LEVEL = 0; // millivolts
	public static final byte DATA_DIAGNOSTICS_FREEMEMORY = 1; // bytes
	public static final byte DATA_DIAGNOSTICS_BTN_ENTER = 2;
	public static final byte DATA_DIAGNOSTICS_BTN_ESCAPE = 3;
	public static final byte DATA_DIAGNOSTICS_BTN_LEFT = 4;
	public static final byte DATA_DIAGNOSTICS_BTN_RIGHT = 5;
	
	public static final byte DATA_MOTOR_TACHO = 0;
	
	public static final byte DATA_TOUCH_STATUS = 0;
	
	public static final byte DATA_SOUND_LEVEL = 0;
	
	public static final byte DATA_LIGHT_LEVEL = 0;
	
	public static final byte DATA_COLOR_LEVEL = 0;
	public static final byte DATA_COLOR_ID = 1;
	public static final byte DATA_COLOR_RGB = 2;

	public static final byte DATA_ULTRASONIC_DISTANCE = 0;
	
	public static final byte DATA_TOUCHMUX_STATUS = 0;
	
	public static final byte DATA_DIMU_ACCEL = 0;
	public static final byte DATA_DIMU_GYRO = 1;
	
	public static final byte DATA_DCOMPASS_HEADING = 0;
}
