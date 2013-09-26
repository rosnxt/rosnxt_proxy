package com.github.rosnxt.proxy;

import java.util.Set;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import static com.github.rosnxt.proxy.NXTProtocolConstants.*;

public class NXTProtocolString {
	private static BiMap<Byte, String> type = HashBiMap.create();
	static {
		type.put(Byte.valueOf(TYPE_COLOR), "color");
		type.put(Byte.valueOf(TYPE_COMPASS), "compass");
		type.put(Byte.valueOf(TYPE_DIAGNOSTICS), "diagnostics");
		type.put(Byte.valueOf(TYPE_IMU), "imu");
		type.put(Byte.valueOf(TYPE_IRLINK), "irlink");
		type.put(Byte.valueOf(TYPE_LIGHT), "light");
		type.put(Byte.valueOf(TYPE_MOTOR), "motor");
		type.put(Byte.valueOf(TYPE_MUXTOUCH), "muxtouch");
		type.put(Byte.valueOf(TYPE_SOUND), "sound");
		type.put(Byte.valueOf(TYPE_TOUCH), "touch");
		type.put(Byte.valueOf(TYPE_ULTRASONIC), "ultrasonic");
	};
	
	private static BiMap<Byte, String> port = HashBiMap.create();
	static {
		port.put(Byte.valueOf(PORT_MISC), "misc");
		port.put(Byte.valueOf(PORT_1), "s1");
		port.put(Byte.valueOf(PORT_2), "s2");
		port.put(Byte.valueOf(PORT_3), "s3");
		port.put(Byte.valueOf(PORT_4), "s4");
		port.put(Byte.valueOf(PORT_A), "a");
		port.put(Byte.valueOf(PORT_B), "b");
		port.put(Byte.valueOf(PORT_C), "c");
	}
	
	public static Byte typeFromString(String s) {
		return type.inverse().get(s);
	}
	
	public static String typeToString(Byte b) {
		return type.get(b);
	}
	
	public static Byte portFromString(String s) {
		return port.inverse().get(s);
	}
	
	public static String portToString(Byte b) {
		return port.get(b);
	}
	
	public static Set<String> portNames() {
		return port.values();
	}
}
