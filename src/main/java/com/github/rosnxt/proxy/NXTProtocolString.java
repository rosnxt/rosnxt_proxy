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

import java.util.Set;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import static com.github.rosnxt.proxy.NXTProtocolConstants.*;

/**
 * To/from string conversions of some protocol constants
 *
 * @author Federico Ferri
 *
 */
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
