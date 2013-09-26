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

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.pc.comm.NXTComm;
import static com.github.rosnxt.proxy.NXTProtocolConstants.*;

/**
 * Client (PC) side implementation of the protocol
 *
 * @author Federico Ferri
 *
 */
public class NXTProtocol {
	private final DataInputStream dis;
	private final DataOutputStream dos;
	private boolean run;
	private Thread readerThread;
	
	public NXTProtocol(NXTComm comm) {
		dis = new DataInputStream(comm.getInputStream());
		dos = new DataOutputStream(comm.getOutputStream());
		run = true;
		
		Thread readerThread = new Thread() {
			public void run() {
				while(run) {
					try {
						byte b = dis.readByte();
						if(b != HDR_DATA) {
							//System.out.println("warning: unrecognized data header: " + b);
							continue;
						}
						byte port = dis.readByte();
						byte slot = dis.readByte();
						byte type = dis.readByte();
						byte ni = dis.readByte();
						int iv[] = new int[ni];
						for(int i = 0; i < ni; i++) iv[i] = dis.readInt();
						byte nf = dis.readByte();
						float fv[] = new float[nf];
						for(int i = 0; i < nf; i++) fv[i] = dis.readFloat();
						onRawData(port, slot, type, iv, fv);
					} catch(IOException e) {
						e.printStackTrace();
						return;
					}
				}
			}
		};
		readerThread.start();
	}
	
	protected void onColorLightValue(byte port, int value) {
		
	}
	
	protected void onColor(byte port, int colorID) {
		
	}
	
	protected void onColor(byte port, int r, int g, int b) {
		
	}
	
	protected void onCompass(byte port, float degree) {
		
	}
	
	protected void onBatteryLevel(byte port, int levelMilliVolt) {
		
	}
	
	protected void onFreeMemory(byte port, int mem) {
		
	}
	
	protected void onButtons(byte port, boolean enter, boolean escape, boolean left, boolean right) {
		
	}
	
	protected void onAccelerometer(byte port, int x, int y, int z) {
		
	}
	
	protected void onGyroscope(byte port, float x, float y, float z) {
		
	}
	
	protected void onLightValue(byte port, int value) {
		
	}
	
	protected void onMotor(byte port, boolean isMoving, boolean isStalled, int tachoCount, int speed, int rotationSpeed, int limitAngle) {
		
	}
	
	protected void onMuxTouch(byte port, boolean b1, boolean b2, boolean b3, boolean b4) {
		
	}
	
	protected void onSound(byte port, int value) {
		
	}
	
	protected void onTouch(byte port, boolean b) {
		
	}
	
	protected void onUltrasonic(byte port, int range) {
		
	}
	
	protected void onRawData(byte port, byte slot, byte type, int iv[], float fv[]) {
		switch(type) {
		case TYPE_COLOR:
			switch(slot) {
			case 0:
				onColorLightValue(port, iv[0]);
				break;
			case 1:
				onColor(port, iv[0]);
				break;
			case 2:
				onColor(port, iv[0], iv[1], iv[2]);
				break;
			}
			break;
		case TYPE_COMPASS:
			switch(slot) {
			case 0:
				onCompass(port, fv[0]);
				break;
			}
			break;
		case TYPE_DIAGNOSTICS:
			switch(slot) {
			case 0:
				onBatteryLevel(port, iv[0]);
				break;
			case 1:
				onFreeMemory(port, iv[0]);
				break;
			case 2:
				onButtons(port, iv[0] != 0, iv[1] != 0, iv[2] != 0, iv[3] != 0);
				break;
			}
			break;
		case TYPE_IMU:
			switch(slot) {
			case 0:
				onAccelerometer(port, iv[0], iv[1], iv[2]);
				break;
			case 1:
				onGyroscope(port, fv[0], fv[1], fv[2]);
				break;
			}
			break;
		case TYPE_IRLINK:
			switch(slot) {
			}
			break;
		case TYPE_LIGHT:
			switch(slot) {
			case 0:
				onLightValue(port, iv[0]);
				break;
			}
			break;
		case TYPE_MOTOR:
			switch(slot) {
			case 0:
				onMotor(port, iv[0] != 0, iv[1] != 0, iv[2], iv[3], iv[4], iv[5]);
				break;
			}
			break;
		case TYPE_MUXTOUCH:
			switch(slot) {
			case 0:
				onMuxTouch(port, iv[0] != 0, iv[1] != 0, iv[2] != 0, iv[3] != 0);
				break;
			}
			break;
		case TYPE_SOUND:
			switch(slot) {
			case 0:
				onSound(port, iv[0]);
				break;
			}
			break;
		case TYPE_TOUCH:
			switch(slot) {
			case 0:
				onTouch(port, iv[0] != 0);
				break;
			}
			break;
		case TYPE_ULTRASONIC:
			switch(slot) {
			case 0:
				onUltrasonic(port, iv[0]);
				break;
			}
			break;
		}
	}
	
	public void setupPort(byte port, byte type) throws IOException {
		dos.writeByte(CMD_SETUP_PORT);
		dos.writeByte(port);
		dos.writeByte(type);
		dos.flush();
	}
	
	public void startPolling(byte port, byte slot, int period) throws IOException {
		dos.writeByte(CMD_START_POLLING);
		dos.writeByte(port);
		dos.writeByte(slot);
		dos.writeInt(period);
		dos.flush();
	}
	
	public void stopPolling(byte port, byte slot) throws IOException {
		dos.writeByte(CMD_STOP_POLLING);
		dos.writeByte(port);
		dos.writeByte(slot);
		dos.flush();
	}

	public void write(byte port, int iv[], float fv[]) throws IOException {
		dos.writeByte(CMD_WRITE);
		dos.writeByte(port);
		dos.writeByte((byte)iv.length);
		for(int i = 0; i < iv.length; i++)
			dos.writeInt(iv[i]);
		dos.writeByte((byte)fv.length);
		for(int i = 0; i < fv.length; i++)
			dos.writeFloat(fv[i]);
		dos.flush();
	}
	
	public void write(byte port, int iv[]) throws IOException {
		write(port, iv, new float[0]);
	}
	
	public void write(byte port, float fv[]) throws IOException {
		write(port, new int[0], fv);
	}

	public void shutDown() throws IOException {
		run = false;
		dos.writeByte(CMD_SHUT_DOWN);
		dos.flush();
		dis.close();
		dos.close();
		try {
			readerThread.join();
		} catch (InterruptedException e) {
		}
	}
	
	public void motorRotateTo(byte port, int angle) throws IOException {
		write(port, new int[]{MOTOR_ROTATE_TO, angle});
	}
	
	public void motorRotateTo(byte port, int angle, int speed) throws IOException {
		write(port, new int[]{MOTOR_ROTATE_TO, angle, speed});
	}
	
	public void motorRotateTo(byte port, int angle, int speed, int accel) throws IOException {
		write(port, new int[]{MOTOR_ROTATE_TO, angle, speed, accel});
	}
	
	public void motorRotateBy(byte port, int angle) throws IOException {
		write(port, new int[]{MOTOR_ROTATE_BY, angle});
	}
	
	public void motorRotateBy(byte port, int angle, int speed) throws IOException {
		write(port, new int[]{MOTOR_ROTATE_BY, angle, speed});
	}
	
	public void motorRotateBy(byte port, int angle, int speed, int accel) throws IOException {
		write(port, new int[]{MOTOR_ROTATE_BY, angle, speed, accel});
	}
	
	public void motorFloat(byte port) throws IOException {
		write(port, new int[]{MOTOR_FLOAT});
	}
	
	public void motorStop(byte port) throws IOException {
		write(port, new int[]{MOTOR_STOP});
	}

	public void motorSetSpeed(byte port, int speed) throws IOException {
		write(port, new int[]{MOTOR_SET_SPEED, speed});
	}
	
	public void motorSetAccel(byte port, int accel) throws IOException {
		write(port, new int[]{MOTOR_SET_ACCEL, accel});
	}
	
	public void motorSetStallTreshold(byte port, int error, int time) throws IOException {
		write(port, new int[]{MOTOR_SET_STALL_TRESHOLD, error, time});
	}
	
	public void motorForward(byte port) throws IOException {
		write(port, new int[]{MOTOR_FORWARD});
	}
	
	public void motorBackward(byte port) throws IOException {
		write(port, new int[]{MOTOR_BACKWARD});
	}
}
