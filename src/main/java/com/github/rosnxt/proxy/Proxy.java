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

import java.io.IOException;
import java.util.Map;

import lejos.pc.comm.NXTComm;
import lejos.pc.comm.NXTCommException;
import lejos.pc.comm.NXTCommFactory;
import lejos.pc.comm.NXTInfo;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NodeNameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import rosnxt_msgs.*;

import static com.github.rosnxt.proxy.ProtocolConstants.*;

/**
 * ROS node acting as a proxy for communication with the ROS-responder
 * runngin on the NXT brick.
 *
 * @author Federico Ferri
 *
 */
public class Proxy implements NodeMain {
	private NXTComm comm;
	private NXTProtocol proto;
	
	private Publisher<Accelerometer> pubAccelerometer = null;
	private Publisher<Battery> pubBattery = null;
	private Publisher<Buttons> pubButtons = null;
	private Publisher<Color> pubColor = null;
	private Publisher<ColorRGB> pubColorRGB = null;
	private Publisher<Compass> pubCompass = null;
	private Publisher<Gyroscope> pubGyroscope = null;
	private Publisher<Light> pubLight = null;
	private Publisher<Memory> pubMemory = null;
	private Publisher<Motor> pubMotor = null;
	private Publisher<Sound> pubSound = null;
	private Publisher<Touch> pubTouch = null;
	private Publisher<Ultrasonic> pubUltrasonic = null;
	
	private Subscriber<MotorCmd> subMotorCmd = null;
	private Subscriber<MotorSetStallTreshold> subMotorSetStallTreshold = null;
	private Subscriber<IRLinkCmd> subIRLinkCmd = null;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("nxt_proxy");
	}

	@Override
	public void onStart(final ConnectedNode node) {
		ParameterTree params = node.getParameterTree();
		String brickName = params.getString("~brick_name");
		if(brickName == null) {
			node.getLog().error("Brick name must be set (brick_name)");
			return;
		} else {
			node.getLog().info("Brick name is: " + brickName);
		}
		
		String connectionMethod = params.getString("~connection_method");
		if(connectionMethod == null || !(connectionMethod.equals("bt") || connectionMethod.equals("usb"))) {
			node.getLog().error("Connection must be set to either bt or usb (connection_method)");
			return;
		} else {
			node.getLog().info("Connection method: " + connectionMethod);
		}

		try {
			if(connectionMethod.equals("bt")) {
				node.getLog().info("Creating Bluetooth comm...");
				comm = NXTCommFactory.createNXTComm(NXTCommFactory.BLUETOOTH);
			} else {
				node.getLog().info("Creating USB comm...");
				comm = NXTCommFactory.createNXTComm(NXTCommFactory.USB);
			}
		} catch (NXTCommException e) {
			if(connectionMethod.equals("bt")) {
				node.getLog().error("Cannot get Bluetooth comm", e);
			} else {
				node.getLog().error("Cannot get USB comm", e);
			}
			return;
		}
		
		node.getLog().info("Searching device with name " + brickName + "...");
		try {
			NXTInfo is[] = comm.search(brickName);
			if(is.length == 0) {
				node.getLog().error("No device found with name " + brickName + "!");
				return;
			} else if(!comm.open(is[0])) {
				node.getLog().error("Open device '" + brickName + "' failed");
				return;
			}
		} catch (NXTCommException e) {
			node.getLog().error("Cannot search for device", e);
			return;
		}
		
		node.getLog().info("Opened device " + brickName);
		
		DataInputStream inputStream = new DataInputStream(comm.getInputStream());
		DataOutputStream outputStream = new DataOutputStream(comm.getOutputStream());

		boolean run = true;

		Thread readerThread = new Thread() {
			public void run() {
				while(run) {
					try {
						byte device = inputStream.readByte();
						byte port = inputStream.readByte();
						byte type = inputStream.readByte();
						byte len = inputStream.readByte();
						boolean l1, l2, l3, l4; byte b1, b2, b3, b4; int i1, i2, i3; float f1, f2, f3;
						switch(device) {
						case DEV_DIAGNOSTICS:
							switch(type) {
							case DATA_DIAGNOSTICS_BATTERY_LEVEL:
								i1 = inputStream.readInt();
								if(pubBattery != null) {
									Battery m = pubBattery.newMessage();
									m.setPort(port);
									m.setLevelMilliVolts(i1);
									pubBattery.publish(m);
								}
								break;
							case DATA_DIAGNOSTICS_FREEMEMORY:
								i1 = inputStream.readInt();
								if(pubMemory != null) {
									Memory m = pubMemory.newMessage();
									m.setPort(port);
									m.setFreeMemory(i1);
									pubMemory.publish(m);
								}
								break;
							case DATA_DIAGNOSTICS_BTN_ENTER:
							case DATA_DIAGNOSTICS_BTN_ESCAPE:
							case DATA_DIAGNOSTICS_BTN_LEFT:
							case DATA_DIAGNOSTICS_BTN_RIGHT:
								l1 = inputStream.readBoolean();
								if(pubButtons != null) {
									Buttons m = pubButtons.newMessage();
									m.setPort(port);
									//m.setEnter(enter ? (byte)1 : 0);
									//m.setEscape(escape ? (byte)1 : 0);
									//m.setLeft(left ? (byte)1 : 0);
									//m.setRight(right ? (byte)1 : 0);
									pubButtons.publish(m);
								}
								break;
							}
							break;
						case DEV_MOTOR:
							switch(type) {
							case DATA_MOTOR_TACHO:
								i1 = inputStream.readInt();
								if(pubMotor != null) {
									Motor m = pubMotor.newMessage();
									m.setPort(port);
									m.setTacho(i1);
									pubMotor.publish(m);
								}
								break;
							}
							break;
						case DEV_TOUCH:
							switch(type) {
							case DATA_TOUCH_STATUS:
								l1 = inputStream.readBoolean();
								if(pubTouch != null) {
									Touch m = pubTouch.newMessage();
									m.setPort(port);
									m.setButton((byte)0);
									m.setPressed(l1 ? (byte)1 : (byte)0);
									pubTouch.publish(m);
								}
								break;
							}
							break;
						case DEV_SOUND:
							switch(type) {
							case DATA_SOUND_LEVEL:
								i1 = inputStream.readInt();
								if(pubSound != null) {
									Sound m = pubSound.newMessage();
									m.setPort(port);
									m.setLevel(i1);
									pubSound.publish(m);
								}
								break;
							}
							break;
						case DEV_LIGHT:
							switch(type) {
							case DATA_LIGHT_LEVEL:
								i1 = inputStream.readInt();
								if(pubLight != null) {
									Light m = pubLight.newMessage();
									m.setPort(port);
									m.setLightValue(i1);
									pubLight.publish(m);
								}
								break;
							}
							break;
						case DEV_COLOR:
							switch(type) {
							case DATA_COLOR_LEVEL:
								i1 = inputStream.readInt();
								if(pubLight != null) {
									Light m = pubLight.newMessage();
									m.setPort(port);
									m.setLightValue(i1);
									pubLight.publish(m);
								}
								break;
							case DATA_COLOR_ID:
								i1 = inputStream.readInt();
								if(pubColor != null) {
									Color m = pubColor.newMessage();
									m.setPort(port);
									m.setColorId((byte)i1);
									pubColor.publish(m);
								}
								break;
							case DATA_COLOR_RGB:
								b1 = inputStream.readByte();
								b2 = inputStream.readByte();
								b3 = inputStream.readByte();
								if(pubColorRGB != null) {
									ColorRGB m = pubColorRGB.newMessage();
									m.setPort(port);
									m.setR((byte)b1);
									m.setG((byte)b2);
									m.setB((byte)b3);
									pubColorRGB.publish(m);
								}
								break;
							}
							break;
						case DEV_ULTRASONIC:
							switch(type) {
							case DATA_ULTRASONIC_DISTANCE:
								i1 = inputStream.readInt();
								if(pubUltrasonic != null) {
									Ultrasonic m = pubUltrasonic.newMessage();
									m.setPort(port);
									m.setDistance(i1);
									pubUltrasonic.publish(m);
								}
								break;
							}
							break;
						case DEV_TOUCHMUX:
							switch(type) {
							case DATA_TOUCHMUX_STATUS:
								l1 = inputStream.readBoolean();
								l2 = inputStream.readBoolean();
								l3 = inputStream.readBoolean();
								l4 = inputStream.readBoolean();
								if(pubTouch != null) {
									Touch m = pubTouch.newMessage();
									m.setPort(port);
									m.setButton((byte)0);
									m.setPressed(l1 ? (byte)1 : (byte)0);
									pubTouch.publish(m);
									Touch m = pubTouch.newMessage();
									m.setPort(port);
									m.setButton((byte)1);
									m.setPressed(l2 ? (byte)1 : (byte)0);
									pubTouch.publish(m);
									Touch m = pubTouch.newMessage();
									m.setPort(port);
									m.setButton((byte)2);
									m.setPressed(l3 ? (byte)1 : (byte)0);
									pubTouch.publish(m);
									Touch m = pubTouch.newMessage();
									m.setPort(port);
									m.setButton((byte)3);
									m.setPressed(l4 ? (byte)1 : (byte)0);
									pubTouch.publish(m);
								}
								break;
							}
							break;
						case DEV_IRLINK:
							break;
						case DEV_DIMU:
							switch(type) {
							case DATA_DIMU_ACCEL:
								i1 = inputStream.readInt();
								i2 = inputStream.readInt();
								i3 = inputStream.readInt();
								if(pubAccelerometer != null) {
									Accelerometer m = pubAccelerometer.newMessage();
									m.setPort(port);
									m.setX(i1);
									m.setY(i2);
									m.setZ(i3);
									pubAccelerometer.publish(m);
								}
								break;
							case DATA_DIMU_GYRO:
								f1 = inputStream.readFloat();
								f2 = inputStream.readFloat();
								f3 = inputStream.readFloat();
								if(pubGyroscope != null) {
									Gyroscope m = pubGyroscope.newMessage();
									m.setPort(port);
									m.setX(f1);
									m.setY(f2);
									m.setZ(f3);
									pubGyroscope.publish(m);
								}
								break;
							}
							break;
						case DEV_DCOMPASS:
							switch(type) {
							case DATA_DCOMPASS_HEADING:
								f1 = inputStream.readFloat();
								if(pubCompass != null) {
									Compass m = pubCompass.newMessage();
									m.setPort(port);
									m.setDegrees(f1);
									pubCompass.publish(m);
								}
								break;
							}
							break;
						}
					} catch(IOException e) {
						e.printStackTrace();
						run = false;
						return;
					}
				}
			}
		};
		readerThread.start();

		setupPorts(node, params);
	}

	protected static final String portNames = new String[]{"internal", "s1", "s2", "s3", "s4", "a", "b", "c"};

	protected static byte port(String portStr) {
		if(portStr.equals("internal")) return PORT_INTERNAL;
		if(portStr.equals("s1")) return PORT_S1;
		if(portStr.equals("s2")) return PORT_S2;
		if(portStr.equals("s3")) return PORT_S3;
		if(portStr.equals("s4")) return PORT_S4;
		if(portStr.equals("a")) return PORT_A;
		if(portStr.equals("b")) return PORT_B;
		if(portStr.equals("c")) return PORT_C;
		return -1;
	}	

	protected static byte device(String deviceStr) {
		if(deviceStr.equals("dummy")) return DEV_DUMMY;
		if(deviceStr.equals("diagnostics")) return DEV_DIAGNOSTICS;
		if(deviceStr.equals("motor")) return DEV_MOTOR;
		if(deviceStr.equals("touch")) return DEV_TOUCH;
		if(deviceStr.equals("sound")) return DEV_SOUND;
		if(deviceStr.equals("light")) return DEV_LIGHT;
		if(deviceStr.equals("color")) return DEV_COLOR;
		if(deviceStr.equals("ultrasonic")) return DEV_ULTRASONIC;
		if(deviceStr.equals("touchmux")) return DEV_TOUCHMUX;
		if(deviceStr.equals("irlink")) return DEV_IRLINK;
		if(deviceStr.equals("dimu")) return DEV_DIMU;
		if(deviceStr.equals("dcompass")) return DEV_DCOMPASS;
		return -1;
	}

	protected void cmdSystemSetDeviceType(byte port, byte type) {
		outputStream.writeByte(DEV_SYSTEM);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_SYSTEM_SET_DEVICE_TYPE);
		outputStream.writeByte(1);
		outputStream.writeByte(type);
		outputStream.flush()
	}

	protected void cmdSystemSetDeviceType(byte port, byte subport, int period) {
		outputStream.writeByte(DEV_SYSTEM);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_SYSTEM_SET_POLL_PERIOD);
		outputStream.writeByte(1 + Integer.SIZE/Byte.SIZE);
		outputStream.writeByte(subport);
		outputStream.writeInt(period);
		outputStream.flush()
	}

	protected void cmdMotorRotate(byte port, byte direction) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_ROTATE);
		outputStream.writeByte(1);
		outputStream.writeByte(direction);
		outputStream.flush()
	}

	protected void cmdMotorRotateBy(byte port, int angle) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_ROTATE_BY);
		outputStream.writeByte(Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(angle);
		outputStream.flush()
	}

	protected void cmdMotorRotateTo(byte port, int angle) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_ROTATE_TO);
		outputStream.writeByte(Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(angle);
		outputStream.flush()
	}

	protected void cmdMotorFlt(byte port) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_FLT);
		outputStream.writeByte(0);
		outputStream.flush()
	}

	protected void cmdMotorStop(byte port) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_STOP);
		outputStream.writeByte(0);
		outputStream.flush()
	}

	protected void cmdMotorSetSpeed(byte port, int speed) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_SET_SPEED);
		outputStream.writeByte(Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(speed);
		outputStream.flush()
	}

	protected void cmdMotorSetAccel(byte port, int accel) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_SET_ACCEL);
		outputStream.writeByte(Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(accel);
		outputStream.flush()
	}

	protected void cmdMotorSetStallTreshold(byte port, int error, int time) {
		outputStream.writeByte(DEV_MOTOR);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_MOTOR_SET_STALL_TRESHOLD);
		outputStream.writeByte(2*Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(error);
		outputStream.writeInt(time);
		outputStream.flush()
	}

	protected void cmdIRLinkSendComboDirect(byte port, int channel, int opA, int opB) {
		outputStream.writeByte(DEV_IRLINK);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_IRLINK_SEND_COMBO_DIRECT);
		outputStream.writeByte(3*Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(channel);
		outputStream.writeInt(opA);
		outputStream.writeInt(opB);
		outputStream.flush()
	}

	protected void cmdIRLinkSendComboPWM(byte port, int channel, int opA, int opB) {
		outputStream.writeByte(DEV_IRLINK);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_IRLINK_SEND_COMBO_PWM);
		outputStream.writeByte(3*Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(channel);
		outputStream.writeInt(opA);
		outputStream.writeInt(opB);
		outputStream.flush()
	}

	protected void cmdIRLinkSendExtended(byte port, int channel, int opA) {
		outputStream.writeByte(DEV_IRLINK);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_IRLINK_SEND_EXTENDED);
		outputStream.writeByte(2*Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(channel);
		outputStream.writeInt(opA);
		outputStream.flush()
	}

	protected void cmdIRLinkSendSingleCST(byte port, int channel, int opA, int opB) {
		outputStream.writeByte(DEV_IRLINK);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_IRLINK_SEND_SINGLE_CST);
		outputStream.writeByte(3*Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(channel);
		outputStream.writeInt(opA);
		outputStream.writeInt(opB);
		outputStream.flush()
	}

	protected void cmdIRLinkSendSinglePWM(byte port, int channel, int opA, int opB) {
		outputStream.writeByte(DEV_IRLINK);
		outputStream.writeByte(port);
		outputStream.writeByte(CMD_IRLINK_SEND_SINGLE_PWM);
		outputStream.writeByte(3*Integer.SIZE/Byte.SIZE);
		outputStream.writeInt(channel);
		outputStream.writeInt(opA);
		outputStream.writeInt(opB);
		outputStream.flush()
	}

	protected void setupPorts(final ConnectedNode node, ParameterTree params) {
		for(String portName : portNames) {
			String paramName = "~port/" + portName + "/type";
			if(params.has(paramName)) {
				final String t = params.getString(paramName);
				final byte port = port(portName);
				final byte type = type(t);
				try {
					proto.setupPort(port, type);
				} catch (IOException e) {
					node.getLog().info("setup port '" + portName + "': io error", e);
				}
				for(int i = 0; i < 9; i++) {
					String pp = "~port/" + portName + "/slot/" + i + "/pollPeriod";
					if(params.has(pp)) {
						switch(type) {
						case TYPE_COLOR:
							switch(i) {
							case 0:
								if(pubLight == null)
									pubLight = node.newPublisher("light", Light._TYPE);
								break;
							case 1:
								if(pubColor == null)
									pubColor = node.newPublisher("color", Color._TYPE);
								break;
							case 2:
								if(pubColorRGB == null)
									pubColorRGB = node.newPublisher("colorRGB", ColorRGB._TYPE);
								break;
							}
							break;
						case TYPE_COMPASS:
							switch(i) {
							case 0:
								if(pubCompass == null)
									pubCompass = node.newPublisher("compass", Compass._TYPE);
								break;
							}
							break;
						case TYPE_DIAGNOSTICS:
							switch(i) {
							case 0:
								if(pubBattery == null)
									pubBattery = node.newPublisher("diagnostics/battery", Battery._TYPE);
								break;
							case 1:
								if(pubMemory == null)
									pubMemory = node.newPublisher("diagnostics/memory", Memory._TYPE);
								break;
							case 2:
								if(pubButtons == null)
									pubButtons = node.newPublisher("buttons", Buttons._TYPE);
								break;
							}
							break;
						case TYPE_IMU:
							switch(i) {
							case 0:
								if(pubAccelerometer == null)
									pubAccelerometer = node.newPublisher("imu/accelerometer", Accelerometer._TYPE);
								break;
							case 1:
								if(pubGyroscope == null)
									pubGyroscope = node.newPublisher("imu/gyroscope", Gyroscope._TYPE);
								break;
							}
							break;
						case TYPE_IRLINK:
							if(subIRLinkCmd == null) {
								subIRLinkCmd = node.newSubscriber("irlink_cmd", IRLinkCmd._TYPE);
								subIRLinkCmd.addMessageListener(new MessageListener<IRLinkCmd>() {
									@Override
									public void onNewMessage(IRLinkCmd cmd) {
										int data[] = new int[]{};
										switch(cmd.getMode()) {
										case IRLinkCmd.COMBO_DIRECT:
											data = new int[]{IRLINK_COMBO_DIRECT, cmd.getChannel(), cmd.getOpA(), cmd.getOpB()};
											break;
										case IRLinkCmd.COMBO_PWM:
											data = new int[]{IRLINK_COMBO_PWM, cmd.getChannel(), cmd.getOpA(), cmd.getOpB()};
											break;
										case IRLinkCmd.EXTENDED:
											data = new int[]{IRLINK_EXTENDED, cmd.getChannel(), cmd.getOpA()};
											break;
										case IRLinkCmd.SINGLE_CST:
											data = new int[]{IRLINK_SINGLE_CST, cmd.getChannel(), cmd.getOpA(), cmd.getOpB()};
											break;
										case IRLinkCmd.SINGLE_PWM:
											data = new int[]{IRLINK_SINGLE_PWM, cmd.getChannel(), cmd.getOpA(), cmd.getOpB()};
											break;
										}
										try {
											if(data != null)
												proto.write(port, data);
										} catch (IOException e) {
											node.getLog().error("write: io error", e);
										}
									}
								});
							}
							break;
						case TYPE_LIGHT:
							switch(i) {
							case 0:
								if(pubLight == null)
									pubLight = node.newPublisher("light", Light._TYPE);
								break;
							}
							break;
						case TYPE_MOTOR:
							if(subMotorCmd == null) {
								subMotorCmd = node.newSubscriber("motor_cmd", MotorCmd._TYPE);
								subMotorCmd.addMessageListener(new MessageListener<MotorCmd>() {
									@Override
									public void onNewMessage(MotorCmd cmd) {
										try {
											final int p = cmd.getPosition(), v = cmd.getSpeed(), a = cmd.getAcceleration();
											switch(cmd.getMode()) {
											case MotorCmd.ROTATE_BY:
												if(v > 0 && a > 0)
													proto.motorRotateBy(port, p, v, a);
												else if(v > 0)
													proto.motorRotateBy(port, p, v);
												else
													proto.motorRotateBy(port, p);
												break;
											case MotorCmd.ROTATE_TO:
												if(v > 0 && a > 0)
													proto.motorRotateTo(port, p, v, a);
												else if(v > 0)
													proto.motorRotateTo(port, p, v);
												else
													proto.motorRotateTo(port, p);
												break;
											case MotorCmd.FLOAT:
												proto.motorFloat(port);
												break;
											case MotorCmd.STOP:
												proto.motorStop(port);
												break;
											case MotorCmd.FORWARD:
												proto.motorForward(port);
												break;
											case MotorCmd.BACKWAD:
												proto.motorBackward(port);
												break;
											}
										} catch(IOException e) {
											node.getLog().error("motor command: io error");
										}
									}
								});
							}
							if(subMotorSetStallTreshold == null) {
								subMotorSetStallTreshold = node.newSubscriber("motor_set_stall_treshold", MotorSetStallTreshold._TYPE);
								subMotorSetStallTreshold.addMessageListener(new MessageListener<MotorSetStallTreshold>() {
									@Override
									public void onNewMessage(MotorSetStallTreshold cmd) {
										try {
											proto.motorSetStallTreshold(port, cmd.getError(), cmd.getTime());
										} catch(IOException e) {
											node.getLog().error("motor command: io error");
										}
									}
								});
							}
							switch(i) {
							case 0:
								if(pubMotor == null)
									pubMotor = node.newPublisher("motor", Motor._TYPE);
								break;
							}
							break;
						case TYPE_MUXTOUCH:
							switch(i) {
							case 0:
								if(pubTouch == null)
									pubTouch = node.newPublisher("touch", Touch._TYPE);
								break;
							}
							break;
						case TYPE_SOUND:
							switch(i) {
							case 0:
								if(pubSound == null)
									pubSound = node.newPublisher("sound", Sound._TYPE);
								break;
							}
							break;
						case TYPE_TOUCH:
							switch(i) {
							case 0:
								if(pubTouch == null)
									pubTouch = node.newPublisher("touch", Touch._TYPE);
								break;
							}
							break;
						case TYPE_ULTRASONIC:
							switch(i) {
							case 0:
								if(pubUltrasonic == null)
									pubUltrasonic = node.newPublisher("ultrasonic", Ultrasonic._TYPE);
								break;
							}
							break;
						}
						int period = params.getInteger(pp);
						try {
							proto.startPolling(port, (byte)i, period);
						} catch (IOException e) {
							node.getLog().info("setup slot '" + portName + ":" + i + "' polling: io error", e);
						}
					}
				}
			}
		}
	}
	
	@Override
	public void onShutdown(final Node node) {	
	}

	@Override
	public void onShutdownComplete(final Node node) {
	}

	@Override
	public void onError(final Node node, final Throwable throwable) {
	}
}
