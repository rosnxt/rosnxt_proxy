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

import static com.github.rosnxt.proxy.NXTProtocolConstants.*;

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
		
		node.getLog().info("Creating USB comm...");
		try {
			comm = NXTCommFactory.createNXTComm(NXTCommFactory.USB);
		} catch (NXTCommException e) {
			node.getLog().error("Cannot get USB comm", e);
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
		
		proto = new NXTProtocol(comm) {
			@Override
			protected void onAccelerometer(byte port, int x, int y, int z) {
				if(pubAccelerometer == null) return;
				Accelerometer m = pubAccelerometer.newMessage();
				m.setPort(port);
				m.setX(x);
				m.setY(y);
				m.setZ(z);
				pubAccelerometer.publish(m);
			}
			
			@Override
			protected void onBatteryLevel(byte port, int levelMilliVolt) {
				if(pubBattery == null) return;
				Battery m = pubBattery.newMessage();
				m.setPort(port);
				m.setLevelMilliVolts(levelMilliVolt);
				pubBattery.publish(m);
			}
			
			@Override
			protected void onButtons(byte port, boolean enter, boolean escape, boolean left, boolean right) {
				if(pubButtons == null) return;
				Buttons m = pubButtons.newMessage();
				m.setPort(port);
				m.setEnter(enter ? (byte)1 : 0);
				m.setEscape(escape ? (byte)1 : 0);
				m.setLeft(left ? (byte)1 : 0);
				m.setRight(right ? (byte)1 : 0);
				pubButtons.publish(m);
			}
			
			@Override
			protected void onColor(byte port, int colorID) {
				if(pubColor == null) return;
				Color m = pubColor.newMessage();
				m.setPort(port);
				m.setColorId((byte)colorID);
				pubColor.publish(m);
			}
			
			@Override
			protected void onColor(byte port, int r, int g, int b) {
				if(pubColorRGB == null) return;
				ColorRGB m = pubColorRGB.newMessage();
				m.setPort(port);
				m.setR((byte)r);
				m.setG((byte)g);
				m.setB((byte)b);
				pubColorRGB.publish(m);
			}
			
			@Override
			protected void onColorLightValue(byte port, int value) {
				if(pubLight == null) return;
				Light m = pubLight.newMessage();
				m.setPort(port);
				m.setLightValue(value);
				pubLight.publish(m);
			}
			
			@Override
			protected void onCompass(byte port, float degree) {
				if(pubCompass == null) return;
				Compass m = pubCompass.newMessage();
				m.setPort(port);
				m.setDegrees(degree);
				pubCompass.publish(m);
			}
			
			@Override
			protected void onFreeMemory(byte port, int mem) {
				if(pubMemory == null) return;
				Memory m = pubMemory.newMessage();
				m.setPort(port);
				m.setFreeMemory(mem);
				pubMemory.publish(m);
			}
			
			@Override
			protected void onGyroscope(byte port, float x, float y, float z) {
				if(pubGyroscope == null) return;
				Gyroscope m = pubGyroscope.newMessage();
				m.setPort(port);
				m.setX(x);
				m.setY(y);
				m.setZ(z);
				pubGyroscope.publish(m);
			}
			
			@Override
			protected void onLightValue(byte port, int value) {
				if(pubLight == null) return;
				Light m = pubLight.newMessage();
				m.setPort(port);
				m.setLightValue(value);
				pubLight.publish(m);
			}
			
			@Override
			protected void onMotor(byte port, boolean isMoving, boolean isStalled, int tachoCount, int speed, int rotationSpeed, int limitAngle) {
				if(pubMotor == null) return;
				Motor m = pubMotor.newMessage();
				m.setPort(port);
				m.setMoving(isMoving ? (byte)1 : 0);
				m.setStalled(isStalled ? (byte)1 : 0);
				m.setTacho(tachoCount);
				m.setSpeed(speed);
				m.setRotationSpeed(rotationSpeed);
				m.setLimitAngle(limitAngle);
				pubMotor.publish(m);
			}
			
			@Override
			protected void onMuxTouch(byte port, boolean b1, boolean b2, boolean b3, boolean b4) {
				if(pubTouch == null) return;
				Touch m = pubTouch.newMessage();
				m.setPort(port);
				m.setButton((byte)0);
				m.setPressed(b1 ? (byte)1 : 0);
				pubTouch.publish(m);
				m = pubTouch.newMessage();
				m.setPort(port);
				m.setButton((byte)1);
				m.setPressed(b2 ? (byte)1 : 0);
				pubTouch.publish(m);
				m = pubTouch.newMessage();
				m.setPort(port);
				m.setButton((byte)2);
				m.setPressed(b3 ? (byte)1 : 0);
				pubTouch.publish(m);
				m = pubTouch.newMessage();
				m.setPort(port);
				m.setButton((byte)3);
				m.setPressed(b4 ? (byte)1 : 0);
				pubTouch.publish(m);
			}
			
			@Override
			protected void onSound(byte port, int value) {
				if(pubSound == null) return;
				Sound m = pubSound.newMessage();
				m.setPort(port);
				m.setLevel(value);
				pubSound.publish(m);
			}
			
			@Override
			protected void onTouch(byte port, boolean b) {
				if(pubTouch == null) return;
				Touch m = pubTouch.newMessage();
				m.setPort(port);
				m.setButton((byte)0);
				m.setPressed(b ? (byte)1 : 0);
				pubTouch.publish(m);
			}
			
			@Override
			protected void onUltrasonic(byte port, int range) {
				if(pubUltrasonic == null) return;
				Ultrasonic m = pubUltrasonic.newMessage();
				m.setPort(port);
				m.setDistance(range);
				pubUltrasonic.publish(m);
			}
		};
		
		setupPorts(node, params);
	}
	
	protected void setupPorts(final ConnectedNode node, ParameterTree params) {
		for(String portName : NXTProtocolString.portNames()) {
			String paramName = "~port/" + portName + "/type";
			if(params.has(paramName)) {
				final String t = params.getString(paramName);
				final byte port = NXTProtocolString.portFromString(portName);
				final byte type = NXTProtocolString.typeFromString(t);
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
