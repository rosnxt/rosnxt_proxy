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

package com.github.rosnxt.passtrough;

import java.io.*;
import java.util.*;

import lejos.pc.comm.*;

public class Passtrough {
	static boolean run;

	public static void main(String args[]) throws IOException {
		NXTComm comm;

		if(args.length != 2) {
			System.out.println("usage: Passtrough <brickname> <bt|usb>");
			System.exit(1);
		}

		String brickName = args[0];
		String connectionMethod = args[1];

		try {
			if(connectionMethod.equals("bt")) {
				System.out.println("Creating Bluetooth comm...");
				comm = NXTCommFactory.createNXTComm(NXTCommFactory.BLUETOOTH);
			} else {
				System.out.println("Creating USB comm...");
				comm = NXTCommFactory.createNXTComm(NXTCommFactory.USB);
			}
		} catch (NXTCommException e) {
			if(connectionMethod.equals("bt")) {
				System.out.println("Cannot get Bluetooth comm: " + e);
			} else {
				System.out.println("Cannot get USB comm: " + e);
			}
			return;
		}
		
		System.out.println("Searching device with name " + brickName + "...");
		try {
			NXTInfo is[] = comm.search(brickName);
			if(is.length == 0) {
				System.out.println("No device found with name " + brickName + "!");
				return;
			} else if(!comm.open(is[0])) {
				System.out.println("Open device '" + brickName + "' failed");
				return;
			}
		} catch (NXTCommException e) {
			System.out.println("Cannot search for device: " + e);
			return;
		}
		
		System.out.println("Opened device " + brickName);
		run = true;
		
		final InputStream nxtIn = comm.getInputStream();
		final OutputStream nxtOut = comm.getOutputStream();
		final InputStream stdIn = System.in;
		final OutputStream stdOut = System.err;

		Thread in = new Thread() {
			public void run() {
				try {
					while(run) {
						int b = stdIn.read();
						if(b == -1) return;
						nxtOut.write(b);
						nxtOut.flush();
					}
				} catch(IOException e) {
					run = false;
				}
			}
		};
		Thread out = new Thread() {
			public void run() {
				try {
					while(run) {
						int b = nxtIn.read();
						if(b == -1) return;
						stdOut.write(b);
						stdOut.flush();
					}
				} catch(IOException e) {
					run = false;
				}
			}
		};
		/*
		while(true) {
			int inAvail = stdIn.available();
			if(inAvail > 0) {
				byte buf[] = new byte[inAvail];
				inAvail = stdIn.read(buf);
				nxtOut.write(buf, 0, inAvail);
				nxtOut.flush();
			}

			int outAvail = nxtIn.available();
			if(outAvail > 0) {
				byte buf[] = new byte[outAvail];
				outAvail = nxtIn.read(buf);
				stdOut.write(buf, 0, outAvail);
				stdOut.flush();
			}

			Thread.yield();
		}
		*/
		in.start();
		out.start();
		try {
			in.join();
			out.join();
		} catch(InterruptedException e) {}
	}
}
