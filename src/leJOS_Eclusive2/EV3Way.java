package leJOS_Eclusive2;

//import java.io.DataInputStream;                                                                                                                                        
//import java.io.IOException;                                                                                                                                              

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
//import lejos.nxt.ADSensorPort;                                                                                                                                                                                                                                
//import lejos.nxt.Button;                                                                                                                                                                                                                           
//import lejos.nxt.EV3GyroSensor;                                                                                                                                                                                                      
//import lejos.nxt.EV3IRSensor;                                                                                                                                                                                                           
//import lejos.nxt.EV3TouchSensor;                                                                                                                                                                                                 
//import lejos.nxt.MotorPort;                                                                                                                                                                                                           
//import lejos.nxt.NXTMotor;                                                                                                                                                                                                          
//import lejos.nxt.SensorPort;                                                                                                                                                                                                  
//import lejos.nxt.Sound;                                                                                                                                                                                                         
//import lejos.nxt.UARTPort;                                                                                                                                                                                                      
//import lejos.nxt.addon.GyroSensor;                                                                                                                                                                                                       
import lejos.robotics.Touch;
//import lejos.nxt.comm.Bluetooth;                                                                                                                                                                                                          
//import lejos.nxt.comm.NXTConnection;                                                                                                                                                                                            
//import lejos.robotics.navigation.Segoway;                                                                                                                                                                                         
//import lejos.robotics.navigation.SegowayPilot;                                                                                                                                                                                         
//import lejos.util.Delay;                                                                                                                                                                                                              
import lejos.utility.Delay;

/**
 * <p>
 * This class demonstrates the movement capabilities of the SegowayPilot by
 * tracing three squares on the floor. It also implements a MoveListener to
 * demonstrate the move reporting abilities of all XxxPilot classes. As of
 * version 0.9 the SegowayPilot.rotate() method is very poor, but we will
 * improve this for version 1.0.
 * </p>
 * 
 * <p>
 * The Segoway and SegowayPilot classes require a HiTechnic gyro sensor. Mount
 * the sensor on the left side of the robot. See
 * <a href="http://laurensvalk.com/nxt-2_0-only/anyway">this model</a> for the
 * proper orientation.
 * </p>
 * 
 * @author BB 123456
 * 
 */
// public class SegowayEV3ir {}

public class EV3Way {
	// static final double WHEEL_SIZE = 4.32;
	static final double WHEEL_SIZE = 5.6;

	static void waitForPress(Touch t) {
		while (!t.isPressed())
			Delay.msDelay(100);
	}

	static void waitForRelease(Touch t) {
		while (t.isPressed())
			Delay.msDelay(100);
	}

	public static void main(String[] args) throws InterruptedException {
	System.out.println();
	System.out.println("6/28/2018");
	System.out.println("IIIIIIIIII\r\n" + "HHHHHHHHHH\r\n" + "AAAAATTTTT\r\n" + "ATATATATAT");
	System.out.println();
	System.out.printf("%20s\t\tBlackcombWay\n", "Project:");
	System.out.printf("%20s\tEV3Way_Paginated_Executable2\n", "Eclipse name:");

	System.out.println();
	// System.out.printf("\t\tStart\n");
	// long lastTimeStep = System.nanoTime();
	// System.out.println("System time is " + System.nanoTime());
	System.out.println();
	// System.out.println("6/27/2018");
	// System.out.println("EV3Way_Paginated_Executable.java main open");
	UnregulatedMotor left = new UnregulatedMotor(MotorPort.A);
	UnregulatedMotor right = new UnregulatedMotor(MotorPort.D);
	EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);
	Port port = LocalEV3.get().getPort("S1");
	EV3IRSensor ir = new EV3IRSensor(port);
	// while(Segoway.isRunning())
	while (Button.ESCAPE.isUp()) {
	System.out.println("while (Button.ESCAPE.isUp())");
	// Gyro seems to reset from time to time, give it a kick
	// gyroPort.setMode(1);
	// waitForPress(startStop);
	// Button.waitForAnyPress();
	// Segoway segway = new Segoway(left, right, gyro, WHEEL_SIZE);
	Thread segway = new Segoway(left, right, gyro, WHEEL_SIZE);// From krchilders
	segway.setPriority(Thread.MAX_PRIORITY); // From krchilders
	segway.setDaemon(true); // From krchilders
	segway.start(); // From krchilders
	segway.join(); // From krchilders

	((Segoway) segway).wheelDriver(0, 0);
	Sound.beep();
	int speed = 0;
	int steer = 0;
	while (Segoway.isRunning()) {
	// System.out.println("Inside while(Paginated_Executable.isRunning())");
	int cmd = ir.getRemoteCommand(0);
	switch (cmd) {
	case 1:
	steer += 5;
	break;
	case 3:
	steer -= 5;
	break;
	case 2:
	speed -= 5;
	break;
	case 4:
	speed += 5;
	break;
	case 9:
	speed = 0;
	steer = 0;
	break;
	}
	((Segoway) segway).wheelDriver(speed + steer, speed - steer);
	if (Button.ESCAPE.isDown())
	((Segoway) segway).halt();
	Delay.msDelay(100);
	}
	// waitForRelease(startStop);
	// Button.waitForAnyPress();
	}
	ir.close();
	}

}