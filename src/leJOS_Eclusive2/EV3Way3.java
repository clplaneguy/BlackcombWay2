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
import lejos.robotics.Touch;
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

public class EV3Way3 {
	// static final double WHEEL_SIZE = 4.32;
	static final double WHEEL_SIZE = 5.6;
	static int runNum =1;																																															
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
		 while (true) 
	        {
	        //System.out.println("while (Button.ESCAPE.isUp())");
			// Gyro seems to reset from time to time, give it a kick
			// gyroPort.setMode(1);
			// waitForPress(startStop);
			// Button.waitForAnyPress();
			
			// Segoway segway = new Segoway(left, right, gyro, WHEEL_SIZE);
	        //System.out.println("run " + run);
	        //Thread segway = new Segoway2(run, left, right, gyro, WHEEL_SIZE);    // From krchilders
	        Segoway3 segway = new Segoway3(runNum, left, right, gyro, WHEEL_SIZE); 
			segway.setPriority(Thread.MAX_PRIORITY);                               // From krchilders
			segway.start();                                                        // From krchilders
			((Segoway3) segway).wheelDriver(0, 0);
			Sound.beep();
			int speed = 0;  
			int steer = 0;
			while(((Segoway3) segway).isRunning()) {
			// System.out.println("Inside while(Paginated_Executable.isRunning())");
				int cmd = ir.getRemoteCommand(0);
				switch (cmd) {
				case 1: // Top left: Turn left
					steer += 5;
					break;
				case 3: // Top right: Turn right
					steer -= 5;
					break;
				case 2: // Bottom left: Forward
					speed += 5;
					break;
				case 4: // Bottom right: Backward
					speed -= 5;
					break;
				case 9: // Top center: Stop
					speed = 0;
					steer = 0;
					break;
				}
				((Segoway3) segway).wheelDriver(speed + steer, speed - steer);
				if (Button.ESCAPE.isDown()) ((Segoway3) segway).halt();
				Delay.msDelay(100);
			}
			runNum++;
		}
	}

}