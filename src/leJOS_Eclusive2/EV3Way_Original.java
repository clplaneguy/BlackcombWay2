package leJOS_Eclusive2;                                                                                                                             
                                                                                                                                              
import java.io.DataInputStream;                                                                                                       
import java.io.IOException;                                                                                                        
                                                                                                                                          
import lejos.hardware.Button;                                                                                                            
import lejos.hardware.Sound;                                                                                                               
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
 * <p>This class demonstrates the movement capabilities of the SegowayPilot by tracing three squares on the floor.                          
 * It also implements a MoveListener to demonstrate the move reporting abilities of all XxxPilot classes.                                   
 * As of version 0.9 the SegowayPilot.rotate() method is very poor, but we will improve this for version 1.0.</p>                             
 *                                                                                                                                         
 * <p>The Segoway and SegowayPilot classes require a HiTechnic gyro sensor. Mount the sensor on the left side of the                     
 * robot. See <a href="http://laurensvalk.com/nxt-2_0-only/anyway">this model</a> for the proper orientation.</p>                          
 *                                                                                                                                         
 * @author BB 1234556                                                                                                                              
 *                                                                                                                                      
 */                                                                                                                                        
//public class SegowayEV3ir {}                                                                                                            
                                                                                                                                           
public class EV3Way_Original {                                                                                                                       
    //static final double WHEEL_SIZE = 4.32;                                                                                               
    static final double WHEEL_SIZE = 5.6;                                                                                                  
                                                                                                                                             
    static void waitForPress(Touch t)                                                                                                       
    {                                                                                                                                         
        while(!t.isPressed())                                                                                                               
            Delay.msDelay(100);                                                                                                         
    }                                                                                                                                  
                                                                                                                                              
    static void waitForRelease(Touch t)                                                                                                  
    {                                                                                                                                   
        while(t.isPressed())                                                                                                          
            Delay.msDelay(100);                                                                                                         
    }                                                                                                                                     
                                                                                                                                         
    public static void main(String [] args) throws InterruptedException {                                                              
        //NXTMotor left = new NXTMotor(MotorPort.A);                                                                                     
        UnregulatedMotor left = new UnregulatedMotor(MotorPort.A);                                                                         
    	//NXTMotor right = new NXTMotor(MotorPort.D);                                                                                    
    	UnregulatedMotor right = new UnregulatedMotor(MotorPort.D);                                                                           
    	//UARTPort gyroPort = (UARTPort) SensorPort.S1;                                                                                  
    	//EV3GyroSensor gyro = new EV3GyroSensor(gyroPort);                                                                                 
        //EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);                                                                  
        EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);                                                                            
    	//EV3TouchSensor startStop = new EV3TouchSensor((ADSensorPort) SensorPort.S4);                                                     
                                                                                                                                           
        while (true)                                                                                                                     
        	{    ///////////////////////////////////////////////////////////////                                                                                                                             
            // Gyro seems to reset from time to time, give it a kick          //                                                                      
            //gyroPort.setMode(1);                                            //                                       
            //waitForPress(startStop);                                        //                                            
            Button.waitForAnyPress();                                         //                                           
            //Segoway segway = new Segoway(left, right, gyro, WHEEL_SIZE);                                                                                                                             
            SegowayEV3 segway = new SegowayEV3(left, right, gyro, WHEEL_SIZE);    //                                             
            segway.wheelDriver(0, 0);                                         //                                             
            Sound.beep();                                                     //                                          
            //EV3IRSensor ir = new EV3IRSensor((UARTPort) SensorPort.S2);     //                                             
            EV3IRSensor ir = new EV3IRSensor((Port) SensorPort.S2);           //                                                  
            int speed = 0;                                                    //                                             
            int steer = 0;                                                    //                                              
            while(SegowayEV3.isRunning())                                     //                                              
            	{    //////////////////////////////////////////////           //                                                                                                                 
                int cmd = ir.getRemoteCommand(0);                //           //                                                     
                switch(cmd)                                      //           //                                                     
                	{    ////////////////                        //           //                                                           
                	case 1:            //                        //           //                                                                
                		steer += 5;    //                        //           //                                                             
                		break;         //                        //           //                                                 
                	case 3:            //                        //           //                                                           
                		steer -= 5;    //                        //           //                                                                           
                		break;         //                        //           //                                                               
                	case 2:            //                        //           //                                                                         
                		speed -= 5;    //                        //           //                                                                          
                		break;         //                        //           //                                                                         
                	case 4:            //                        //           //                                                                        
                		speed += 5;    //                        //           //                                                                         
                		break;         //                        //           //                                                                       
                	case 9:            //                        //           //                                                                         
                		speed = 0;     //                        //           //                                                                          
                		steer = 0;     //                        //           //                                                                           
                		break;         //                        //           //                                                                          
                	}    ////////////////                        //           //                                                                         
                segway.wheelDriver(speed+steer, speed-steer);    //           //                                                                       
                if (Button.ESCAPE.isDown())                      //           //                                                                             
                    segway.halt();                               //           //                                                                          
                Delay.msDelay(100);                              //           //                                                                           
            	}    //////////////////////////////////////////////           //                                                                                                                        
            //waitForRelease(startStop);                                      //                                                                         
            Button.waitForAnyPress();                                         //                                                                                                                 
        	}    ///////////////////////////////////////////////////////////////                                                                                                                                   
    }                                                                                                                                
                                                                                                                                        
}                                                                        