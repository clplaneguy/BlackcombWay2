package leJOS_Eclusive2;                                    
                                                            
//1234567891234567892234567893                                                                                    
//  **************************                                                                              
//      **********************                                                                         
//          ******************      Spaces                                                             
//              **************                                                            
//                  **********                                                         
//                      ******                                                     
//                          **                                                        
//                              *                                                        
//1234567891234567892234567893                                                                                 
//	**************************                                                                                                                                                                                                                                       
//		**********************                                                         
//			******************      Tab = 4 spaces                                      
//				**************                                                      
//					**********                                                                 
//						******                                                                
//							**                                                                      
//								*                                                                           
//1234567891234567892234567893                                                                                              
//	**********************                                                                           
//		**************         Tab = 8 spaces                                                    
//			******                                                         
//				*                                                         
                                                                               
import java.io.BufferedWriter;                                                    
import java.io.File;                                                                
import java.io.FileWriter;                                                      
import java.io.IOException;                                                          
                                                                                              
import lejos.hardware.Button;                                                                
import lejos.hardware.Sound;                                                                          
import lejos.hardware.lcd.LCD;                                                                    
import lejos.hardware.motor.UnregulatedMotor;                                                      
import lejos.hardware.sensor.EV3GyroSensor;                                                           
import lejos.robotics.EncoderMotor;                                                                
import lejos.robotics.SampleProvider;                                                                
import lejos.utility.Delay;                                                                         
                                                                                                       
/* <p>                                                                                                                                                                                                                        
 * <li>This class balances a two-wheeled Segway-like robot. It works with almost any construction                                                                                                                           
 * (short or tall) such as the                                                                                 
 * <li><a> href="http://www.laurensvalk.com/nxt-2_0-only/anyway">Anyway</a>                                      
 * <li>or the                                                                                                        
 * <li><a> href="http://www.hitechnic.com/blog/gyro-sensor/htway/"HTWay</a></p>                                       
 *                                                                                                                              
 * <p>Wheel diameter is the most important construction variable, which is specified in the constructor.</p>                                                                                                                                   
 *                                                                                                                                                                                                           
 * <p>To start the robot balancing:                                                                                                                                                                                         
 * <li>1. Run the program. You will be prompted to lay it down.                                                                                                                                                                                                                  
 * <li>2. Lay it down (orientation doesn't matter). When it detects it is not moving it will automatically calibrate the gyro sensor.                                                                                                                      
 * <li>3. When the beeping begins, stand it up so it is vertically balanced.                                                                                                                                                        
 * <li>4. When the beeping stops, let go and it will begin balancing on its own.</p>                                                                                                                                 
 *                                                                                                                                                                                                                               
 * <p>Alternately you can lean the robot against a wall and run the program. After the gyro                                                                                                                                                        
 * calibration, the robot backs up against the wall until it falls forward. When it detects the                                                                                                                                          
 * forward fall, it start the balance loop.</p>                                                                                                                                                                                
 *                                                                                                                                                                                                               
 * <p>NOTE: In order to make the robot move and navigate, use the SegowayPilot class.</p>                                                                                                                                               
 *                                                                                                                                                                                                               
 * <p><i>This code is based on the HTWay by HiTechnic and the programs by BB and gloomyandy.</i></p>                                                                                                               
 *                                                                                                                                                                                                                  
 * @author clplaneguy   123456                                                                                                                                                                                                                                                               
 *                                                                                                                                                                                                                                                             
 */                                                                                                              
public class Segoway2 extends Thread {                                                                               
	private Thread t;                                                                                        
	private String threadName;                                                                            
	                                                                                                                    
	// Motors and gyro:                                                                                            
	private EV3GyroSensor ev3Gyro;                                                                            
	private SampleProvider spGyro;                                                                                
	private float[] gyroSample;                                                                                       
	private EncoderMotor left_motor;                                                                                 
	private EncoderMotor right_motor;                                                                            
                                                                                                                     
	// =====================================================================                                              
	// Balancing constants                                                                                  
	//                                                                                                                       
	// These are the constants used to maintain balance.                                                                                            
	// =====================================================================                                                        
	/**                                                                                                           
	 * Basic wait time used to set variables values. Used to calculate new values                                     
	 * that are time dependent.                                                                     
	 */                                                                                              
	private static final int BASE_WAIT_TIME = 9;                                                
	                                                                                       
	/**                                                                                                                   
	 * Minimum loop time(ms) passed to the Wait command. NOTE: Balance control loop                                    
	 * only takes 0.63ms in EV3 leJOS EVJ 0.9.1-beta after 10,000,000 iterations                                      
	 */                                                                                                                                                                                                                                                                                                                                                                         
	private static final int WAIT_TIME = 7;                                                                                   
                                                                                                                    
	// These are the main four balance constants, only the gyro                                                
	// constants are relative to the wheel size. KPOS and KSPEED                                                    
	// are self-relative to the wheel size.                                                                                                    
	//private static final double KGYROANGLE = 15;                                                                 
	private static final double KGYROANGLE = 16;                                                                 
	private static final double KGYROSPEED = 1.3;                                                                
    //private static final double KPOS       = 0.1;                                                                 
	private static final double KPOS       = 0.2;                                                                 
	//private static final double KSPEED     = 0.11;                                                          
	//private static final double KSPEED     = 0.2;                                                          
	//private static final double KSPEED     = 0.15;                                                          
	private static final double KSPEED     = 0.12;                                                          
    //KGYROANGLE = 16;   KGYROSPEED = 1.3;     KPOS       = 0.2;  KSPEED     = 0.12;                                                                                              
                  	                                                               
  	/**                                                                                                         
	 * This constant aids in drive control. When the robot starts moving because of                                     
	 * user control, this constant helps get the robot leaning in the right                                       
	 * direction. Similarly, it helps bring robot to a stop when stopping.                                          
	 */                                                                                                                
	private static final double KDRIVE = -0.02;                                                                      
                                                                                                                      
	                                                                                                        
	/**                                                                                                                                                                                  
	 * Power differential used for steering based on difference of target steering                               
	 * and actual motor difference.                                                                                     
	 */                                                                                                                 
	private static final double KSTEER = 0.25;                                                                           
                                                                                                                        
                  	                                                                                                       
 	/**                                                                                                                  
	 * Gyro offset control The gyro sensor will drift with time. This constant is                                     
	 * used in a simple long term averaging to adjust for this drift. Every time                                        
	 * through the loop, the current gyro sensor value is averaged into the gyro                                         
	 * offset weighted according to this constant. The value is set to reset the                                          
	 * offset over a 5 second period.                                                                                      
	 */                                                                                                                  
	//private static final double EMAOFFSET = WAIT_TIME / (1000.0 * 5);                                                  
	private static final double EMAOFFSET = WAIT_TIME / 5000.0;                                                              
                                                
	                                   
	/**                                                                                                            
	 * If robot power is saturated (over 100%) for over this time limit(ms) then                      
	 * robot has fallen.                  
	 */                                                                                                                               
	//private static final double TIME_FALL_LIMIT  = 700; // originally 1000
    // private static final double TIME_FALL_LIMIT = 2000;      
	// private static final double TIME_FALL_LIMIT = 200000; 
	// private static final double TIME_FALL_LIMIT = 1E3;   
	//private static final double TIME_FALL_LIMIT    = 2E3;       
	private static final double TIME_FALL_LIMIT    = 1E3;       
                                                                                      
	// ---------------------------------------------------------------------             
                                        
	                                        
	/**                                                                                                                                  
	 * Maximum speed(degrees/second) Note that position and speed are measured as  
	 * the sum of the two motors, in other words, 600 would actually be 300               
	 * degrees/second for each motor.                               
	 */                                                                                          
	private static final double CONTROL_SPEED  = 600.0;                                
	private static final int    OFFSET_SAMPLES = 100;                                 
	private static final int    MAX_VALUE      = 1000;                                  
	                                                                              
	                                                                                     
	// =====================================================================                
	// Global variables                                                                          
	// =====================================================================                      
                                                                                                       
	// These two xxControlDrive variables are used to control the movement of the                                        
	// robot. Both                                                                       
	// are in degrees/second:                                                             
	/**                                                                                       
	 * Target speed(degrees per second) for the sum of the two motors.                       
	 */                                                                                    
	private double motorControlDrive = 0.0;                                                
                                                                                            
                  	                                                                       
	/**                                                                                            
	 * Target change in difference(degrees per second) for two motors.                           
	 */                                                                                      
	private double motorControlSteer = 0.0;                                                       
                                                                                                    
                   	                                                                               
	/**                                                                                                 
	 * This global contains the target motor differential, essentially, which way                      
	 * the robot should be pointing. This value is updated every time through the                      
	 * balance loop based on motorControlSteer.                                                      
	 */                                                                                              
	private double motorDiffTarget = 0.0;                                                             
                                                                                                   
                  	                                                                                  
	/**                                                                                               
	 * Time robot first starts to balance. Used to calculate tInterval.                              
	 */                                                                                         
	private long tCalcStart;                                                                    
                                                                                                
              	                                                                                  
	/**                                                                                       
	 * Time(seconds) for each iteration of the balance loop.                                 
	 */                                                                                           
	private double tInterval;                                                                    
                                                                                                   
                	                                                                                 
	/**                                                                                                
	 * Smoothed version of tInterval                                                              
	 */                                                                                               
	double tSmoothedInterval;                                                                         
                                                                                                     
                   	                                                                                  
	/**                                                                                                      
	 * Relative wheel size compared to a standard NXT 1.0 wheel.                                      
	 */                                                                                                             
	private double ratioWheel;                                                  
                                                                                      
                             	                                                          
	// Gyro globals                                                                                   
	//private double gOffset;                                                                 
	private double gyroSpeed, gyroAngle;                                                       
	private double gAngleGlobal;                   
	private boolean invertGyro = true; 
 
                            	
	// Motor globals               
	private double motorPos;              
	private long mrcSum, mrcSumPrev; 
	private long motorDiff;          
	private long mrcDeltaP3;    
	private long mrcDeltaP2;   
	private long mrcDeltaP1; 
 
            	
	/**                                                                                    
	 * Global variables used to control the amount of power to apply to each wheel. 
	 * Updated by the steerControl() method. 
	 */                                       
	private int powerLeft, powerRight; 
	private double motorSpeed; 
                                                   
	private static boolean running = true; 
                                
	long fatigue = 0;                           
	double[] Duration = new double[10]; 
	double testv[] = new double[10]; 
	int TempIndex; 
	double Time;     
	long MyTime; 
	long mytime;   
	long now1;    
	long now2;                                                
	long mrcLeft, mrcRight, mrcDelta; // Segoway original 
 
                                      
	BufferedWriter bw = null;  
	// boolean Debug = false; 
	boolean Debug = true; 
	double  RunTimeMS; 
	double  RunTimeS;                     
	double  AverageMS; // 1E6 ms 
	double  AverageS;  // 1E6 ms 
	//double  KGYROANGLE2 = 9; 
	//double  KGYROSPEED2 = 3; 
	//double  KPOS2 = 0.08; 
	//double  KSPEED2 = 0.2;          
	//double  KDRIVE2 = 0.2;                                     
	long    freeMemory = Runtime.getRuntime().freeMemory();    
	float   gyroRaw;                                      
	int     run;          
	double offset = 0.0f;                    
	double smoothmtrSpeed = 0;
	double SmoothDelta = 0;
	                                                                      
	 int TestInterval = (int) 1E1;  //       10; Did not work                                                        
	// int TestInterval = (int) 1E2;  //      100; Stoped at 52               Added close Buffered Writer         
	// int TestInterval = (int) 1E3;  //     1000; Stoped at 972              Added close Buffered Writer 
	// int TestInterval = (int) 1E4;  //    10000; Stoped at 9966             Added close Buffered Writer           
	//int TestInterval = (int) 1E5;    //   100000; Stoped at 15993 of 16043   Added close Buffered Writer 
	// int TestInterval = (int) 1E6;  //  1000000; Program ended   
                                                                              
	int    [] loopCountA         = new    int [TestInterval]  ;      
	long   [] freeMemoryA        = new   long [TestInterval]  ; 
	long   [] mrcDeltaA          = new   long [TestInterval]  ; 
	long   [] mrcDeltaP1A        = new   long [TestInterval]  ;  
	long   [] mrcDeltaP2A        = new   long [TestInterval]  ; 
	long   [] mrcDeltaP3A        = new   long [TestInterval]  ; 
	float  [] gyroRawA           = new  float [TestInterval]  ;                           
	double [] TestIntervalA      = new double [TestInterval]  ; 
	double [] gyroAngleA         = new double [TestInterval]  ; 
	double [] gyroSpeedA         = new double [TestInterval]  ; 
	double [] motorPosA          = new double [TestInterval]  ; 
	double [] motorSpeedA        = new double [TestInterval]  ; 
	double [] gAngleGlobalA      = new double [TestInterval]  ; 
	double [] powerA             = new double [TestInterval]  ; 
	double [] powerLeftA         = new double [TestInterval]  ; 
	double [] powerRightA        = new double [TestInterval]  ; 
	double [] mrcLeftA           = new double [TestInterval]  ; 
	double [] mrcRightA          = new double [TestInterval]  ; 
	double [] tCalcStartA        = new double [TestInterval]  ; 
	double [] tIntervalA         = new double [TestInterval]  ; 
	double [] tMotorPosOKA       = new double [TestInterval]  ; 
	double [] RunTimeMSA         = new double [TestInterval]  ; 
	double [] RunTimeSA          = new double [TestInterval]  ;                                              
	double [] motorControlDriveA = new double [TestInterval]  ; // target speed in degrees per second 
	double [] gOffsetA           = new double [TestInterval]  ; 
 	  
	          
	/**                                                                                   
	 * Creates an instance of the Segoway, prompts the user to steady Segoway for 
	 * gyro calibration, then begins self-balancing thread. Wheel diameter is used          
	 * in balancing equations. 
	 *                                        
	 * <li>NXT 1.0 wheels = 5.6 cm    
	 * <li>NXT 2.0 wheels = 4.32 cm              
	 * <li>RCX "motorcycle" wheels = 8.16 cm 
	 * <li>EV3 wheels = 5.6 cm 
	 *                    
	 * @param left                                            
	 *            The left motor. An unregulated motor. 
	 * @param right                                               
	 *            The right motor. An unregulated motor. 
	 * @param ev3Gyro                      
	 *            A EV3 gyro sensor 
	 * @param invertGyro                                                                         
	 *            Whether or not gyro readings should be inverted, depends on sensor 
	 *            orientation. 
	 * @param wheelDiameter                                                               
	 *            diameter of wheel, preferably use cm (printed on side of LEGO 
	 *            tires in mm) 
	 */                                                                                                                                 
	public Segoway2(int run, UnregulatedMotor left, UnregulatedMotor right, EV3GyroSensor ev3Gyro, double wheelDiameter) { 
		this.left_motor = left;            
		this.right_motor = right; 
                                         
		this.ev3Gyro = ev3Gyro;                        
		this.spGyro = ev3Gyro.getRateMode();                 
		gyroSample = new float[spGyro.sampleSize()]; 
                                                                             
		// Original algorithm was tuned for 5.6 cm NXT 1.0 wheels. 
		this.ratioWheel = wheelDiameter / 5.6; 
                                                                          
		// Get the initial gyro offset and calibrate sensor. 
		calibrateGyro(); 
                                                                          
		// Play warning beep sequence before balance starts 
		startBeeps();                       
	}                                                                                                                
            
	                                                                                                                  
	/**                                                                                 //  From krchilders                                        
	 * This function returns a suitable initial gyro offset. It takes 200 gyro          //  From krchilders                                 
	 * samples over a time of 1 second and averages them to get the offset. It also     //  From krchilders                            
	 * checks the maximum and minimum during that time and if the difference is         //  From krchilders                            
	 * larger than one (1), it rejects the data and gets another set of samples.        //  From krchilders                              
	 */                                                                                 //  From krchilders                           
	private void calibrateGyro() {                                                      //  From krchilders                             
		LCD.clear();                                                                    //  From krchilders                          
		LCD.drawString("EV3 Segoway" , 0, 0);                                                              
		LCD.drawString("Steady robot", 0, 2);                                                    
		LCD.drawString("to calibrate", 0, 3);                                                          
		LCD.drawString("the gyro"    , 0, 4);                                            
	                                                                                    //  From krchilders                  
		double gSum;                                                               //  From Andy                       
	    float gMin;                                                                //  From Andy                    
	    float gMax;                                                                //  From Andy              
	    int g;
	        do {                                                                         //  From Andy                    
	            gSum = 0.0;                                                              //  From Andy                     
	            gMin =  Float.MAX_VALUE;                                                 //  From Andy                     
	            gMax = -Float.MAX_VALUE;                                                 //  From Andy                     
	            for (int i1=0; i1<OFFSET_SAMPLES; i1++)                                  //  From Andy                       
	            {                                                                        //  From Andy                              
	                //float g = getAngularVelocity();                                    //  From Andy                                
	                spGyro.fetchSample(gyroSample, 0);                                   //  From krchilders                                                 
					g = (int) gyroSample[0];                                             //  From krchilders                                           
				    if (g > gMax) gMax = g;                
	                if (g < gMin) gMin = g;                 
                    gSum += g;                                                           //  From Andy                           
	                Delay.msDelay(5);                                                    //  From Andy                            
	            }                                                                        //  From Andy                               
	        //} while ((gMax - gMin) > 5);   // Reject and sample again if range too large //  From Andy                                   
	        //} while ((gMax - gMin) > 4);   // Reject and sample again if range too large                                   
	        } while ((gMax - gMin) > 3);   // Reject and sample again if range too large                                   
                                                                                         //  From Andy                              
	        //Average the sum of the samples.                                            //  From Andy                                          
	        offset = (float)(gSum / OFFSET_SAMPLES) + 1; // TODO: Used to have +1, which was mainly for stopping Segway wandering.                                                                                                               
		if (invertGyro) offset = -offset;                          
	  }                                                                                                     
          
	           
	/**                                                                                                                                                                      
	 * Warn user the Segoway is about to start balancing.                                                                      
	 */                                                                                                                        
	private void startBeeps() {                                                                                                                                             
                                                                                                              
		//System.out.println("Balance in");                                                                              
                                                                                                                     
		// Play warning beep sequence to indicate balance about to start                                                     
		for (int c = 5; c > 0; c--) {                                                                                                 
			//System.out.print(c + " ");                                                                                 
			Sound.playTone(440, 100);                                                                                    
			try {                                                                                                     
				Thread.sleep(1000);                                                                                      
			} catch (InterruptedException e) {                                                                            
			}                                                                                                                 
		}                                                                                                             
		//System.out.println("GO");                                                                                         
		//System.out.println();                                                                                          
	}                                                                                                                    
         
                                                                                                                                                                                                                                          
	/**                                                                                      //  From krchilders                            
	 * Get the data from the gyro. Fills the pass by reference gyroSpeed and                 //  From krchilders                                       
	 * gyroAngle based on updated information from the Gyro Sensor. Maintains an             //  From krchilders                               
	 * automatically adjusted gyro offset as well as the integrated gyro angle.              //  From krchilders                              
	 *                                                                                       //  From krchilders                             
	 */                                                                                      //  From krchilders                                      
	private void updateGyroData() {                                                          //  From krchilders                                                                       
		// NOTE: The GyroSensor class actually rebaselines for drift ever 5 seconds.         //  From krchilders                    
		// This not needed? Or is this method better? Some of this fine tuning may           //  From krchilders                                              
		// actually interfere with fine-tuning happening in the hardcoded dIMU and           //  From krchilders                               
		// GyroScope code. As of EV3 0.8.1-beta drift accounting is still needed.            //  From krchilders                          
		//float gyroRaw;                                                                     //  From krchilders                             
		spGyro.fetchSample(gyroSample, 0);                                                   //  From krchilders                                 
                                                                                             //  From krchilders                           
		gyroRaw = gyroSample[0]; // deg/s                                                    //  From krchilders                              
	                                                                                         //  From krchilders                          
		if (invertGyro) gyroRaw = -gyroRaw;                           
	                                                                                         //  From krchilders                                  
		//gOffset = EMAOFFSET * gyroRaw        +   (1 - EMAOFFSET) * gOffset;                                             
		 offset = EMAOFFSET * gyroRaw        +   (1 - EMAOFFSET) * offset;                          
		// offset = (1 - EMAOFFSET) * offset   +   EMAOFFSET * gyroRaw;                          
		//EMAOFFSET = 0.0014;                                                                             
		// offset = 0.0014 * gyroRaw           +   (1 - 0.0014) * offset;                          
		// offset = (1 - 0.0014) * offset      +   0.0014 * gyroRaw;                          
		// offset = 0.9986 * offset            +   0.0014 * gyroRaw;                          
		// offset = 0.99999*offset             +   0.00001*gyroRaw;                          
		 
		gyroSpeed = gyroRaw - offset; // Angular velocity (degrees/sec)                                                
	                                                                                         //  From krchilders                                
		gAngleGlobal +=  gyroSpeed * tInterval;                                                                               
		gyroAngle     =  gAngleGlobal;            // Absolute angle (degrees)                                                                                
	}                                                                                        //  From krchilders                            
               
	             
	/**                                                                                                                                   
	 * Keeps track of wheel position with both motors.  
	 */                                          
	private void updateMotorData() {            
		long mrcLeft, mrcRight, mrcDelta; 
                                                            
		// Keep track of motor position and speed 
		mrcLeft = left_motor.getTachoCount();        
		mrcRight = right_motor.getTachoCount(); 
                                                                                      
		// Maintain previous mrcSum so that delta can be calculated and get 
		// new mrcSum and Diff values      
		mrcSumPrev = mrcSum;                
		mrcSum = mrcLeft + mrcRight;            
		motorDiff = mrcLeft - mrcRight; 
                                                                              
		// mrcDetla is the change int sum of the motor encoders, update 
		// motorPos based on this detla 
		mrcDelta = mrcSum - mrcSumPrev; 
		//SmoothDelta = 0.75 * SmoothDelta  +  0.25 * mrcDelta;
		//SmoothDelta = 0.8 * SmoothDelta  +  0.2 * mrcDelta;
		SmoothDelta = 0.75 * SmoothDelta  +  0.25 * mrcDelta;
		//motorPos += mrcDelta; 
		motorPos += SmoothDelta; 
                                                                                    
		// motorSpeed is based on the average of the last four delta's.                                      
		//motorSpeed = (mrcDelta + mrcDeltaP1 + mrcDeltaP2 + mrcDeltaP3) / (4 * tSmoothedInterval); 
		motorSpeed = SmoothDelta / tSmoothedInterval; 
        //smoothmtrSpeed = 0.75 * smoothmtrSpeed  +  0.25 * motorSpeed;
		
		// Shift the latest mrcDelta into the previous three saved delta values 
		mrcDeltaP3 = mrcDeltaP2; 
		mrcDeltaP2 = mrcDeltaP1; 
		mrcDeltaP1 = mrcDelta; 
	} 
  
	                 
	/**                                                                                      
	 * This function determines the left and right motor power that should be used 
	 * based on the balance power and the steering control. 
	 */                                             
	private void steerControl(int power) { 
		int powerSteer; 
                                                                                
		// Update the target motor difference based on the user steering 
		// control value.                                                     
		motorDiffTarget += motorControlSteer * tSmoothedInterval; 
                                                                                   
		// Determine the proportionate power differential to be used based 
		// on the difference between the target motor difference and the 
		// actual motor difference.                                         
		powerSteer = (int) (KSTEER * (motorDiffTarget - motorDiff)); 
                                                                           
		// Apply the power steering value with the main power value to 
		// get the left and right power values. 
		powerLeft  = power + powerSteer;          
		powerRight = power - powerSteer;                       
                                                                   
		// Limit the power to motor power range -100 to 100 
		if (powerLeft >  100) powerLeft =  100; 
		if (powerLeft < -100) powerLeft = -100; 
                                                                      
		// Limit the power to motor power range -100 to 100 
		if (powerRight >  100) powerRight =  100; 
		if (powerRight < -100) powerRight = -100; 
	} 
 
                  	
	/**                                                                                     
	 * Calculate the interval time from one iteration of the loop to the next. Note     
	 * that first time through, cLoop is 0, and has not gone through the body of the 
	 * loop yet. Use it to save the start time. After the first iteration, take the 
	 * average time and convert it to seconds for use as interval time. 
	 */                                         
	private void calcInterval(long cLoop) { 
		if (cLoop == 0) {                                                 
			// First time through, set an initial tInterval time and 
			// record start time                
			tInterval = WAIT_TIME / 1000.0; 
			tSmoothedInterval = tInterval; 
			;                                             
			tCalcStart = System.currentTimeMillis(); 
		} else {                                                         
			// Take average of number of times through the loop and 
			// use for interval time.                      
			long tNew = System.currentTimeMillis();      
			tInterval = (tNew - tCalcStart) / 1000.0;                                
			//tSmoothedInterval = 0.75 * tSmoothedInterval + 0.25 * tInterval; 
			tSmoothedInterval = 0.7 * tSmoothedInterval + 0.3 * tInterval; 
			tCalcStart = tNew; 
		} 
             
	} 
 
                	
	/**                                               
	 * Is the control loop still running 
	 *                                                          
	 * @return true if the loop is running. 
	 */                                      
	public boolean isRunning() { 
		return running;  
	} 
 
                 	
	/**                               
	 * Stop the control loop 
	 */                          
	public void halt() {  
		running = false;  
	}      
                                                                                       
	// --------------------------------------------------------------------- 
	//                                                         
	// This is the main balance thread for the robot. 
	//                                                                          
	// Robot is assumed to start leaning on a wall. The first thing it         
	// does is take multiple samples of the gyro sensor to establish and 
	// initial gyro offset. 
	//                                                                             
	// After an initial gyro offset is established, the robot backs up       
	// against the wall until it falls forward, when it detects the 
	// forward fall, it start the balance loop. 
	//                                      
	// The main state variables are:                                          
	// gyroAngle This is the angle of the robot, it is the results of 
	// integrating on the gyro value. 
	// Units: degrees                                                              
	// gyroSpeed The value from the Gyro Sensor after offset subtracted 
	// Units: degrees/second                                          
	// motorPos This is the motor position used for balancing. 
	// Note that this variable has two sources of input: 
	// Change in motor position based on the sum of 
	// MotorRotationCount of the two motors, 
	// and,                                                      
	// forced movement based on user driving the robot. 
	// Units: degrees (sum of the two motors)                                     
	// motorSpeed This is the speed of the wheels of the robot based on the 
	// motor encoders.                                    
	// Units: degrees/second (sum of the two motors) 
	//                                                                               
	// From these state variables, the power to the motors is determined 
	// by this linear equation:         
	// power = KGYROSPEED * gyro + 
	// KGYROANGLE * gyroAngle + 
	// KPOS * motorPos +       
	// KSPEED * motorSpeed;        
	//                          
	public void run() { 
		mrcSum      = 0; 
		motorDiff   = 0; 
		mrcDeltaP3  = 0; 
		mrcDeltaP2  = 0; 
		mrcDeltaP1  = 0; 
		running = true;                      
		//double gAngleGlobal = 0; 
		gAngleGlobal = 0; 
		motorPos = 0;                                                                                                
		int     loopCount    =  1; // postpone activation of the motors until dt in the loop is stable // From 
		double  power        =  0;          
		double  smoothPower  =  0.0; 
		long    cLoop        =  0;                                       
		long    StartTime    =  System.currentTimeMillis(); // 1E9 
		long    tMotorPosOK  =  System.currentTimeMillis(); 
		double KGYROANGLE2;            
		double KGYROSPEED2;
		double KPOS2;
		double KSPEED2;
		
		//KGYROANGLE = 16;   KGYROSPEED = 1.3;     KPOS       = 0.2;  KSPEED     = 0.12;                                                                                              
		//KGYROANGLE2 = KGYROANGLE;   KGYROSPEED2 = KGYROSPEED;     KPOS2 = KPOS;  KSPEED2 = KSPEED;                                                                                              
		//KGYROANGLE2 = 16;   KGYROSPEED2 = 1.3;     KPOS2 = 0.2;  KSPEED2 = 0.12;                                                                                              
		KGYROANGLE2 = 17;   KGYROSPEED2 = 1.3;     KPOS2 = 0.2;  KSPEED2 = 0.12;                                                                                              
	    
		try {                                                  
			// Specify the file name and path here 
			File file = new File("myfile.txt"); 
			/*                                                                                     
			 * This logic will make sure that the file gets created if it is not present at 
			 * the specified location 
			 */                             
			if (!file.exists()) {            
				file.createNewFile(); 
			}                                          
			FileWriter fw = new FileWriter(file); 
			bw = new BufferedWriter(fw);                            
			//System.out.println("File written Successfully"); 
		} catch (IOException ioe) {     
			ioe.printStackTrace(); 
		} finally { 
		} 
                                                   
		//System.out.println("Balancing"); 
		//System.out.println(); 
 
                                                                                    		
		// Reset the motors to make sure we start at a zero position   
		left_motor.resetTachoCount();   
		right_motor.resetTachoCount(); 
		right_motor.forward(); 
		left_motor.forward();                                                           
		//System.out.printf("TIME_FALL_LIMIT is %10.0f(ms)\n", TIME_FALL_LIMIT);             
		// NOTE: Balance control loop only takes 0.63ms in EV3 leJOS EVJ 0.9.1-beta 
		// after 10,000,000 iterations                          
		//System.out.println("loopCount is " + loopCount);          
		//System.out.println("Free Memory is " + freeMemory); 
		//System.out.println();   
		fatigue = 0; 
		                                                         
		// while(loopCount<=1E0)  //            1 Done 
		// while(loopCount<=1E1)  //           10 Done 
		// while(loopCount<=1E2)  //          100 Done 
		// while(loopCount<=1E3)  //         1000 Done 
		// while(loopCount<=1E4)  //        10000 Done 
		// while(loopCount<=1E5)  //       100000 Done 
		// while(loopCount<=1E6)  //      1000000 Done 
		// while(loopCount<=1E7)  //     10000000 Done                  
		// while(loopCount<=1E8)  //    100000000 Never finishes 
		// while(loopCount<=1E9)  //   1000000000 
		// while(loopCount<=1E10) //  10000000000 
		mrcSum      = 0;     
		motorDiff   = 0; 
		mrcDeltaP3  = 0; 
		mrcDeltaP2  = 0; 
		mrcDeltaP1  = 0; 
		running = true;          
		//Delay.msDelay(1000);                                                               
		while (running) {         
			 //if (false) {     
		     if (Debug) {    ////////////////////////////////////////////////////////////////////                                                                                      
		    	                                                                               //   
			 			if (loopCount <= TestInterval) {    ///////////////////////////////    //              
							TestIntervalA      [loopCount - 1]  =  TestInterval;         //    //               
							gyroAngleA         [loopCount - 1]  =  gyroAngle;            //    //              
							gyroSpeedA         [loopCount - 1]  =  gyroSpeed;            //    //            
							motorPosA          [loopCount - 1]  =  motorPos;             //    //             
							motorSpeedA        [loopCount - 1]  =  motorSpeed;           //    //      
							gAngleGlobalA      [loopCount - 1]  =  gAngleGlobal;         //    //       
							mrcDeltaA          [loopCount - 1]  =  mrcDelta;             //    //         
							mrcDeltaP1A        [loopCount - 1]  =  mrcDeltaP1;           //    //   
							mrcDeltaP2A        [loopCount - 1]  =  mrcDeltaP2;           //    //      
							mrcDeltaP3A        [loopCount - 1]  =  mrcDeltaP3;           //    //       
							powerLeftA         [loopCount - 1]  =  powerLeft;            //    //       
							powerRightA        [loopCount - 1]  =  powerRight;           //    //      
							mrcLeftA           [loopCount - 1]  =  mrcLeft;              //    //    
							mrcRightA          [loopCount - 1]  =  mrcRight;             //    //    
							tCalcStartA        [loopCount - 1]  =  tCalcStart;           //    //     
							tIntervalA         [loopCount - 1]  =  tInterval;            //    //       
							RunTimeMSA         [loopCount - 1]  =  RunTimeMS;            //    //       
							RunTimeSA          [loopCount - 1]  =  RunTimeS;             //    //       
							motorControlDriveA [loopCount - 1]  =  motorControlDrive;    //    //             
							freeMemoryA        [loopCount - 1]  =  freeMemory;           //    //         
							loopCountA         [loopCount - 1]  =  loopCount;            //    //       
							powerA             [loopCount - 1]  =  power;                //    // 
							tMotorPosOKA       [loopCount - 1]  =  tMotorPosOK;          //    //
									}    //////////////////////////////////////////////////    //           
			 			                                                                       //               
			 			}    ////////////////////////////////////////////////////////////////////             
			calcInterval(cLoop++);       
                                        
			updateGyroData();       
                                          
			updateMotorData();      
                                                                                                                                                             
			// Apply the drive control value to the motor position to get robot to move.  
			motorPos -= motorControlDrive * tSmoothedInterval;  
                                                                              
			// This is the main balancing equation                                                                      
			power = (( KGYROSPEED * gyroSpeed +                 // Deg/Sec from Gyro sensor                 
					   KGYROANGLE * gyroAngle) / ratioWheel +   // Deg from integral of gyro                         
					   KPOS * motorPos +                        // From MotorRotaionCount of both motors    
					   KDRIVE * motorControlDrive +             // To improve start/stop performance 
					   KSPEED * motorSpeed);                    // Motor speed in Deg/Sec 
			//smoothPower = 0.7 * smoothPower + 0.3 * power;      
			//smoothPower = 0.8 * smoothPower + 0.2 * power;      
			//smoothPower = 0.6 * smoothPower + 0.4 * power;      
			smoothPower = 0.7 * smoothPower + 0.3 * power;      
			if (Math.abs(smoothPower) < 100)                   
				tMotorPosOK = System.currentTimeMillis(); 
                                                                                                  
			steerControl((int) smoothPower); // Movement control. Not used for balancing. 
                                                             
			// Apply the power values to the motors 
			left_motor.setPower(powerLeft);   
			right_motor.setPower(powerRight);   
                                                                                              
			// Check if robot has fallen by detecting that motorPos is being limited 
			// for an extended amount of time.                         
			fatigue = System.currentTimeMillis() - tMotorPosOK; 
                                                                       
			RunTimeMS = System.currentTimeMillis() - StartTime; 
			RunTimeS = RunTimeMS * 1E-3;                          
			freeMemory = Runtime.getRuntime().freeMemory(); 
      
		                   	
                                           
			//if (RunTimeS % 20 < 1)                  
				//System.out.println(fatigue);      
			if (fatigue > TIME_FALL_LIMIT)       
				running = false;    
			Delay.msDelay(1);                                                               
			Delay.msDelay(WAIT_TIME - (System.currentTimeMillis() - tCalcStart)); 
			loopCount++;              
		} // end of while() loop 
                                
		//System.out.printf(                                                                                                                                                                                                         
		//		"cLoop %4d\ttSmoothedInterval %5.0f(ms)\tfatique %4d(ms)\t    power %5.0f\tsmoothPower %5.0f\tgyroAngle %5.0f             gyroSpeed  %5.0f      gyroAngle  %5.0f motorPos%5.0f     motorSpeed %5.0f\n", 
		//		cLoop, tSmoothedInterval * 1000, fatigue, power, smoothPower, gyroAngle, KGYROSPEED * gyroSpeed, KGYROANGLE * gyroAngle,     
		//		KPOS * motorPos, KSPEED * motorSpeed);     
		left_motor.flt();        
		right_motor.flt();            
		Sound.beepSequenceUp();                      
		//System.out.println("Oops... I fell");                       
		//System.out.println("tInt ms:" + (tInterval * 1000)); 
		//System.out.println();              
		//System.out.println(fatigue); 
                              
		loopCount--;                                           
		AverageMS = RunTimeMS / loopCount; // 1E6 ms 
		AverageS = RunTimeS / loopCount; // 1E6 ms 
                                                                                                                                                         
		System.out.printf("After %6d iterations and %2.0f minutes %2.0f seconds the average loop time is %5.3f ms\n", loopCount, RunTimeS / 60, 
				RunTimeS % 60, AverageMS);  
		try {             
			bw.write(                                                                                                                                                                                                                                                                                                                                         
					" run,  loopCount, i,    j,     loopCountA[j],  tIntervalA[j],  RunTimeMSA[j],  RunTimeSA[j], gyroRawA[j], gOffsetA[j],  gyroSpeedA[j],  gyroAngleA[j],  motorPosA[j],   mrcDeltaP1A[j],    mrcDeltaP2A[j],    mrcDeltaP3A[j],   motorSpeedA[j],  powerA[j],  powerLeftA[j],  motorControlDriveA[j],       freeMemoryA[j]  "); 
			bw.newLine();                    
		} catch (IOException e1) {                
			// TODO Auto-generated catch block 
			e1.printStackTrace(); 
		}             
                                                         
		for (int i = 1; i <= loopCount; i++) { 
			int j = i - 1;                  
			if (j < TestInterval) { 
				try {                                 
					bw.write(String.format(                                                                                                                                                                                                                                                                                                                                                                        
							" %5d,     %5d,   %10d,  %10d,      %10d,          %10.10f,       %10.10f,       %10.10f,      %10.10f,     %10.10f ,             %10.10f,                   %10.10f,                %10.10f,              %10d,           %10d,          %10d,          %10.10f,             %10.10f,        %10.10f,          %10.10f,                   %10d,         ",                                        
						      run,  loopCount, i,    j,     loopCountA[j],  tIntervalA[j],  RunTimeMSA[j],  RunTimeSA[j], gyroRawA[j], gOffsetA[j],  KGYROSPEED * gyroSpeedA[j],   KGYROANGLE * gyroAngleA[j], KPOS * motorPosA[j],   mrcDeltaP1A[j],    mrcDeltaP2A[j],    mrcDeltaP3A[j],   KSPEED * motorSpeedA[j],  powerA[j],  powerLeftA[j],  motorControlDriveA[j],       freeMemoryA[j]    ));                
					bw.newLine();                                                                                                                                                                                                                                                                                                                                                                                                    
				} catch (IOException e) {                                                            
					// TODO Auto-generated catch block      
					e.printStackTrace();           
				} 
                                                                    
			} /////////////////////// if (j <= loopCount) 
                                                                                  
		} //////////////////////////// for (int j=1; j<=loopCount; j++) 
		running = false;             
		// Delay.msDelay(100); 
		try {                      
			if (bw != null)        
				bw.close();              
		} catch (IOException ex) { 
			ex.printStackTrace(); 
		}                                                    
		//System.out.println("Stop the program here."); 
		//Delay.msDelay(10000);             
		//Button.waitForAnyPress();              
		} // END OF BALANCING THREAD CODE 
		// 
 
                     	
	/**                                                                                 
	 * This method allows the robot to move forward/backward and make in-spot            
	 * rotations as well as arcs by varying the power to each wheel. This method  
	 * does not actually apply direct power to the wheels. Control is filtered               
	 * through to each wheel, allowing the robot to drive forward/backward and make 
	 * turns. Higher values are faster. Negative values cause the wheel to rotate 
	 * backwards. Values between -200 and 200 are good. If values are too high it   
	 * can make the robot balance unstable. 
	 *                           
	 * @param left_wheel                                                                       
	 *            The relative control power to the left wheel. -200 to 200 are good 
	 *            numbers.      
	 * @param right_wheel                                                                
	 *            The relative control power to the right wheel. -200 to 200 are 
	 *            good numbers. 
	 */ 
	//                                                                     
	public void wheelDriver(int left_wheel, int right_wheel) {                                 
		// Set control Drive and Steer. Both these values are in motor degree/second 
		motorControlDrive = (left_wheel + right_wheel) * CONTROL_SPEED / 200.0; 
		motorControlSteer = (left_wheel - right_wheel) * CONTROL_SPEED / 200.0; 
	}            
 
}                               
