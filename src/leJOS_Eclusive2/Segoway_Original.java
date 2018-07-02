package leJOS_Eclusive2;                                                                                                                                                          
                                                                                                                                                                                  
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;                                                                                                                                                                   
import lejos.hardware.sensor.EV3GyroSensor;                                                                                                                                                                   
//import lejos.nxt.Sound; // TODO: Visual count-down only, no sound? Or some sort of sound interface and container for Sound class (can't implement interface on static methods)?                           
import lejos.robotics.EncoderMotor;                                                                                                                                                                     
import lejos.robotics.Gyroscope;
import lejos.robotics.SampleProvider;
//import lejos.util.Delay;                                                                                                                                                                             
import lejos.utility.Delay;                                                                                                                                                                                       
                                                                                                                                                                                                       
                                                                                                                                                                                                   
/**                                                                                                                                                                                                               
 * <p>
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
 * <p><i>This code is based on the HTWay by HiTechnic and the program by BB</i></p>                                                                                                            
 *                                                                                                                                                                                                               
 * @author GloomyAndy 1                                                                                                                                                                                                             
 *                                                                                                                                                                                                                                      
 */                                                                                                                                                                                                                                    
public class Segoway_Original extends Thread
	{ /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////                                                                                                                                  
	                                                                                                                     //                                                                    
    // Motors and gyro:                                                                                                  //                                                                     
	private EV3GyroSensor ev3Gyro;                                                                                       //                                                                              
	private SampleProvider spGyro;                                                                                       //                                                                               
	private float[] gyroSample;                                                                                          //                                                                        
	private EncoderMotor left_motor;                                                                                     //                                                                          
	private EncoderMotor right_motor;                                                                                    //                                                                          
                                                                                                                         //                                                                       
    //=====================================================================                                              //                                                                                            
    // Balancing constants                                                                                               //                                                                         
    //                                                                                                                   //                                                                            
    // These are the constants used to maintain balance.                                                                 //                                                                                 
    //=====================================================================                                              //                                                                                                    
    /**                                                                                                                                                                                          
     * Minimum loop time(ms) passed to the Wait command.
     * NOTE: Balance control loop only takes 0.76ms in EV3 leJOS EVJ 0.9.1-beta!                                                                                          
     */                                                                                                                  //           
    private static final int WAIT_TIME = 10; // originally 8                                                             //                               
                                                                                                                         //                              
    // These are the main four balance constants, only the gyro                                                          //                                                                                      
    // constants are relative to the wheel size.  KPOS and KSPEED                                                        //                                                                                      
    // are self-relative to the wheel size.                                                                              //                                                                                        
    private static final double KGYROANGLE = 15    ;    //    By krchilders                                                                  //                                                                            
	private static final double KGYROSPEED =  0.8  ;    //    By krchilders                                                                //                                                                      
	private static final double KPOS       =  0.12 ;    //    By krchilders                                                              //                                                                              
	private static final double KSPEED     =  0.08 ;    //    By krchilders                                                             //                                                                              
                                                                                                                         //                                               
    /**                                                                                                                                                                    
     * This constant aids in drive control. When the robot starts moving because of user control,                                                                                                           
     * this constant helps get the robot leaning in the right direction.  Similarly, it helps                                                                                                                         
     * bring robot to a stop when stopping.                                                                                                                              
     */                                                                                                                  //                                                 
    private static final double KDRIVE = -0.02;                                                                          //                                                                   
                                                                                                                         //                                                 
    /**                                                                                                                                                                        
     * Power differential used for steering based on difference of target steering and actual motor difference.                                                                                                     
     */                                                                                                                  //                                
    private static final double KSTEER = 0.25;                                                                           //                                                                   
                                                                                                                         //                                                
    /**                                                                                                                                                                      
     * Gyro offset control                                                                                                                                                                      
     * The gyro sensor will drift with time.  This constant is used in a simple long term averaging                                                                                                                       
     * to adjust for this drift. Every time through the loop, the current gyro sensor value is                                                                                                             
     * averaged into the gyro offset weighted according to this constant. The value is                                                                                                                           
     * set to reset the offset over a 5 second period.                                                                                                                                                     
     */                                                                                                                  //                                                     
    //private static final double EMAOFFSET = 0.0005;        //    By krchilders                                         //                                                 	                              
    private static final double EMAOFFSET = WAIT_TIME/(1000.0*5);                                                        //                                                                                    
                                                                                                                         //                                                       
    /**                                                                                                                                                
     * If robot power is saturated (over 100%) for over this time limit(ms) then                                                                                                                                      
     * robot has fallen.                                                                                                                                                
     */                                                                                                                  //                                 
    private static final double TIME_FALL_LIMIT = 500; // originally 1000                                                //                                                                         
                                                                                                                         //                                                 
    //---------------------------------------------------------------------                                              //                                                                             
                                                                                                                         //                          
    /**                                                                                                                                                                                              
     * Maximum speed(degrees/second)  Note that position                                                                                                                        
     * and speed are measured as the sum of the two motors, in other words, 600                                                                                                                             
     * would actually be 300 degrees/second for each motor.                                                                                                                                               
     */                                                                                                                  //                                                                                
    private static final double CONTROL_SPEED  = 600.0;                                                                  //                                                                               
                                                                                                                         //                                                      
    //=====================================================================                                              //                                                                           
    // Global variables                                                                                                                  //                                                                 
    //=====================================================================                                                              //                                                                                               
                                                                                                                                         //                                                                       
    // These two xxControlDrive variables are used to control the movement of the robot. Both                                            //                                                                                                    
    // are in degrees/second:                                                                                                            //                                                       
    /**                                                                                                                                                                                  
     * Target speed(degrees per second) for the sum of the two motors.                                                                                                                                                                                   
     */                                                                                                                                  //                                                  
    private double motorControlDrive = 0.0;                                                                                              //                                                                      
                                                                                                                                         //                                                                    
    /**                                                                                                                                                                                                           
     * Target change in difference(degrees per second) for two motors.                                                                                                                                                                                       
     */                                                                                                                                  //                                                      
    private double motorControlSteer = 0.0;                                                                                              //                                                                                      
                                                                                                                                         //                                                                                       
    /**                                                                                                                                                                                                                                 
     * This global contains the target motor differential, essentially, which                                                                                                                                                                 
     * way the robot should be pointing.  This value is updated every time through                                                                                                                                                                
     * the balance loop based on motorControlSteer.                                                                                                                                                                                     
     */                                                                                                                                  //                                                                                            
    private double motorDiffTarget = 0.0;                                                                                                //                                                                                   
                                                                                                                                         //                                                                                       
    /**                                                                                                                                                                                                         
     * Time robot first starts to balance.  Used to calculate tInterval.                                                                                                                                                             
     */                                                                                                                                  //                                                                                                
    private long tCalcStart;                                                                                                             //                                                                                            
                                                                                                                                         //                                                                                        
    /**                                                                                                                                                                                                                               
     * Time(seconds) for each iteration of the balance loop.                                                                                                                                                                 
     */                                                                                                                                  //                                                                       
    private double tInterval;                                                                                                            //                                                                              
                                                                                                                                         //                                                                                 
    /**                                                                                                                                                                                             
     * Smoothed version of tInterval                                                                                                                                                                                  
     */                                                                                                                                  //                                                                     
    double tSmoothedInterval;                                                                                                            //                                                                          
                                                                                                                                         //                                                        
    /**                                                                                                                                                                                                               
     * Relative wheel size compared to a standard NXT 1.0 wheel.                                                                                                                                                    
     */                                                                                                                                  //                                                                                           
    private double ratioWheel;                                                                                                           //                                                                                 
                                                                                                                                         //                                                                                        
    // Gyro globals                                                                                                                      //                                                                                                  
    private double gOffset;                                                                                                              //                                                                                              
    private double gyroSpeed, gyroAngle;                                                                                                 //                                                                                 
    private double gAngleGlobal = 0;                                                                                                     //                                                  
	private boolean invertGyro = false;                                                                                                  //                                                       
	                                                                                                                                     //                                                                                         
    // Motor globals                                                                                                                     //                                                                                             
    private double motorPos = 0;                                                                                                         //                                                                            
    private long mrcSum = 0, mrcSumPrev;                                                                                                 //                                                                                     
    private long motorDiff;                                                                                                              //                                                                           
    private long mrcDeltaP3 = 0;                                                                                                         //                                                                                   
    private long mrcDeltaP2 = 0;                                                                                                         //                                                                               
    private long mrcDeltaP1 = 0;                                                                                                         //                                                                            
                                                                                                                                         //                                                                     
    /**                                                                                                                                  //                                                                            
     * Global variables used to control the amount of power to apply to each wheel.                                                      //                                                                                           
     * Updated by the steerControl() method.                                                                                             //                                                                                       
     */                                                                                                                                  //                                                                          
    private int powerLeft, powerRight;                                                                                                   //                                                                                  
    private double motorSpeed;                                                                                                           //                                                                                 
                                                                                                                                         //                                                                     
    //private static boolean running = false;                                                                                              //                                                                                        
    private static boolean running = true;                                                                                              //                                                                                        
                                                                                                                                         //                                              
    // Logging                                                                                                                           //                                       
    /*                                                                                                                                   //                                                                      
    private static final int LOG_SZ = 100;                                                                                               //                                                           
    private double [] gaLog = new double[LOG_SZ];                                                                                        //                                                         
    private double [] gsLog = new double[LOG_SZ];                                                                                        //                                                    
    private double [] intLog = new double[LOG_SZ];                                                                       //                                
    private int logIndex = 0;                                                                                            //                                         
    */                                                                                                                   //                                         
                                                                                                                         //                                            
                                                                                                                         //                                                      
    /**                                                                                                                  //                                  
	 * Creates an instance of the Segoway, prompts the user to steady Segoway                                            //                                                                
	 * for gyro calibration, then begins self-balancing thread. Wheel diameter                                           //                                                                                                 
	 * is used in balancing equations.                                                                                   //                                                                                                   
	 *                                                                                                                   //                                      
	 * <li>NXT 1.0 wheels = 5.6 cm                                                                                       //                                                     
	 * <li>NXT 2.0 wheels = 4.32 cm                                                                                      //                                  
	 * <li>RCX "motorcycle" wheels = 8.16 cm                                                                             //                                                                  
	 * <li>EV3 wheels = 5.6 cm                                                                                           //                                                               
	 *                                                                                                                   //                                     
	 * @param left The left motor. An unregulated motor.                                                                 //                                                       
	 * @param right The right motor. An unregulated motor.                                                               //                                                          
	 * @param ev3Gyro A EV3 gyro sensor                                                                                  //                                                        
	 * @param invertGyro Whether or not gyro readings should be inverted, depends                                        //                                                          
	 *                   on sensor orientation.                                                                          //                                                     
	 * @param wheelDiameter diameter of wheel, preferably use cm (printed on side                                        //                                                        
	 *        of LEGO tires in mm)                                                                                       //                        
	 */                                                                                                                  //                       
	//public Segoway(EncoderMotor left, EncoderMotor right, EV3GyroSensor ev3Gyro,                                       //                       	   
	//        boolean invertGyro, double wheelDiameter)                                                                  //                           
	public Segoway_Original(UnregulatedMotor left, UnregulatedMotor right, EV3GyroSensor ev3Gyro,                       //                           
			boolean invertGyro, double wheelDiameter)                                                                    //                             
		{    ///////////////////////////////////////////////////////////                                                 //                                                                                               
		this.left_motor = left;                                       //                                                 //                                
		this.right_motor = right;                                     //                                                 //                                    
		                                                              //                                                 //                    
		this.ev3Gyro = ev3Gyro;                                       //                                                 //                                    
		this.invertGyro = invertGyro;                                 //                                                 //                                                               
		this.spGyro = ev3Gyro.getRateMode();                          //                                                 //                                                               
		gyroSample = new float[spGyro.sampleSize()];                  //                                                 //                                                                 
                                                                      //                                                 //                                  
		// Original algorithm was tuned for 5.6 cm NXT 1.0 wheels.    //                                                 //                                                              
		this.ratioWheel = wheelDiameter / 5.6;                        //                                                 //                                                                 
                                                                      //                                                 //                                   
		// Get the initial gyro offset and calibrate sensor.          //                                                 //                                                               
		calibrateGyro();                                              //                                                 //                                                    
                                                                      //                                                 //                                     
		// Play warning beep sequence before balance starts           //                                                 //                                                                 
		startBeeps();                                                 //                                                 //                                                  
		}    ///////////////////////////////////////////////////////////                                                 //                                                                                                
                                                                                                                         //                                                                        
	                                                                                                                     //
	/**                                                                                                                  //                              
	 * This function returns a suitable initial gyro offset. It takes 200 gyro                                           //                                                             
	 * samples over a time of 1 second and averages them to get the offset. It                                           //                                                                                          
	 * also checks the maximum and minimum during that time and if the difference is larger                              //                                                                                          
	 * than one (1), it rejects the data and gets another set of samples.                                                //                                                                                               
	 */                                                                                                                  //                                     
	private void calibrateGyro()                                                                                         //                                    
		{    /////////////////////////////////////////////////////////////////////////                                   //                                                                                                              
		LCD.clear();                                                                //                                   //                                     
		LCD.drawString("EV3 Segoway", 0, 0);                                        //                                   //                                                                
		LCD.drawString("Steady robot", 0, 2);                                       //                                   //                                                               
		LCD.drawString("to calibrate", 0, 3);                                       //                                   //                                                                  
		LCD.drawString("the gyro", 0, 4);                                           //                                   //                                                                 
                                                                                    //                                   //                             
		double gSum;                                                                //                                   //                             
		int i, gMin, gMax, g;                                                       //                                   //                                
                                                                                    //                                   //                                 
		do                                                                          //                                   //                             
			{    /////////////////////////////////////////////                      //                                   //                               
			ev3Gyro.reset();                                //                      //                                   //                               
                                                            //                      //                                   //                                 
			gSum = 0.0;                                     //                      //                                   //                               
			gMin = 1000;                                    //                      //                                   //                               
			gMax = -1000;                                   //                      //                                   //                                
                                                            //                      //                                   //                               
			for (i = 1; i <= 200; i++)                      //                      //                                   //                               
				{    ///////////////////////////////////    //                      //                                   //                                                                   
				spGyro.fetchSample(gyroSample, 0);    //    //                      //                                   //                                                        
				g = (int) gyroSample[0];              //    //                      //                                   //                                              
                                                      //    //                      //                                   //                    
				if (g > gMax)                         //    //                      //                                   //                                            
					{    //////////                   //    //                      //                                   //                                     
					gMax = g;    //                   //    //                      //                                   //                                     
					}    //////////                   //    //                      //                                   //                                              
				if (g < gMin)                         //    //                      //                                   //                                  
					{    //////////                   //    //                      //                                   //                                             
					gMin = g;    //                   //    //                      //                                   //                                     
					}    //////////                   //    //                      //                                   //                                               
                                                      //    //                      //                                   //                                   
				gSum += g;                            //    //                      //                                   //                                    
				try                                   //    //                      //                                   //                                   
					{    /////////////////            //    //                      //                                   //                                                     
					Thread.sleep(5);    //            //    //                      //                                   //                                     
					}    /////////////////            //    //                      //                                   //                                                       
				catch (InterruptedException e)        //    //                      //                                   //                                                                  
					{    //////////                   //    //                      //                                   //                                                          
					// Ignore    //                   //    //                      //                                   //                                            
					}    //////////                   //    //                      //                                   //                                                              
				}    ///////////////////////////////////    //                      //                                   //                                                                                    
			                                                //                      //                                   //                                                                                                              
			}    /////////////////////////////////////////////                      //                                   //                                                                                                                                                                                                                       
		while ((gMax - gMin) > 1); // Reject and sample again if range too large    //                                   //                                                                                    
                                                                                    //                                   //                                        
		// Average the sum of the samples.                                          //                                   //                                                                            
		// Used to have +1, which was mainly for stopping Segoway wandering.        //                                   //                                                                                                          
		gOffset = gSum / 1000;                                                      //                                   //                                                                     
                                                                                    //                                   //                                              
		if (invertGyro)                                                             //                                   //                                             
			{    ////////////////////////                                           //                                   //                                                                          
			gOffset = gOffset * -1;    //                                           //                                   //                                                
			}    ////////////////////////                                           //                                   //                                                                     
                                                                                    //                                   //                                                                                                         
		}    /////////////////////////////////////////////////////////////////////////                                   //                                                                                                                  
	                                                                                                                     //                                                                                                      
	                                                                                                                     //
	/**                                                                                                                  //                                                                       
     * Warn user the Segoway is about to start balancing.                                                                //                                                                                   
     */                                                                                                                  //                                                                          
    private void startBeeps()                                                                                            //                              
    	{    /////////////////////////////////////////////////////////////////                                           //                                                                                                                        
                                                                            //                                           //                                                                           
        System.out.println("Balance in");                                   //                                           //                                                                                      
                                                                            //                                           //                                                                                          
        // Play warning beep sequence to indicate balance about to start    //                                           //                                                                                       
        for (int c=5; c>0;c--)                                              //                                           //              
        	{    //////////////////////////////////                         //                                           //                                                                                                  
            System.out.print(c + " ");           //                         //                                           //                                                                                     
            Sound.playTone(440,100);             //                         //                                           //                                                                  
            try                                  //                         //                                           //                            
            	{    ////////////////////        //                         //                                           //                                                  
            	Thread.sleep(1000);    //        //                         //                                           //                                                                 
            	}    ////////////////////        //                         //                                                           //                                                     
            catch (InterruptedException e) {}    //                         //                                                           //                                                                           
        	}    //////////////////////////////////                         //                                                           //                                                                                                
        System.out.println("GO");                                           //                                                           //                                                           
        System.out.println();                                               //                                                           //                                                                          
    	}    /////////////////////////////////////////////////////////////////                                                           //                                                                                              
                                                                                                                                         //                                             
                                                                                                                                         //                                    
    /**                                                                                                                                  //                                                    
     * Get the data from the gyro.                                                                                                       //                                                        
     * Fills the pass by reference gyroSpeed and gyroAngle based on updated information from the Gyro Sensor.                            //                                                                                                            
     * Maintains an automatically adjusted gyro offset as well as the integrated gyro angle.                                             //                                                                                           
     *                                                                                                                                   //                                                                           
     */                                                                                                                                  //                                                                             
    private void updateGyroData()                                                                                                        //                                      
    	{    /////////////////////////////////////////////////////////////////////////////                                               //                                                                                                                                                                                   
    	// NOTE: The GyroSensor class actually rebaselines for drift ever 5 seconds.    //                                               //                                                                                  
    	// This not needed? Or is this method better? Some of this fine tuning may      //                                               //                                                                                                                  
    	// actually interfere with fine-tuning happening in the hardcoded dIMU and      //                                               //                                                                                                             
    	// GyroScope code. As of EV3 0.8.1-beta drift accounting is still needed.       //                                               //                                                                                                                
    	float gyroRaw;                                                                  //                                               //                                                                
    	spGyro.fetchSample(gyroSample, 0);                                              //                                               //                                                                                
    	                                                                                //                                               //                                                 
    	gyroRaw = gyroSample[0]; // deg/s                                               //                                               //                                                                              
    	                                                                                //                                               //                                         
    	if (invertGyro)                                                                 //                                               //                                       
    		{    //////////////////////////                                             //                                               //                                                                          
    		gyroRaw = (-1 * gyroRaw);    //                                             //                                               //                                                  
    		}    //////////////////////////                                             //                                               //                                                                       
    	                                                                                //                                               //                                              
    	gOffset = EMAOFFSET * gyroRaw + (1 - EMAOFFSET) * gOffset;                      //                                               //                                            
    	gyroSpeed = gyroRaw - gOffset; // Angular velocity (degrees/sec)                //                                               //                                                                              
    	                                                                                //                                               //                                           
    	gAngleGlobal += gyroSpeed * tInterval;                                          //                                               //                                                                                                              
    	gyroAngle = gAngleGlobal; // Absolute angle (degrees)                           //                                               //                                                                                                          
    	/*                                                                              //                                               //                                                                                                   
        gaLog[logIndex] = gyroAngle;                                                    //                                               //                                                                                     
        gsLog[logIndex] = gyroSpeed;                                                    //                                               //                                                                                     
        intLog[logIndex] = tInterval;                                                   //                                               //                                                                                  
        logIndex = (logIndex + 1) % LOG_SZ;                                             //                                               //                                                                                                      
        */                                                                              //                                               //                                                                                                  
    	}    /////////////////////////////////////////////////////////////////////////////                                               //                                                                                                                                                                                                                       
                                                                                                                                         //                                                                                          
                                                                                                                                         //                                                          
    /**                                                                                                                                  //                                                                                   
     * Keeps track of wheel position with both motors.                                                                                   //                                                                                      
     */                                                                                                                                  //                                                                              
    private void updateMotorData()                                                                                                       //                                                
    	{    ////////////////////////////////////////////////////////////////////////////////                                            //                                                                                                                                            
        long mrcLeft, mrcRight, mrcDelta;                                                  //                                            //                                                                                          
                                                                                           //                                            //                                                                                             
        // Keep track of motor position and speed                                          //                                            //                                                                                                 
        mrcLeft = left_motor.getTachoCount();                                              //                                            //                                                                                              
        mrcRight = right_motor.getTachoCount();                                            //                                            //                                                                                              
                                                                                           //                                            //                                                                                          
        // Maintain previous mrcSum so that delta can be calculated and get                //                                            //                                                                                                  
        // new mrcSum and Diff values                                                      //                                            //                                                                                                        
        mrcSumPrev = mrcSum;                                                               //                                            //                                                                                               
        mrcSum = mrcLeft + mrcRight;                                                       //                                            //                                                                                                       
        motorDiff = mrcLeft - mrcRight;                                                    //                                            //                                                                                                          
                                                                                           //                                            //                                                                                          
        // mrcDetla is the change int sum of the motor encoders, update                    //                                            //                                                                                                 
        // motorPos based on this detla                                                    //                                            //                                                                                                             
        mrcDelta = mrcSum - mrcSumPrev;                                                    //                                            //                                                                                         
        motorPos += mrcDelta;                                                              //                                            //                                                                                                 
                                                                                           //                                            //                                                                                         
        // motorSpeed is based on the average of the last four delta's.                    //                                            //                                                                                                    
        motorSpeed = (mrcDelta+mrcDeltaP1+mrcDeltaP2+mrcDeltaP3)/(4*tSmoothedInterval);    //                                            //                                                                                                          
                                                                                           //                                            //                                                                      
        // Shift the latest mrcDelta into the previous three saved delta values            //                                            //                                                                                    
        mrcDeltaP3 = mrcDeltaP2;                                                           //                                            //                                                      
        mrcDeltaP2 = mrcDeltaP1;                                                           //                                            //                                                        
        mrcDeltaP1 = mrcDelta;                                                             //                                            //                                                                      
    	}    ////////////////////////////////////////////////////////////////////////////////                                            //                                                                                                                                                         
                                                                                                                                         //                                                                 
                                                                                                                                         //                                                                   
    /**                                                                                                                                  //                                                          
     * This function determines the left and right motor power that should                                                               //                                                                            
     * be used based on the balance power and the steering control.                                                                      //                                                                                      
     */                                                                                                                                  //                                                                                      
    private void steerControl(int power)                                                                                                 //                                                       
    	{    ///////////////////////////////////////////////////////////////////                                                         //                                                                                                                     
        int powerSteer;                                                       //                                                         //                                                                                          
                                                                              //                                                         //                                                                    
        // Update the target motor difference based on the user steering      //                                                         //                                                                                                
        // control value.                                                     //                                                         //                          
        motorDiffTarget += motorControlSteer * tSmoothedInterval;             //                                                         //                                                                     
                                                                              //                                                         //                                               
        // Determine the proportionate power differential to be used based    //                                                         //                                                                               
        // on the difference between the target motor difference and the      //                                                         //                                                                         
        // actual motor difference.                                           //                                                         //                                                            
        powerSteer = (int)(KSTEER * (motorDiffTarget - motorDiff));           //                                                         //                                                                   
                                                                              //                                                         //                                                
        // Apply the power steering value with the main power value to        //                                                         //                                                                       
        // get the left and right power values.                               //                                                         //                                                                   
        powerLeft = power + powerSteer;                                       //                                                         //                                         
        powerRight = power - powerSteer;                                      //                                                         //                                              
                                                                              //                                                         //                                              
        // Limit the power to motor power range -100 to 100                   //                                                         //                                                            
        if (powerLeft > 100)   powerLeft = 100;                               //                                                         //                                                    
        if (powerLeft < -100)  powerLeft = -100;                              //                                                         //                                               
                                                                              //                                                         //                                            
        // Limit the power to motor power range -100 to 100                   //                                                         //                                                               
        if (powerRight > 100)  powerRight = 100;                              //                                                         //                                                    
        if (powerRight < -100) powerRight = -100;                             //                                                         //                                                  
    	}    ///////////////////////////////////////////////////////////////////                                                         //                                                                                                         
                                                                                                                                         //                                                                           
                                                                                                                                         //
    /**                                                                                                                                  //                                         
     * Calculate the interval time from one iteration of the loop to the next.                                                           //                                     
     * Note that first time through, cLoop is 0, and has not gone through                                                                //                                         
     * the body of the loop yet.  Use it to save the start time.                                                                         //                                                             
     * After the first iteration, take the average time and convert it to                                                                //                                                    
     * seconds for use as interval time.                                                                                                 //                                              
     */                                                                                                                                  //                                         
    private void calcInterval(long cLoop)                                                                                                //                                                   
    	{    ///////////////////////////////////////////////////////////////////////                                                     //                                                                                                      
        if (cLoop == 0)                                                           //                                                     //                       
        	{    /////////////////////////////////////////////////////////        //                                                     //                                                                                               
            // First time through, set an initial tInterval time and    //        //                                                     //                                                                     
            // record start time                                        //        //                                                     //                                                                                 
            tInterval = WAIT_TIME/1000.0;                               //        //                                                     //                                                 
            tSmoothedInterval = tInterval;;                             //        //                                                     //                                                   
            tCalcStart = System.currentTimeMillis();                    //        //                                                     //                                                        
        	}    /////////////////////////////////////////////////////////        //                                                     //                                                                            
        else                                                                      //                                                     //                   
        	{    /////////////////////////////////////////////////////////////    //                                                     //                                                                                                    
            // Take average of number of times through the loop and         //    //                                                     //                                                                      
            // use for interval time.                                       //    //                                                     //                                                                
            long tNew = System.currentTimeMillis();                         //    //                                                     //                                                          
            tInterval = (tNew - tCalcStart)/1000.0;                         //    //                                                     //                                                        
            tSmoothedInterval = 0.75*tSmoothedInterval + 0.25*tInterval;    //    //                                                     //                                                                                   
            tCalcStart = tNew;                                              //    //                                                     //                                                           
        	}    /////////////////////////////////////////////////////////////    //                                                     //                                                                                   
                                                                                  //                                                     //                                  
    	}    ///////////////////////////////////////////////////////////////////////                                                     //                                                                                                          
                                                                                                                                         //                                                                 
                                                                                                                                         //                                                                                           
    /**                                                                                                                                  //                              
     * Is the control loop still running                                                                                                 //                                         
     * @return true if the loop is running.                                                                                              //                                           
     */                                                                                                                                  //                              
    public static boolean isRunning()                                                                                                    //                                         
    	{    ////////////////                                                                                                            //                                                          
        return running;    //                                                                                                            //                            
    	}    ////////////////                                                                                                            //                                                     
                                                                                                                                         //                                                                    
                                                                                                                                         //                                                
    /**                                                                                                                                  //                               
     * Stop the control loop                                                                                                             //                                 
     */                                                                                                                                  //                                
    public void halt()                                                                                                                   //                                            
    	{    /////////////////                                                                                                           //                                                 
        running = false;    //                                                                                                           //                                 
    	}    /////////////////                                                                                                           //                                                                      
                                                                                                                                         //                             
    //---------------------------------------------------------------------                                                              //                                           
    //                                                                                                                                   //                                     
    // This is the main balance thread for the robot.                                                                                    //                                       
    //                                                                                                                                   //                               
    // Robot is assumed to start leaning on a wall.  The first thing it                                                                  //                                              
    // does is take multiple samples of the gyro sensor to establish and                                                                 //                                                
    // initial gyro offset.                                                                                                              //                                      
    //                                                                                                                                   //                                          
    // After an initial gyro offset is established, the robot backs up                                                                   //                                                                                          
    // against the wall until it falls forward, when it detects the                                                                      //                                                                     
    // forward fall, it start the balance loop.                                                                                          //                                                 
    //                                                                                                                                   //                                             
    // The main state variables are:                                                                                                     //                                        
    // gyroAngle  This is the angle of the robot, it is the results of                                                                   //                                                                
    //            integrating on the gyro value.                                                                                         //                                                  
    //            Units: degrees                                                                                                         //                           
    // gyroSpeed  The value from the Gyro Sensor after offset subtracted                                                                 //                            
    //            Units: degrees/second                                                                                                  //                                          
    // motorPos   This is the motor position used for balancing.                                                                         //                                                  
    //            Note that this variable has two sources of input:                                                                      //                                                   
    //             Change in motor position based on the sum of                                                                          //                                                  
    //             MotorRotationCount of the two motors,                                                                                 //                                                         
    //            and,                                                                                                                   //                                             
    //             forced movement based on user driving the robot.                                                                      //                                                    
    //            Units: degrees (sum of the two motors)                                                                                 //                                                 
    // motorSpeed This is the speed of the wheels of the robot based on the                                                              //                                                                              
    //            motor encoders.                                                                                                        //                                    
    //            Units: degrees/second (sum of the two motors)                                                                          //                                                              
    //                                                                                                                                   //                                           
    // From these state variables, the power to the motors is determined                                                                 //                                                                                                 
    // by this linear equation:                                                                                                          //                                                   
    //     power = KGYROSPEED * gyro +                                                                                                   //                                                                                 
    //             KGYROANGLE * gyroAngle +                                                                                              //                                            
    //             KPOS       * motorPos +                                                                                               //                                                                
    //             KSPEED     * motorSpeed;                                                                                              //                                                                
    //                                                                                                                                   //                              
    public void run()                                                                                                                    //                                            
    	{    ////////////////////////////////////////////////////////////////////////////////////////////////////////    //                //                                                                                                                                           
                                                                                                                   //    //                //                             
        double power;                                                                                              //    //                //                                                 
        double smoothPower = 0.0;                                                                                  //    //                //                                                         
        long tMotorPosOK;                                                                                          //    //                //                                       
        long cLoop = 0;                                                                                            //    //                //                                                  
                                                                                                                   //    //                //                                           
        System.out.println("Balancing");                                                                           //                    //                                           
        System.out.println();                                                                                      //                    //                                                    
                                                                                                                   //                    //                                      
        tMotorPosOK = System.currentTimeMillis();                                                                  //                    //                                                         
                                                                                                                   //                    //                                  
        // Reset the motors to make sure we start at a zero position                                               //                    //                                                  
        left_motor.resetTachoCount();                                                                              //                    //                                                             
        right_motor.resetTachoCount();                                                                             //                    //                                                                
        //running = true;                                                                                            //                    //                                                    
        right_motor.forward();                                                                                     //                    //                                                           
        left_motor.forward();                                                                                      //                    //                                                          
        // NOTE: This balance control loop only takes 1.128 MS to execute each loop in leJOS NXJ.                  //                    //                                                                             
        while(running)                                                                                             //                    //                       
        	{    /////////////////////////////////////////////////////////////////////////////////////////         //                    //                                                                                                                                                             
            calcInterval(cLoop++);                                                                      //         //                    //                                                                                     
                                                                                                        //         //                    //                                                                          
            updateGyroData();                                                                           //         //                    //                                                                
                                                                                                        //         //                    //                                                                           
            updateMotorData();                                                                          //         //                    //                                                                                  
                                                                                                        //         //                    //                                                                           
            // Apply the drive control value to the motor position to get robot to move.                //         //                    //                                                                                                  
            motorPos -= motorControlDrive * tSmoothedInterval;                                          //         //                    //                                                                                              
                                                                                                        //         //                    //                                                                              
            // This is the main balancing equation                                                      //         //                    //                                                                    
            power = ((KGYROSPEED * gyroSpeed +               // Deg/Sec from Gyro sensor                //         //                    //                                                                                               
                      KGYROANGLE * gyroAngle) / ratioWheel + // Deg from integral of gyro                 //         //                    //                                                                                                  
                      KPOS       * motorPos +                // From MotorRotaionCount of both motors    //         //                    //                                                                                         
                      KDRIVE     * motorControlDrive +       // To improve start/stop performance        //         //                    //                                                                                               
                      KSPEED     * motorSpeed);              // Motor speed in Deg/Sec                  //         //                    //                                                                                  
            smoothPower = 0.7*smoothPower + 0.3*power;                                                  //         //                    //                                                                               
            if (Math.abs(smoothPower) < 100)                                                            //         //                    //                                                                  
                tMotorPosOK = System.currentTimeMillis();                                               //         //                    //                                                                   
                                                                                                        //         //                    //                                                                                 
            steerControl((int)smoothPower); // Movement control. Not used for balancing.                //         //                    //                                                                                                                 
                                                                                                        //         //                    //                                                              
            // Apply the power values to the motors                                                     //         //                    //                                                                                            
            left_motor.setPower(powerLeft);                                                             //         //                    //                                                                                                  
            right_motor.setPower(powerRight);                                                           //         //                    //                                                                                   
                                                                                                        //         //                    //                                                                                   
            // Check if robot has fallen by detecting that motorPos is being limited                    //         //                    //                                                                                                   
            // for an extended amount of time.                                                          //         //                    //                                                                                  
            if ((System.currentTimeMillis() - tMotorPosOK) > TIME_FALL_LIMIT) running = false;          //         //                    //                                                                                                             
            //Delay.msDelay(1);                                                                           //         //                    //                                                                                                                     
            Delay.msDelay(WAIT_TIME - (System.currentTimeMillis() - tCalcStart));                       //         //                    //                                                                                             
        	} // end of while() loop    //////////////////////////////////////////////////////////////////         //                    //                                                                                                                               
                                                                                                                   //                    //                                                          
        left_motor.flt();                                                                                          //                    //                                                          
        right_motor.flt();                                                                                         //                    //                                                       
        //running = false;                                                                                         //                    //                                                                            
        Sound.beepSequenceUp();                                                                                    //                    //                                                                    
        System.out.println("Oops... I fell");                                                                      //                    //                                                                                
        System.out.println("tInt ms:" + (tInterval*1000));                                                         //                    //                                                                        
        System.out.println();                                                                                      //                    //                                                                 
        /*                                                                                                         //                    //                                                                
        for(int i = 0; i < LOG_SZ; i++)                                                                            //                    //                                                                      
        {                                                                                                          //                    //                                                                
            System.out.println("I " + intLog[logIndex] + " ga " + gaLog[logIndex] + " gs " + gsLog[logIndex]);     //                    //                                                                                  
            logIndex = (logIndex + 1) % LOG_SZ;                                                                    //                    //                                                                     
        }*/                                                                                                        //                    //                                                             
    	} // END OF BALANCING THREAD CODE    ////////////////////////////////////////////////////////////////////////                    //                                                                                                                                                           
                                                                                                                                         //                                                                 
    /**                                                                                                                                  //                                                                           
     * This method allows the robot to move forward/backward and make in-spot rotations as                                               //                                                                          
     * well as arcs by varying the power to each wheel. This method does not actually                                                    //                                                                                       
     * apply direct power to the wheels. Control is filtered through to each wheel, allowing the robot to                                //                                                                                                        
     * drive forward/backward and make turns. Higher values are faster. Negative values cause the wheel                                  //                                                                                                     
     * to rotate backwards. Values between -200 and 200 are good. If values are too high it can make the                                 //                                                                                              
     * robot balance unstable.                                                                                                           //                                                                            
     *                                                                                                                                   //                                                               
     * @param left_wheel The relative control power to the left wheel. -200 to 200 are good numbers.                                     //                                                                          
     * @param right_wheel The relative control power to the right wheel. -200 to 200 are good numbers.                                   //                                                                               
     */                                                                                                                                  //                                                                        
                                                                                                                                         //                                                                         
    public void wheelDriver(int left_wheel, int right_wheel)                                                                             //                                     
    	{     //////////////////////////////////////////////////////////////////////////////                                             //                                                                                                                                  
        // Set control Drive and Steer.  Both these values are in motor degree/second     //                                             //                                                                                                            
        motorControlDrive = (left_wheel + right_wheel) * CONTROL_SPEED / 200.0;           //                                             //                                                                                                        
        motorControlSteer = (left_wheel - right_wheel) * CONTROL_SPEED / 200.0;           //                                             //                                                                                 
    	}    ///////////////////////////////////////////////////////////////////////////////                                             //                                                                                                                     
                                                                                                                                         //                                                                
	}    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////         