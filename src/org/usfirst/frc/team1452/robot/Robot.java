package org.usfirst.frc.team1452.robot;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//smartdashboard stuff
    final String left = "Left";
    final String right = "Right";
    final String center = "Center";
    String autoSelected;
    SendableChooser chooser;
    
    //sensor stuff
    AnalogInput frontRangefinder = new AnalogInput(1);
    AnalogInput backRangefinder = new AnalogInput(2);
    AnalogGyro gyro = new AnalogGyro(0);
    UsbCamera mainCamera;
    UsbCamera winchCamera;
    DigitalInput clawButton = new DigitalInput(0);
    
    //joystick stuff
    Boolean[] buttonState = new Boolean[12];//THE BUTTON STATES ARE 1 LESS THAN THE NUMBER PRINTED ON THE JOYSTICK
    Boolean[] prevButtonState = new Boolean[12];
    Joystick j = new Joystick(0);
    Double throttleValue;
    
    //motor stuff                  front left       back left        front right      back right       winch 1          winch 2
    CANTalon[] CANTalonList = {new CANTalon(0), new CANTalon(1), new CANTalon(2), new CANTalon(3), new CANTalon(4), new CANTalon(5)};
    RobotDrive drive = new RobotDrive(CANTalonList[0], CANTalonList[1], CANTalonList[2], CANTalonList[3]);
    double autonMotorSpeed = 0;
    
    //pneumatics stuff
    Compressor c = new Compressor();    
    DoubleSolenoid grabClaw = new DoubleSolenoid(0,1);
    Solenoid liftClaw = new Solenoid(2);

    //misc stuff
    Boolean firstTime = true;
    int autonomousStep = 0;
    Timer timer = new Timer();
    boolean clawButtonState;
    boolean prevClawButtonState;
    final double FULL_SPEED = 1.0;
	final double THREE_QUARTERS_SPEED = .75;
	final double HALF_SPEED = .5;
	final double ONE_QUARTER_SPEED = .5;
    
    //image processing stuff
    private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	int visionRunning = 0;
	int visionFinding = 0;
	int prevRunning;
	double[] prevTurnValues = new double [25];
	double turnAverage = 0;
	int valsUsed = 0;
	private VisionThread processingThread;
	private Thread visionThread;
	private double centerX = 0.0;
	boolean cam0 = true;
    Mat image = new Mat();
	
	private final Object imgLock = new Object();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	//Ramp for motors
	public double rampDown(double value, double increment, double limit){
		if(value<limit){
    		value+=increment;
    	}
    	if(value>limit){
    		value = limit;
    	}
    	return value;
    }
	public double rampUp(double value, double increment, double limit){
    	if(value>limit){
    		value-=increment;
    	}
    	if(value<limit){
    		value = limit;
    	}
    	return value;
    }
	///////////////
	
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Center", center);
        chooser.addObject("Left", left);
        chooser.addObject("Right", right);
        SmartDashboard.putData("Auto choices", chooser);
        c.setClosedLoopControl(true);
        
        drive.setInvertedMotor(MotorType.kFrontRight, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
        
        //CAMERA STUFF
		for(int i = 0; i<10; i++){
			prevTurnValues[i] = 0;
		}
		
		UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture(0);
		UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(1);
	    camera0.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    camera1.setResolution(IMG_WIDTH, IMG_HEIGHT);
    	visionFinding = 0;
    	visionRunning = 0;
    	/////
    	
	    ///VISION PROCESSING THREAD
	    processingThread = new VisionThread(camera0, new GripPipeline(), pipeline -> {
	        if (pipeline.filterContoursOutput().size()>=2) {
	            Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            Rect r2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
	            synchronized (imgLock) {
	                centerX = ((r1.x + (r1.width / 2))+(r2.x + (r2.width / 2)))/2;
		            SmartDashboard.putNumber("Center X", centerX);
	            }
	            visionFinding++;
	            SmartDashboard.putNumber("Vision Finding", visionFinding);
	        }else{
	        	centerX = IMG_WIDTH/2;
	        }


            visionRunning++;
            SmartDashboard.putNumber("Vision Running", visionRunning);
	    });
	    processingThread.start();
	    
	    visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			// Set the resolution

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink0 = CameraServer.getInstance().getVideo(camera0);
			CvSink cvSink1 = CameraServer.getInstance().getVideo(camera1);

			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Vision", 320, 240);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if(j.getRawButton(5)){
					cam0 = true;
				}
				if(j.getRawButton(6)){
					cam0 = false;
				}
				if(cam0){
					if (cvSink0.grabFrame(mat) == 0) {
						// Send the output the error.
						outputStream.notifyError(cvSink0.getError());
						// skip the rest of the current iteration
						continue;
					}
				}
				else{
					if (cvSink1.grabFrame(mat) == 0) {
						// Send the output the error.
						outputStream.notifyError(cvSink1.getError());
						// skip the rest of the current iteration
						continue;
					}
				}
				// Put a rectangle on the image
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
	    visionThread.start();

    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	autoSelected = (String) chooser.getSelected();
//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		grabClaw.set(DoubleSolenoid.Value.kForward);//close the claw
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic(){
    	switch(autoSelected) {
    	case center://center
    		if(autonomousStep==0){//drive towards peg until at specified distance
    			if(frontRangefinder.getAverageValue()*1024 > 300/*Specified distance*/){
    				autonMotorSpeed = rampUp(autonMotorSpeed,.01,.2);
	    			drive.arcadeDrive(autonMotorSpeed, (turnAverage * 0.005));
	    			double centerX;
	    			synchronized (imgLock) {
	    				centerX = this.centerX;
	    			}
	    			double turn = centerX - (IMG_WIDTH / 2);
	    			turn*=3;
	    			if(turn>120){
	    				turn = 120;
	    			}
	    			else if(turn<-120){
	    				turn = -120;
	    			}
	    			for(int i = prevTurnValues.length-1; i>0; i--){
	    				prevTurnValues[i] = prevTurnValues[i-1];
	    			}
	    			prevTurnValues[0] = turn;
	    			valsUsed = 0;
	    			turnAverage = 0;
	    			for(int i = 0; i<prevTurnValues.length; i++){
	    				if(prevTurnValues[i]!=0){
	    					turnAverage+=prevTurnValues[i];
	    					valsUsed++;
	    				}
	    			}
	    			turnAverage = turnAverage/valsUsed;
	    			if(valsUsed == 0){
	    				turnAverage = 0;
	    			}
    			}else{
    				drive.arcadeDrive(0,0);
    				autonomousStep++;
    			}
    		}else if (autonomousStep == 1){//open claw
    			grabClaw.set(DoubleSolenoid.Value.kReverse);
    			autonomousStep++;
    		}
            break;
    	case left://left
    		if(autonomousStep==0){//Initialize stuff like gyro
    			gyro.reset();
    			liftClaw.set(true);
    			autonomousStep++;
    		}else if(autonomousStep == 1){//drive forward until at the right distance
			    if(backRangefinder.getAverageValue()*1024 < 0/*put real number here*/){
	    			autonMotorSpeed = rampUp(autonMotorSpeed, .03, THREE_QUARTERS_SPEED);
			        drive.arcadeDrive(autonMotorSpeed, gyro.getAngle()*-0.03);
		        }else{
		        	autonomousStep++;
		        }
    		}else if(autonomousStep==2){//stop driving until stopped
    			if(autonMotorSpeed !=0){
	    			autonMotorSpeed = rampDown(autonMotorSpeed, .05, 0);
			        drive.arcadeDrive(autonMotorSpeed, gyro.getAngle()*-0.03);
    			}else{
    				autonomousStep++;
    				gyro.reset();
    			}
    		}else if(autonomousStep==3){//rotate towards peg until rotated
    			if(gyro.getAngle()<60){
    				drive.arcadeDrive(0, .2);
    			}else{
    				autonomousStep++;
    				gyro.reset();
    			}
    		}else if(autonomousStep==4){//drive towards peg until at specified distance
    			if(frontRangefinder.getAverageValue()*1024 > 300/*Specified distance*/){
    				autonMotorSpeed = rampUp(autonMotorSpeed,.01,.2);
	    			drive.arcadeDrive(autonMotorSpeed, (turnAverage * 0.005));
	    			double centerX;
	    			synchronized (imgLock) {
	    				centerX = this.centerX;
	    			}
	    			double turn = centerX - (IMG_WIDTH / 2);
	    			turn*=3;
	    			if(turn>120){
	    				turn = 120;
	    			}
	    			else if(turn<-120){
	    				turn = -120;
	    			}
	    			for(int i = prevTurnValues.length-1; i>0; i--){
	    				prevTurnValues[i] = prevTurnValues[i-1];
	    			}
	    			prevTurnValues[0] = turn;
	    			valsUsed = 0;
	    			turnAverage = 0;
	    			for(int i = 0; i<prevTurnValues.length; i++){
	    				if(prevTurnValues[i]!=0){
	    					turnAverage+=prevTurnValues[i];
	    					valsUsed++;
	    				}
	    			}
	    			turnAverage = turnAverage/valsUsed;
	    			if(valsUsed == 0){
	    				turnAverage = 0;
	    			}
    			}else{
    				drive.arcadeDrive(0,0);
    				autonomousStep++;
    			}
    		}else if (autonomousStep == 5){//open claw
    			grabClaw.set(DoubleSolenoid.Value.kReverse);
    			autonomousStep++;
    		}
            break;
    	case right:
    	default:
    		if(autonomousStep==0){//Initialize stuff like gyro
    			gyro.reset();
    			liftClaw.set(true);
    			autonomousStep++;
    		}else if(autonomousStep == 1){//drive forward until at the right distance
			    if(backRangefinder.getAverageValue()*1024 < 0/*put real number here*/){
	    			autonMotorSpeed = rampUp(autonMotorSpeed, .03, THREE_QUARTERS_SPEED);
			        drive.arcadeDrive(autonMotorSpeed, gyro.getAngle()*-0.03);
		        }else{
		        	autonomousStep++;
		        }
    		}else if(autonomousStep==2){//stop driving until stopped
    			if(autonMotorSpeed !=0){
	    			autonMotorSpeed = rampDown(autonMotorSpeed, .05, 0);
			        drive.arcadeDrive(autonMotorSpeed, gyro.getAngle()*-0.03);
    			}else{
    				autonomousStep++;
    				gyro.reset();
    			}
    		}else if(autonomousStep==3){//rotate towards peg until rotated
    			if(gyro.getAngle()>-60){
    				drive.arcadeDrive(0, -.2);
    			}else{
    				autonomousStep++;
    				gyro.reset();
    			}
    		}else if(autonomousStep==4){//drive towards peg until at specified distance
    			if(frontRangefinder.getAverageValue()*1024 > 300/*Specified distance*/){
    				autonMotorSpeed = rampUp(autonMotorSpeed,.01,.2);
	    			drive.arcadeDrive(autonMotorSpeed, (turnAverage * 0.005));
	    			double centerX;
	    			synchronized (imgLock) {
	    				centerX = this.centerX;
	    			}
	    			double turn = centerX - (IMG_WIDTH / 2);
	    			turn*=3;
	    			if(turn>120){
	    				turn = 120;
	    			}
	    			else if(turn<-120){
	    				turn = -120;
	    			}
	    			for(int i = prevTurnValues.length-1; i>0; i--){
	    				prevTurnValues[i] = prevTurnValues[i-1];
	    			}
	    			prevTurnValues[0] = turn;
	    			valsUsed = 0;
	    			turnAverage = 0;
	    			for(int i = 0; i<prevTurnValues.length; i++){
	    				if(prevTurnValues[i]!=0){
	    					turnAverage+=prevTurnValues[i];
	    					valsUsed++;
	    				}
	    			}
	    			turnAverage = turnAverage/valsUsed;
	    			if(valsUsed == 0){
	    				turnAverage = 0;
	    			}
    			}else{
    				drive.arcadeDrive(0,0);
    				autonomousStep++;
    			}
    		}else if (autonomousStep == 5){//open claw
    			grabClaw.set(DoubleSolenoid.Value.kReverse);
    			autonomousStep++;
    		}
            break;
    	}
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	if(firstTime){//if this is the first time running the teleopPeriodic loop
    		grabClaw.set(DoubleSolenoid.Value.kForward);
    		liftClaw.set(true);
    		firstTime = false;
    	}
    	//set the button states and throttle value
    	for(int i = 0; i < buttonState.length; i++){
    		buttonState[i] = j.getRawButton(i+1);
    	}
    	clawButtonState = clawButton.get();
    	throttleValue = (j.getThrottle() - 1)/2;
    	
    	//if button #3 is HELD, run the vision processing, otherwise drive
		if(buttonState[2]){
			drive.arcadeDrive(0, (turnAverage * 0.005));
			double centerX;
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			double turn = centerX - (IMG_WIDTH / 2);
			turn*=3;
			if(turn>120){
				turn = 120;
			}
			else if(turn<-120){
				turn = -120;
			}
			for(int i = prevTurnValues.length-1; i>0; i--){
				prevTurnValues[i] = prevTurnValues[i-1];
			}
			prevTurnValues[0] = turn;
			valsUsed = 0;
			turnAverage = 0;
			for(int i = 0; i<prevTurnValues.length; i++){
				if(prevTurnValues[i]!=0){
					turnAverage+=prevTurnValues[i];
					valsUsed++;
				}
			}
			turnAverage = turnAverage/valsUsed;
			if(valsUsed == 0){
				turnAverage = 0;
			}
			SmartDashboard.putNumber("Turn Last Used Value", prevTurnValues[9]);
			SmartDashboard.putNumber("Turn Value", turn);
			SmartDashboard.putNumber("Turn Average Value", turnAverage);
		}else{
			drive.mecanumDrive_Cartesian(j.getX(), j.getY(), j.getTwist(),0);
		}
		
		//do pneumatics
		if(clawButtonState&&!prevClawButtonState){
			grabClaw.set(DoubleSolenoid.Value.kForward);
			timer.start();
		}if(timer.get()>=.15){//the second number is in seconds
			liftClaw.set(true);
			timer.stop();
			timer.reset();
		}
    	if(buttonState[0]&&!prevButtonState[0]){
    		if(grabClaw.get().equals(DoubleSolenoid.Value.kForward)){
    			grabClaw.set(DoubleSolenoid.Value.kReverse);
    		}else{
    			grabClaw.set(DoubleSolenoid.Value.kForward);
    		}
    	}
    	if(buttonState[1]&&!prevButtonState[1]){
    		if(liftClaw.get()){
    			liftClaw.set(false);
    		}else{
    			liftClaw.set(true);
    		}
    	}
    	
    	//set winches to forward or backwards depending on the button
    	if(buttonState[10]){
    		CANTalonList[4].set(throttleValue);
    		CANTalonList[5].set(-throttleValue);
    	}else if(buttonState[11]){
    		CANTalonList[4].set(-throttleValue);
    		CANTalonList[5].set(throttleValue);
    	}else{
    		CANTalonList[4].set(0);
    		CANTalonList[5].set(0);
    	}
    	
        
        //previous button state thingy
        for(int i = 0; i < prevButtonState.length; i++){
    		prevButtonState[i] = j.getRawButton(i+1);
    	}
        prevClawButtonState = clawButton.get();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
}
