// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Joysticks
  private Joystick joyLeft = new Joystick(0);
  private Joystick joyRight = new Joystick(1);
  private Joystick gamePad = new Joystick(2);

  // motor controllers for non-mecanum drivetrain
  private CANSparkMax frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rearLeftMotor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax rearRightMotor = new CANSparkMax(4, MotorType.kBrushless);

  // encoders
  private CANEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
  private CANEncoder frontRightEncoder = frontRightMotor.getEncoder();
  private CANEncoder rearLeftEncoder = rearLeftMotor.getEncoder();
  private CANEncoder rearRightEncoder = rearRightMotor.getEncoder();
  // Variable for CANEncoder
  // According to Neo Specification
  // (https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf)
  // Hall-Sensor Encoder Resolution: 42 counts (ticks) per rev
  // unit conversion from tick to feet
  // 42 ticks = (π * wheelDiameterInches) * (ft / 12 Inches) * GearRatio
  // 42 ticks = (π * 6 Inches) * (ft / 12 Inches) * GearRatio
  // ticks = 1/42 * (6 π / 12) * GearRatio
  private final double kDriveTick2Feet = 1.0 / 42 * 6 * Math.PI / 12 * 4.16;
  // 42 ticks = (π * wheelDiameterInches) * (ft / 12 Inches) * GearRatio  
  private final double kArmTick2Deg = 360.0 / 42 / 6.5;

  // Arm
  private CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  private CANEncoder armEncoder = arm.getEncoder();

  // Climber
  private CANSparkMax climber = new CANSparkMax(6, MotorType.kBrushless);
  // Intake
  private CANSparkMax intake = new CANSparkMax(7,MotorType.kBrushed);

  // Variables for Intake (Redline 775 Motor w Encoder)
  //Ref:https://motors.vex.com/vexpro-motors/775pro
  //1 RPM = 0.10472 Rad/s
  //Free Speed = 18700 RPM * 0.10472 Rad/sec / RPM = 1958.264 Rad/sec
  // variables to control the drivetrain
  private final double kDeadband = 0.02;   //motors will stop (to save the motor from burning) if the abs(joystick input) is less than the deadBand
  private double kTestSpeed = 0.3;         //limit the test speed
  private double kMaxDriveSpeed =  0.75;   //limit the drive speed
  private double kAutoTiming = 3.0;        //autonomous timing drive in sec
  private double kAutoDistanceFt = 5.0;	   //autonomous distance in feet
  private double startTime;                //time when autonomous mode starts

  //variables controlling limelight camera
  private boolean limelightHasValidTarget = false;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private double tx, ty, tv, ta;
  private double limelightDriveCommand, limelightSteerCommand;

  //for debugging purpose
  private boolean debug = false;

  //autonomous choices
  private static final String kDefaultAuto = "AUTO_SHOOT";
  private static final String kCustomAuto1 = "DRIVE_DISTANCE_LIMIT";
  private static final String kCustomAuto2 = "DRIVE_TIME_LIMIT";    
  private String m_AutoleSelected;
  private final SendableChooser<String> m_AutoChooser = new SendableChooser<>();                   //declare a radio button object to select the desired autonomous 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    enableMotors(true);                             //set Brake mode
    setInvertMotors();                              //setInverted motors

    //reset encoders to zero position
    resetEncoders();

    //set the conversion factor to each encoder
    frontLeftEncoder.setPositionConversionFactor(kDriveTick2Feet);
    rearLeftEncoder.setPositionConversionFactor(kDriveTick2Feet);
    frontRightEncoder.setPositionConversionFactor(kDriveTick2Feet);
    rearRightEncoder.setPositionConversionFactor(kDriveTick2Feet);

    //display autonomous choices on Dashboard
    m_AutoChooser.setDefaultOption("AUTO SHOOT", kDefaultAuto);                               //add default choice to radio button object
    m_AutoChooser.addOption("DRIVE_TIME_LIMIT", kCustomAuto1);                                     //add another choice for autonomous mode
    m_AutoChooser.addOption("DRIVE_DISTANCE_LIMIT", kCustomAuto2);                                 //add another choice for autonomous mode
    SmartDashboard.putData("Autonomous Mode", m_AutoChooser);                                      //display the choices on SmartDashboard


  }

  @Override
  public void robotPeriodic() {
    log();

    SmartDashboard.putNumber("joyLeft Y", joyLeft.getY());
    SmartDashboard.putNumber("joyRight Y", joyRight.getY());

    //measure the distance (in ft) how far the drive goes
    SmartDashboard.putNumber("Left Drive Encoder Value", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Left Drive Encoder Value", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Left Drive Encoder Value", rearLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right Drive Encoder Value", rearRightEncoder.getPosition());
  }

  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();	              //time when autonomous mode starts
    m_AutoleSelected = m_AutoChooser.getSelected();		//get the autonomous choice
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_AutoleSelected){
      case "AUTO SHOOT":
        autoShoot();
        break;
      case "DRIVE_DISTANCE_LIMIT":
        double distance = getCurrentPos();

        //drive at 60% of power until kAutoDistanceFt ft
        if (distance < kAutoDistanceFt) {
          tankDrive(kTestSpeed, kTestSpeed);
        } 
        else {
          stop();
        }
        break;
      case "DRIVE_TIME_LIMIT":
        double time = Timer.getFPGATimestamp();             //get the current time

        if (time - startTime < kAutoTiming) {           //if we are still in kAutoTiming sec.
          frontLeftMotor.set(kTestSpeed);               //drive forward at kTestSpeed
          rearLeftMotor.set(kTestSpeed);
          frontRightMotor.set(kTestSpeed);
          rearRightMotor.set(kTestSpeed);
        } 
        else {                                          //stop when time is running out
          stop();
        }
        break;
      case "DRIVE TO TARGET":
        driveToTarget();
        break;
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
      
    // Intake
    if(gamePad.getRawButton(1)){
      setIntakeIn();
    }
    else if(gamePad.getRawButton(2)){
      setIntakeOut();
    }

    // Arm
    else if(gamePad.getRawButton(4)){
      setArmMotorUp();
    }
    else if(gamePad.getRawButton(3)){
      setArmMotorDown();
    }
    
    // climb
    else if(gamePad.getRawButton(6)){
      setClimberMotorUp();
    }
    else if(gamePad.getRawButton(8)){
      setClimberMotorDown();
    }
    else{
      intake.set(0);
      climber.set(0);
      arm.set(0);
    }

    double leftSpeed = -joyLeft.getY() * kMaxDriveSpeed;  //negate because pushing joystick forward always return negative
    double rightSpeed = -joyRight.getY() * kMaxDriveSpeed; 

    if (joyLeft.getRawButton(1)){         //we use button 1 to tell the robot to drive to target with limelight assistance
        driveToTarget();
    } else{
        tankDrive(leftSpeed, rightSpeed);
    }

  }

  @Override
  public void disabledInit() {
    enableMotors(false);
  }

  @Override
  public void disabledPeriodic() {
    //enableMotors(false);
  }

  @Override
  public void testInit() {
    enableMotors(true);                   //set Brake mode
  }

  @Override
  public void testPeriodic() {
    // Forward polarity test
    if (joyLeft.getRawButton(5))	        //press and hold button 5 of joystick1
      frontLeftMotor.set(kTestSpeed);		        //  frontLeftMotor wheel should turn forward. If not, change its inverse value
    else if (joyLeft.getRawButton(3))	    //press and hold button 3 of joystick1
      rearLeftMotor.set(kTestSpeed);		        //  rearLeftMotor wheel should turn forward. If not, change its inverse value
    else if (joyLeft.getRawButton(6))	    //press and hold button 6 of joystick1
      frontRightMotor.set(-kTestSpeed);	        //  frontRightMotor wheel should turn backward. If not, change its inverse value
    else if (joyLeft.getRawButton(4))	    //press and hold button 4 of joystick1
      rearRightMotor.set(-kTestSpeed);	        //  rearRightMotor wheel should turn backward. If not, change its inverse value
    else if (joyLeft.getRawButton(1))     //press button 1
      driveToTarget();                          //  drive to target
    else if (joyLeft.getRawButton(2))     //press button 2
      driveStraight();                    //and push joyLeft forward/backward to drive forward/backward in a straight line
    else if (joyLeft.getRawButton(7))     //press button 7
      driveDistance(5.0, kTestSpeed);           //   drive for 5ft
    else if (gamePad.getRawButton(5))        //press gamPad button 5
      setArmMotorUp();                          //arm up
    else stop();			                    //if no button pressed, all motors stop
      
 }

  private void enableMotors(boolean on) {
    CANSparkMax.IdleMode mode;

    if (on) mode = CANSparkMax.IdleMode.kBrake;
    else mode = CANSparkMax.IdleMode.kCoast;

    frontLeftMotor.setIdleMode(mode);
    rearLeftMotor.setIdleMode(mode);
    frontRightMotor.setIdleMode(mode);
    rearRightMotor.setIdleMode(mode);
  }

  public void resetEncoders(){
    //reset encoders to zero position
    frontLeftEncoder.setPosition(0.0);
    frontRightEncoder.setPosition(0.0);
    rearLeftEncoder.setPosition(0.0);
    rearRightEncoder.setPosition(0.0);
  }

  public void setInvertMotors(){
    frontRightMotor.setInverted(true);       //invert the right side
    rearRightMotor.setInverted(true);        //invert the right side 
    frontLeftMotor.setInverted(false);
    rearLeftMotor.setInverted(false);
  }

  public void arcadeDrive(double drivePower, double steerPower){
    if (debug) System.out.println("In arcadeDrive(drive, steer)");
    //apply deadBand
    if (Math.abs(drivePower) <= kDeadband) drivePower = 0.0;
    if (Math.abs(steerPower) <= kDeadband) steerPower = 0.0;

    frontLeftMotor.set( drivePower + steerPower);
    rearLeftMotor.set(  drivePower + steerPower);
    frontRightMotor.set(-(drivePower - steerPower));
    rearRightMotor.set( -(drivePower - steerPower));

    if(debug) System.out.println("frontLeftMotor speed: " + (drivePower + steerPower));
    if(debug) System.out.println("rearLeftMotor speed: " + (drivePower + steerPower));
    if(debug) System.out.println("frontRightMotor speed: " + (-drivePower + steerPower));
    if(debug) System.out.println("rearRightMotor speed: " + (-drivePower + steerPower));

    if (debug) System.out.println("End arcadeDrive(drive, steer)");
  }

  public void tankDrive(double leftPower, double rightPower){
    if (debug) System.out.println("In tankDrive(leftPower, rightPower)");
    //apply deadBand
    if (Math.abs(leftPower) <= kDeadband) leftPower = 0.0;
    if (Math.abs(rightPower) <= kDeadband) rightPower= 0.0;

    frontLeftMotor.set( leftPower);
    rearLeftMotor.set(  leftPower);
    frontRightMotor.set(rightPower);
    rearRightMotor.set( rightPower);

    if(debug) System.out.println("frontLeftMotor speed: " + leftPower);
    if(debug) System.out.println("rearLeftMotor speed: " + leftPower);
    if(debug) System.out.println("frontRightMotor speed: " + rightPower);
    if(debug) System.out.println("rearRightMotor speed: " + rightPower);
    
    if (debug) System.out.println("End tankDrive(leftPower, rightPower)");
  }

  public void log(){
    /*
     * Display important values from NetworkTable generated by limelight camera
    ┌───┬─────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
    │tv │Whether the limelight has any valid targets (0 or 1)                                                         │
    │tx │Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)     │
    │ty │Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees) │
    │ta │Target Area (0% of image to 100% of image)                                                                   │
    └───┴─────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
    NOTE: LL1 = Limelight 1, LL2 = Limelight2 model
    */
    tv = table.getEntry("tv").getDouble(0);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("ta", ta);

    SmartDashboard.putNumber("Left Y value", -joyLeft.getY());
    SmartDashboard.putNumber("Right Y value", -joyRight.getY());
  }

  public void update_Limelight_Tracking(){
    //Ref: https://docs.limelightvision.io/en/latest/cs_drive_to_goal_2019.html
    //
    //limelight return the NetworkTable containing the following info:
    //tx: Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    //ty: Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    //tv: Target valid - Whether the limelight has any valid targets (0 = not found or 1 = found)
    //ta: Target Area (0% to 100% of image)

    if (debug) System.out.println("In update_Limelight_Tracking()");

    double heading_error = tx;                                        //horizontal offset between camera & target
    double distance_error = ty;                                       //vertical offset between camera & target
    double steering_adjust = 0.0;

    final double KpAim = 0.1;                                         // how hard to turn toward the target
    double kpDistance = 0.1;                                          // how hard to drive toward the target
    double min_aim_command = 0.01;
    limelightSteerCommand = 0.0;
    limelightDriveCommand = 0.0;

    if (tv == 0){                                                     //if target is not found
      if (debug) System.out.println("Target NOT FOUND");
      limelightHasValidTarget = false;
      limelightDriveCommand = 0.0;
      limelightSteerCommand = 0.0;

      while (tv == 0.0){
        steering_adjust = 0.3;                                       //seek for the target by spinning in place at a safe speed.
        if (debug) System.out.println("Robot spin in place at 30% of full speed");
        tankDrive(steering_adjust, -steering_adjust);
      }
     // return;
    }

    if (debug) System.out.println("Target FOUND");
    limelightHasValidTarget = true;                                   //target found

    //steering
    if (tx > 1.0){                                                    //if target is to the right side of the camera
      steering_adjust = KpAim * heading_error - min_aim_command;      //  steer to the left
    }
    else if (tx < -1.0){                                                            //if target is to the left side of the camera
      steering_adjust = KpAim * heading_error + min_aim_command;      //  steer to the right
    }
    limelightSteerCommand = steering_adjust;

    //driving
    double distance_adjust = kpDistance * distance_error;
    limelightDriveCommand = distance_adjust;

    if (debug) System.out.println("limelightDriveCommand" + limelightDriveCommand);
    if (debug) System.out.println("limelightSteerCommand" + limelightSteerCommand);
    if (debug) System.out.println("End update_Limelight_Tracking()");
  }

  public double getCurrentPos(){
    if(debug) System.out.println("In getCurrentPos()");
    double position =
               (frontLeftEncoder.getPosition() +
                rearLeftEncoder.getPosition() +
                frontRightEncoder.getPosition() +
                rearRightEncoder.getPosition()) / 4;

    if (debug) System.out.println("current position: " + position);
    if(debug) System.out.println("End getCurrentPos()");
    return position;
  }

  public void driveToTarget(){
    if(debug) System.out.println("In driveToTarget");
    // With limelight assistance, drive the robot to target using arcade drive mode

    update_Limelight_Tracking();                                    //get the limelightDriveCommand and limelightSteerCommand

    if (limelightHasValidTarget){                                   //if found the target
      tankDrive(limelightDriveCommand, -limelightSteerCommand);      //drive to that target
    }
    if(debug) System.out.println("End driveToTarget");
  }

  public void driveDistance(double feet, double speed){
    if(debug) System.out.println("In driveDistance");
    resetEncoders();
    double errDistance = feet - getCurrentPos();

    while (errDistance > 0){
      tankDrive(speed, speed);
      errDistance = feet - getCurrentPos();
      System.out.println("errDistance = " + errDistance);
    }
    stop();

    if(debug) System.out.println("End driveDistance");
  }

  //drive Straight with the speed provided by joysticks
  public void driveStraight(){
    double joystickInput1, joystickInput2;

    if(debug) System.out.println("In driveStraight");
    joystickInput1 = - joyLeft.getY()  * kMaxDriveSpeed;
    joystickInput2 = - joyRight.getY() * kMaxDriveSpeed;

    //to drive straight, speed in both sides should be the same,
    //drive straight with max speed of both sides
    if (Math.abs(joystickInput1) > Math.abs(joystickInput2)) joystickInput2 = joystickInput1;
    else joystickInput1 = joystickInput2;

    SmartDashboard.putNumber("Left side speed :", joystickInput1);
    SmartDashboard.putNumber("Right side speed:", joystickInput2);

    tankDrive(joystickInput1, joystickInput2);
    if(debug) System.out.println("End driveStraight");    
  }

  //drive straight with a preset speed
  public void driveStraight(double speed){

    if(debug) System.out.println("In driveStraight(dist)");
    double joystickInput1 = speed;
    double joystickInput2 = speed;

    //to drive straight, speed in both sides should be the same,
    //drive straight with max speed of both sides
    if (Math.abs(joystickInput1) > Math.abs(joystickInput2)) joystickInput2 = joystickInput1;
    else joystickInput1 = joystickInput2;

    SmartDashboard.putNumber("Left side speed :", joystickInput1);
    SmartDashboard.putNumber("Right side speed:", joystickInput2);

    tankDrive(joystickInput1, joystickInput2);
    if(debug) System.out.println("End driveStraight(dist)");    
  }

  //drive straight at a preset speed within preset distance
  public void autoDriveStraight(double speed, double dist){

  }

  public void stop(){
    frontLeftMotor.set(0.0);
    rearLeftMotor.set(0.0);
    frontRightMotor.set(0.0);
    rearRightMotor.set(0.0);
  }

  public void setArmMotorUp(){
    arm.set(0.6);
    SmartDashboard.putNumber("arm angle", getArmAngle());
    //armEncoder.setPosition(293.0); // needs to be double number for up position
  }

  public void setArmMotorDown(){
    arm.set(-0.6);
    SmartDashboard.putNumber("arm angle", getArmAngle());    
  }

  public void setClimberMotorUp(){
    climber.set(1.0);
  }

  public void setClimberMotorDown(){
    climber.set(-1.0);
  }

  public void setIntakeIn(){
    intake.set(1.0);
  }

  public void setIntakeIn(double time){
    double timeStart = Timer.getFPGATimestamp();

    while (Timer.getFPGATimestamp() - timeStart < time){
      intake.set(1.0);
    }
    intake.set(0);
  }

  public void setIntakeOut(){
      intake.set(-1.0);
      //intake out
  }
  
  public void autoShoot(){
    //turn on intake for 5 sec
    setIntakeIn(5.0);

    //move backward straight for 4ft
    driveDistance(4.0, -1.0);

    driveToTarget();
  }

  public double getArmAngle(){
    double angle = 0.0;

    angle = armEncoder.getPosition() * kArmTick2Deg;
    SmartDashboard.putNumber("Arm Enoder possition", armEncoder.getPosition());
    SmartDashboard.putNumber("Arm angle", angle);
    return angle;
  }
}
