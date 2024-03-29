// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
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
  private RelativeEncoder frontLeftEncoder = frontLeftMotor.getEncoder();
  private RelativeEncoder frontRightEncoder = frontRightMotor.getEncoder();
  private RelativeEncoder rearLeftEncoder = rearLeftMotor.getEncoder();
  private RelativeEncoder rearRightEncoder = rearRightMotor.getEncoder();

  // Variable for CANEncoder
  // According to Neo Specification
  // (https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf)
  // Hall-Sensor Encoder Resolution: 42 counts (ticks) per rev
  // unit conversion from tick to feet
  // 42 ticks = (π * wheelDiameterInches) * (ft / 12 Inches) * GearRatio
  // 42 ticks = (π * 6 Inches) * (ft / 12 Inches) * GearRatio
  // ticks = 1/42 * (6 π / 12) * GearRatio
  //private final double kDriveTick2Feet = 1.0 / 42 * 6 * Math.PI / 12 * 4.16 * 141;

  //https://docs.revrobotics.com/rev-hardware-client/spark-max/navigating-the-client-spark-max
  // 4096 ticks = (π * wheelDiameterInches) * (ft / 12 Inches) * GearRatio
  // ticks = 1/4096 * (π * 6) /12
  //private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
  private final double kDriveTick2Feet = 0.155;

  // Arm
  private CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  private RelativeEncoder armEncoder = arm.getEncoder();
  //private final double kArmTick2Deg = 360.0 / 42 / 6.5;
  private final double kArmTick2Deg = 0.9868; //1.3186;

  // Climber
  private CANSparkMax climber = new CANSparkMax(6, MotorType.kBrushless);
  private RelativeEncoder climberEncoder = climber.getEncoder();

  // Intake
   private CANSparkMax intake = new CANSparkMax(7,MotorType.kBrushless);

  // Variables for Intake (Redline 775 Motor w Encoder)
  //Ref:https://motors.vex.com/vexpro-motors/775pro
  //1 RPM = 0.10472 Rad/s
  //Free Speed = 18700 RPM * 0.10472 Rad/sec / RPM = 1958.264 Rad/sec

  // variables to control the drivetrain
  private final double kDeadband = 0.05;   //motors will stop (to save the motor from burning) if the abs(joystick input) is less than the deadBand
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

  //autonomous choices
  private static final String kDefaultAuto = "AUTO_SHOOT";
  private static final String kCustomAuto1 = "DRIVE_DISTANCE_LIMIT";
  private static final String kCustomAuto2 = "DRIVE_TIME_LIMIT";    
  private static final String kCustomAuto3 = "DRIVE_TIME_TARGET";    
  private String m_AutoleSelected;
  private final SendableChooser<String> m_AutoChooser = new SendableChooser<>();                   //declare a radio button object to select the desired autonomous 

  private boolean driveDistanceFinished = false;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //enableMotors(true);                             //set Brake mode
    setInvertMotors();                              //setInverted motors

    //reset encoders to zero position
    resetEncoders();
/*
    //set forward and reverse boundary limits for the arm 
    arm.setSoftLimit(SoftLimitDirection.kForward, (float) (160.0 / kArmTick2Deg));
    arm.setSoftLimit(SoftLimitDirection.kReverse, (float) (0.0 / kArmTick2Deg));
    //enable the arm softLimits
    arm.enableSoftLimit(SoftLimitDirection.kForward, true);
    arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
/*
    //set forward and reverse boundary limits for the climber 
    climber.setSoftLimit(SoftLimitDirection.kForward, (float) (60.5 / kArmTick2Deg));
    climber.setSoftLimit(SoftLimitDirection.kReverse, (float) (0.0 / kArmTick2Deg));
    //enable the arm softLimits
    climber.enableSoftLimit(SoftLimitDirection.kForward, true);
    climber.enableSoftLimit(SoftLimitDirection.kReverse, true);
*/

    //set the conversion factor to each encoder
/*
    frontLeftEncoder.setPositionConversionFactor(kDriveTick2Feet);
    rearLeftEncoder.setPositionConversionFactor(kDriveTick2Feet);
    frontRightEncoder.setPositionConversionFactor(kDriveTick2Feet);
    rearRightEncoder.setPositionConversionFactor(kDriveTick2Feet);
*/
    //display autonomous choices on Dashboard
    m_AutoChooser.setDefaultOption("AUTO SHOOT", kDefaultAuto);                               //add default choice to radio button object
    m_AutoChooser.addOption("DRIVE TIME LIMIT", kCustomAuto1);                                //add another choice for autonomous mode
    m_AutoChooser.addOption("DRIVE DISTANCE LIMIT", kCustomAuto2);                            //add another choice for autonomous mode
    m_AutoChooser.addOption("DRIVE TO TARGET", kCustomAuto3);                                 //add another choice for autonomous mode
    SmartDashboard.putData("Autonomous Mode", m_AutoChooser);                                 //display the choices on SmartDashboard
  }

  @Override
  public void robotPeriodic() {
    log();
  }

  @Override
  public void autonomousInit() {
    resetEncoders();

    enableMotors(true);                             //set Brake mode
    startTime = Timer.getFPGATimestamp();	              //time when autonomous mode starts
    m_AutoleSelected = m_AutoChooser.getSelected();		//get the autonomous choice
    SmartDashboard.putString("m_AutoleSelected", m_AutoleSelected);
  }

  @Override
  public void autonomousPeriodic() {

    switch (m_AutoleSelected){
      case "AUTO_SHOOT":
        autoShoot();
        break;
      case "DRIVE_DISTANCE_LIMIT":
        driveDistance(kAutoDistanceFt, kTestSpeed);
        break;
      case "DRIVE_TIME_LIMIT":
        driveTime(kAutoTiming, kTestSpeed);
        break;
      case "DRIVE_TO_TARGET":
        driveToTarget();
        break;
    }
 
  }

  @Override
  public void teleopInit() {
    enableMotors(true);
  }

  @Override
  public void teleopPeriodic() {
    //log();

    //Driving
    double leftSpeed = -joyLeft.getY() * kMaxDriveSpeed;  //negate because pushing joystick forward always return negative
    double rightSpeed = -joyRight.getY() * kMaxDriveSpeed; 
    //apply deadband
    if (Math.abs(leftSpeed) < kDeadband) leftSpeed = 0.0;
    if (Math.abs(rightSpeed) < kDeadband) rightSpeed = 0.0;

    if (joyLeft.getRawButton(1)){         //we use button 1 to tell the robot to drive to target with limelight assistance
        driveToTarget();
    } else{
        tankDrive(leftSpeed, rightSpeed);
    }

    // Intake
    double intakePower = 0.0;
    if (gamePad.getRawButton(1)) intakePower = -1.0;
    else if (gamePad.getRawButton(2)) intakePower = 1.0;
    intake.set(intakePower);

    // Arm
    //control arm using a button
    double armPower = 0.0;
    if (gamePad.getRawButton(4)) armPower = 0.6;
    else if (gamePad.getRawButton(3)) armPower = -0.6;
/*
    else{
      //control arm using joystick
      //REMEMBER to press MODE button on gamePad to make it works like a Joystick, Not as a POV (default)
      armPower = - gamePad.getRawAxis(1);
      //apply deadband
      if (Math.abs(armPower) < kDeadband) armPower = 0.0;
      armPower *= 0.6;  //60% of full power
    }
*/
    arm.set(armPower);

    // climber
    double climbPower = 0.0;
    if (gamePad.getRawButton(6)) climbPower = 1.0;
    else if(gamePad.getRawButton(8)) climbPower = -1.0;
    climber.set(climbPower);

  }

  @Override
  public void disabledInit() {
    enableMotors(false);
  }

  @Override
  public void disabledPeriodic() {
    enableMotors(false);
  }

  @Override
  public void testInit() {
    enableMotors(true);                   //set Brake mode
  }

  @Override
  public void testPeriodic() {
    log();
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
      driveDistance(0.1, 0.1);           //   drive for 5ft
    else if (gamePad.getRawButton(5))        //press gamPad button 5
      setArmMotorUp();                          //arm up
    else if(joyRight.getRawButton(1))
      driveDistance(4.0, -1.0);
    else if(joyRight.getRawButton(2))
      autoShoot();
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

    arm.setIdleMode(mode);
    climber.setIdleMode(mode);
  }

  public void resetEncoders(){
    //reset drivetrain encoders to zero position
    frontLeftEncoder.setPosition(0.0);
    frontRightEncoder.setPosition(0.0);
    rearLeftEncoder.setPosition(0.0);
    rearRightEncoder.setPosition(0.0);

    //reset arm encoder to zero position
    armEncoder.setPosition(0.0);
    //reset climber encoder to zero position
    climberEncoder.setPosition(0.0);

  }

  public void setInvertMotors(){
    frontRightMotor.setInverted(true);       //invert the right side
    rearRightMotor.setInverted(true);        //invert the right side 
    frontLeftMotor.setInverted(false);
    rearLeftMotor.setInverted(false);
  }

  public void arcadeDrive(double drivePower, double steerPower){
    //apply deadBand
    if (Math.abs(drivePower) <= kDeadband) drivePower = 0.0;
    if (Math.abs(steerPower) <= kDeadband) steerPower = 0.0;

    frontLeftMotor.set( drivePower + steerPower);
    rearLeftMotor.set(  drivePower + steerPower);
    frontRightMotor.set(-(drivePower - steerPower));
    rearRightMotor.set( -(drivePower - steerPower));

  }

  public void tankDrive(double leftPower, double rightPower){
    //apply deadBand
    if (Math.abs(leftPower) <= kDeadband) leftPower = 0.0;
    if (Math.abs(rightPower) <= kDeadband) rightPower= 0.0;

    frontLeftMotor.set( leftPower);
    rearLeftMotor.set(  leftPower);
    frontRightMotor.set(rightPower);
    rearRightMotor.set( rightPower);
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

    //measure the distance (in ft) how far the drive goes
    SmartDashboard.putNumber("frontLeft Encoder Value", frontLeftEncoder.getPosition() * kDriveTick2Feet );
    SmartDashboard.putNumber("frontRight Encoder Value", frontRightEncoder.getPosition() * kDriveTick2Feet );
    SmartDashboard.putNumber("rearLeft Encoder Value", rearLeftEncoder.getPosition() * kDriveTick2Feet );
    SmartDashboard.putNumber("rearRight Encoder Value", rearRightEncoder.getPosition() * kDriveTick2Feet);

    SmartDashboard.putNumber("Arm Encode Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Angle", armEncoder.getPosition() * kArmTick2Deg);

    SmartDashboard.putNumber("Climber Encode Position", climberEncoder.getPosition());
    SmartDashboard.putNumber("Climber Angle", climberEncoder.getPosition() * kArmTick2Deg);

  }

  public void update_Limelight_Tracking(){
    //Ref: https://docs.limelightvision.io/en/latest/cs_drive_to_goal_2019.html
    //
    //limelight return the NetworkTable containing the following info:
    //tx: Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    //ty: Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    //tv: Target valid - Whether the limelight has any valid targets (0 = not found or 1 = found)
    //ta: Target Area (0% to 100% of image)

    double heading_error = tx;                                        //horizontal offset between camera & target
    double distance_error = ty;                                       //vertical offset between camera & target
    double steering_adjust = 0.0;

    final double KpAim = 0.1;                                         // how hard to turn toward the target
    double kpDistance = 0.1;                                          // how hard to drive toward the target
    double min_aim_command = 0.01;
    limelightSteerCommand = 0.0;
    limelightDriveCommand = 0.0;

    if (tv == 0){                                                     //if target is not found
      limelightHasValidTarget = false;
      limelightDriveCommand = 0.0;
      limelightSteerCommand = 0.0;

      if (startTime == 0.0) startTime = Timer.getFPGATimestamp();
      double time = Timer.getFPGATimestamp();

      if(time - startTime < 1.0){                                    //spin for 1 sec
        steering_adjust = 0.2;                                       //seek for the target by spinning in place at a safe speed.
        tankDrive(steering_adjust, -steering_adjust);
      }
      else{
        stop();
      }

    }
    else{
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
      distance_error = estimate_distance();

      double distance_adjust = kpDistance * distance_error;
      limelightDriveCommand = distance_adjust;
    }
  }

  public double estimate_distance(){
    /*
    https://docs.limelightvision.io/en/latest/cs_estimating_distance.html

    d = (h2 - h1) / tan(a1 + a2)

    */

    double targetOffsetAngle_Vertical = ty;   // (a2)

    // how many degrees back is your limelight rotated from perfectly vertical? (a1)
    double limelightMountAngleDegrees = 0.0;

    // distance (in) from the center of the Limelight lens to the floor (h1)
    double limelightHeightInches = 3.0 * 12.0;

    // distance from the target to the floor (h2)
    double goalHeightInches = 0.0;

    // (a1 + a2)
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    // d = (h2 - h1) / tan (a1 + a2)
    double distanceFromLimelightToGoalInches = Math.abs(goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }

  public double getCurrentPos(){
    double position =
               (frontLeftEncoder.getPosition() +
                frontRightEncoder.getPosition() ) /2;

    position *= kDriveTick2Feet;

    return position;
  }

  public void driveToTarget(){
    // With limelight assistance, drive the robot to target using arcade drive mode

    update_Limelight_Tracking();                                    //get the limelightDriveCommand and limelightSteerCommand

    if (limelightHasValidTarget){                                   //if found the target
      tankDrive(limelightDriveCommand, -limelightSteerCommand);      //drive to that target
    }
  }

  public void driveDistance(double feet, double speed){
    double distance = Math.abs(getCurrentPos());

    if (distance < feet){
      tankDrive(speed, speed);
    }
    else{
      stop();
      driveDistanceFinished = true;
    }
  }

  public void driveTime(double timeLimit, double speed){
    if (startTime == 0.0) startTime = Timer.getFPGATimestamp();

    double time = Timer.getFPGATimestamp();
    if (time - startTime < timeLimit){
      tankDrive(speed, speed);
    }
    else{
      stop();
    }
  }

  //drive Straight with the speed provided by joysticks
  public void driveStraight(){
    double joystickInput1, joystickInput2;

    joystickInput1 = - joyLeft.getY()  * kMaxDriveSpeed;
    joystickInput2 = - joyRight.getY() * kMaxDriveSpeed;

    //to drive straight, speed in both sides should be the same,
    //drive straight with max speed of both sides
    if (Math.abs(joystickInput1) > Math.abs(joystickInput2)) joystickInput2 = joystickInput1;
    else joystickInput1 = joystickInput2;

    tankDrive(joystickInput1, joystickInput2);
  }

  public void stop(){
    frontLeftMotor.set(0.0);
    rearLeftMotor.set(0.0);
    frontRightMotor.set(0.0);
    rearRightMotor.set(0.0);
  }

  public void setArmMotorUp(){
    arm.set(0.6);
    //armEncoder.setPosition(293.0); // needs to be double number for up position
  }

  public void setArmMotorDown(){
    arm.set(-0.6);
  }

  public void setClimberMotorUp(){
    climber.set(1.0);
  }

  public void setClimberMotorDown(){
    climber.set(-1.0);
  }

  public void setIntakeIn(){
    intake.set(-1.0);
  }
/*
  public void setIntakeIn(double time){
    double timeStart = Timer.getFPGATimestamp();

    while (Timer.getFPGATimestamp() - timeStart < time){
      intake.set(1.0);
    }
    intake.set(0);
  }
*/

  public void setIntakeOut(){
      intake.set(1.0);
      //intake out
  }
  
  public void autoShoot(){
    double time = Timer.getFPGATimestamp();
     // it will shoot the ball for 2 seconds
     if (time - startTime < 2){
      setIntakeOut();
    }
    else{
      intake.set(0);
      //move backward straight for 8ft
      driveDistance(8.0, -0.2);
    }   

    //if (driveDistanceFinished) driveToTarget();

  }

  public double getArmAngle(){
    double angle = 0.0;

    angle = armEncoder.getPosition() * kArmTick2Deg;
    return angle;
  }
  public double getClimberAngle(){
    double angle = 0.0;
    angle = climberEncoder.getPosition() * kArmTick2Deg;
    return angle;
  }
  
}// end of robot.java
