// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //create and instantiate 3 joysticks. 
  //joyLeft and joyRight are for driving the robot. 
  //The xbox is for controlling other actuators (graber, arm, climber)
  private Joystick joyLeft = new Joystick(0);
  private Joystick joyRight = new Joystick(1);
//  private XboxController xbox = new XboxController(2);

  //create and instantiate 4 motor controllers
  private  CANSparkMax  frontLeftMotor = new CANSparkMax(0, MotorType.kbrushless);
   private CANSparkMax  rearLeftMotor = new CANSparkMax(1, MotorType.kbrushless);
   private CANSparkMax  frontRightMotor = new CANSparkMax(3, MotorType.kbrushless);
   private CANSparkMax  rearRightMotor = new CANSparkMax(2, MotorType.kbrushless);

  //Neo Encoder is connected to this motorController CANSparkMax called test
  //Because it doesn't connect to any wheel (due to technical difficulty),
  //we can not get the distance the robot travels, all we can do is to test that the motor works
  //private CANSparkMax test = new CANSparkMax(4,MotorType.kBrushless);
  
  private double kDeadband = 0.02;       //motors will stop (to save the motor from burning) if the abs(input) is less than the deadBand

  private double kTestSpeed = 0.3;       //limit the test speed
  private double kMaxSpeed =  0.6;      //limit the drive speed
  //private double startTime;

  private boolean limelightHasValidTarget = false;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private double tx, ty, tv, ta;
  private double limelightDriveCommand, limelightSteerCommand;
  private double driveInput=0;
  private double strafeInput =0;
  private double steerInput = 0;
  private boolean isTankDrive = false;    //if true: joysticks operated in tankDrive mode, false: mecanumDrive mode
  private boolean debug = true;

  private static final String kDefaultJoystickStyle = "TANK_DRIVE";
  private static final String kCustom1JoystickStyle = "ARCADE_DRIVE";
  private static final String kCustom2JoystickStyle = "MECANUM_DRIVE";
  private String m_joystickStyleSelected;
  private final SendableChooser<String> m_joystickStyleChooser = new SendableChooser<>();  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if (debug) System.out.println("In robotInit()");

    //display jostickStyle chooser on Dashboard, also display the default choice
    m_joystickStyleChooser.setDefaultOption("TANK DRIVE", kDefaultJoystickStyle);
    m_joystickStyleChooser.addOption("ARCADE DRIVE", kCustom1JoystickStyle);
    m_joystickStyleChooser.addOption("MECANUM DRIVE", kCustom2JoystickStyle);
    SmartDashboard.putData("Joystick Drive Style", m_joystickStyleChooser);    


    frontLeftMotor.setInverted(false);
    rearLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);	      //invert on the right side
    rearRightMotor.setInverted(true);	        //invert on the right side

    if (debug) System.out.println("End robotInit()");    
  }

  @Override
  public void robotPeriodic() {
    //log();
  }

  @Override
  public void autonomousInit() {
    //startTime = Timer.getFPGATimestamp();	//time when robot starts in autonomous mode
  }

  @Override
  public void autonomousPeriodic() {
    if (debug) System.out.println("In autonomousPeriodic()");
/*    
    double time = Timer.getFPGATimestamp();   //get the current time
    if (time - startTime < 3) {               //if we are still in 3 sec.
      frontLeftMotor.set(kTestSpeed);               //since robot moves forward, each wheel has same speed
      rearLeftMotor.set(kTestSpeed);		             //and we don't have to worry about x and z components
      frontRightMotor.set(kTestSpeed);
      rearRightMotor.set(kTestSpeed);
    } 
    else {                                    //stop when time is running out
      stop();
    }
*/

    if(joyLeft.getRawButton(1)){
      driveToTarget();
    }

    if (debug) System.out.println("End autonomousPeriodic()");
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if (debug) System.out.println("In teleopPeriodic()");

    log();

    //tankDrive
    if (isTankDrive){                       //drive Mecanum drivetrain with TankDrive mode
      
      if (joyLeft.getRawButton(1)){         //we use button 1 to tell the robot to drive to target with limelight assistance
        driveToTarget();
      }
      //if we don't need limelight assistance, just drive normally according to joystick inputs
      else{
        tankDrive(-joyLeft.getY(), -joyRight.getY());

        driveInput = -joyLeft.getY();
        steerInput = -joyRight.getY();
        strafeInput = 0.0;
  
      }
    }

    //drive mecanum drivetrain with mecanumDrive mode
    else{
      //Ref: https://docs.limelightvision.io/en/latest/cs_drive_to_goal_2019.html
      //
      //z-axis threshold
      //  Twisting stickTwo clockwwise and counterclockwise rotate the robot clockwise and counterclockwise respectively
      steerInput = joyRight.getRawAxis(2);

      //y-axis threshold
      driveInput = -joyLeft.getRawAxis(1);	 //pushing stick upward return a negative Y value, so negate it

      //x-axis threshold
      strafeInput = joyRight.getRawAxis(0);
      strafeInput *= 1.5;				             //increase x component to counteract imperfect stafing
    
      if (joyLeft.getRawButton(1)){         //we use button 1 to tell the robot to drive to target with limelight assistance
        driveToTarget();
      }
      //if we don't need limelight assistance, just drive normally according to joystick inputs
      else{
        mecanumDrive(driveInput, strafeInput, steerInput);
      }
    }

    if (debug) System.out.println("End teleopPeriodic()");    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    //if (debug) System.out.println("In testInit");
  }

  @Override
  public void testPeriodic() {
    if (debug) System.out.println("In testPeriodic()");

    // Forward polarity test
    if (joyLeft.getRawButton(5))	        //press and hold button 5 of joystick1
      frontLeftMotor.set(kTestSpeed);		        //  frontLeftMotor wheel should turn forward. If not, change its inverse value
    else if (joyLeft.getRawButton(3))	    //press and hold button 3 of joystick1
      rearLeftMotor.set(kTestSpeed);		        //  rearLeftMotor wheel should turn forward. If not, change its inverse value
    else if (joyLeft.getRawButton(6))	    //press and hold button 6 of joystick1
      frontRightMotor.set(-kTestSpeed);	        //  frontRightMotor wheel should turn backward. If not, change its inverse value
    else if (joyLeft.getRawButton(4))	    //press and hold button 4 of joystick1
      rearRightMotor.set(-kTestSpeed);	        //  rearRightMotor wheel should turn backward. If not, change its inverse value
    //else if (joyRight.getRawButton(12))   //press and hold button 12 of joystick2
      //test.set(-1.0);                     //  test Neo motor should turn backward
    else if (joyLeft.getRawButton(1)){
      tankDrive(-0.213742, -0.213742);
    }
    else if (joyLeft.getRawButton(2)){

      m_joystickStyleSelected = m_joystickStyleChooser.getSelected();
      SmartDashboard.putString("Current Joystick Style", m_joystickStyleSelected);
/*
      switch(m_joystickStyleSelected){
        case "TANK_DRIVE": tankDrive(kTestSpeed, kTestSpeed); break;
        case "ARCADE_DRIVE": arcadeDrive(kTestSpeed, kTestSpeed); break;
        case "MECANUM_DRIVE": mecanumDrive(kTestSpeed, kTestSpeed, 0.0); break;
      }
*/        
    }
    else{				                          //if no button pressed, all motors stop
      stop();
    }

    if (debug) System.out.println("End testPeriodic()");    
  }
  
  public void tankDrive(double leftPower, double rightPower){
    if(debug) System.out.println("In tankDrive(double leftPower, double rightPower)");

    //apply deadBand
    if (Math.abs(leftPower) <= kDeadband) leftPower = 0.0;
    if (Math.abs(rightPower) <= kDeadband) rightPower= 0.0;

    frontLeftMotor.set( leftPower  * kMaxSpeed);
    rearLeftMotor.set(  leftPower  * kMaxSpeed);
    frontRightMotor.set(rightPower * kMaxSpeed);
    rearRightMotor.set( rightPower * kMaxSpeed);

    if(debug) System.out.println("End tankDrive(double leftPower, double rightPower)");

  }
  public void arcadeDrive(double drivePower, double steerPower){
    if (debug) System.out.println("In arcadeDrive(double drivePower, double steerPower)");

    //apply deadBand
    if (Math.abs(drivePower) <= kDeadband) drivePower = 0.0;
    if (Math.abs(steerPower) <= kDeadband) steerPower = 0.0;

    frontLeftMotor.set(( drivePower + steerPower) * kMaxSpeed);
    rearLeftMotor.set((  drivePower + steerPower) * kMaxSpeed);
    frontRightMotor.set(-(drivePower - steerPower) * kMaxSpeed);
    rearRightMotor.set(-(drivePower - steerPower) * kMaxSpeed);

    if (debug) System.out.println("End arcadeDrive(double drivePower, double steerPower)");

  }

  public void mecanumDrive(double y, double x, double z){
    if (debug) System.out.println("In mecanumDrive(double y, double x, double z)");

    //Ref: https://github.com/gamemanual0/gm0/blob/main/source/docs/software/mecanum-drive.rst

    double max = 0.0;
    double frontLeftMotorPower, rearLeftMotorPower, frontRightMotorPower, rearRightMotorPower;

    //if power is too weak, it won't overcome the friction and won't move, so set it to 0 to avoid motor burn
    if (Math.abs(z) <= kDeadband) z = 0.0;
    if (Math.abs(y) <= kDeadband) y = 0.0;
    if (Math.abs(x) <= kDeadband) x = 0.0;

/* Ref: https://github.com/gamemanual0/gm0/blob/main/source/docs/software/images/mecanum-drive/mecanum-drive-directions.png 
  FL=FrontLeft, RL=Rear Left, FR=Front Right, RR=Rear Right
          ↑                                →                                     ↷
    FL ┌─────┐FR                     FL ┌─────┐FR                          FL ┌─────┐FR
  ↑ ┌─┐│     │┌─┐↑                 ↑ ┌─┐│     │┌─┐↓                      ↑ ┌─┐│     │┌─┐↓
    └─┘│     │└─┘                    └─┘│     │└─┘                         └─┘│     │└─┘
  ↑ ┌─┐│     │┌─┐↑                 ↓ ┌─┐│     │┌─┐↑                      ↑ ┌─┐│     │┌─┐↓
    └─┘│     │└─┘                    └─┘│     │└─┘                         └─┘│     │└─┘
    RL └─────┘RR                     RL └─────┘RR                          RL └─────┘RR
    y conponent controls Fwd/Bwd   x component controls Straf R/L    z controls Rotate clockwise/counterclockwise
    Fwd:all wheels forward (+y)    Straf R: LF,RB: +x, RF,LB: -x     clocwise: LF,LB: +z, RF,RB: -z
    FL.set(y+ + )                  FL.set(y +x +)                    FL.set(y +x +z)
    RL.set(y+ + )                  RL.set(y -x + )                   RL.set(y -x +z )
    FR.set(y+ + )                  FR.set(y -x + )                   FR.set(y -x -z )
    RR.set(y+ + )                  RR.set(y +x + )                   RR.set(y +x -z )
*/
    frontLeftMotorPower = y +x +z;
    rearLeftMotorPower  = y -x +z;
    frontRightMotorPower= y -x -z;
    rearRightMotorPower = y +x -z;

    //the power applies to each wheel could be > 1 or <-1
    //If the power is >1, it will be set to 1 by the set() function, if it is < -1, it will be set to -1
    //For ex:              if FL power = 0.4, RL=0.1, FR=1.1, and RR=1.4
    //it will be round off to FL power = 0.4, RL=0.1, FR=1.0, and RR=1.0
    //This round off will cause unstability in the robot
    //Instead, we will devide the power of each wheel with the largest of x,y,z if the abs(power) of any wheel > 1 

    if (Math.abs(frontLeftMotorPower) > 1 || Math.abs(rearLeftMotorPower) > 1 ||
        Math.abs(frontRightMotorPower) > 1 || Math.abs(rearRightMotorPower) > 1){

      //find the largest power
      max = Math.max(Math.abs(frontLeftMotorPower), Math.abs(rearLeftMotorPower));
      max = Math.max(Math.abs(frontRightMotorPower), max);
      max = Math.max(Math.abs(rearRightMotorPower), max);

      //Divide everything by max
      frontLeftMotorPower /= max;
      rearLeftMotorPower /= max;
      frontRightMotorPower /= max;
      rearRightMotorPower /= max;
    }

    //set power to each wheel
    frontLeftMotor.set(frontLeftMotorPower);
    rearLeftMotor.set(rearLeftMotorPower);
    frontRightMotor.set(frontRightMotorPower);
    rearRightMotor.set(rearRightMotorPower);

    if (debug) System.out.println("End mecanumDrive(double y, double x, double z)");

  }

  public void update_Limelight_Tracking(){
    if (debug) System.out.println("In update_Limelight_Tracking()");

    //Ref: https://docs.limelightvision.io/en/latest/cs_drive_to_goal_2019.html
    //
    //limelight return the NetworkTable containing the following info:
    //tx: Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
    //ty: Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
    //tv: Target valid - Whether the limelight has any valid targets (0 = not found or 1 = found)
    //ta: Target Area (0% to 100% of image)

    double heading_error = tx;
    //using ty to drive
    double distance_error = ty;
    double steering_adjust = 0.0;

    double KpAim = 0.1;           //0.03                         // how hard to turn toward the target
    double kpDistance = 0.1;      //0.1                         // how hard to drive toward the target
    double DESIRED_TARGET_AREA = 1.0;
    
    //using ta to drive
    //distance_error = DESIRED_TARGET_AREA - ta;

    //the min_aim_command works like the deadBand concept. When tx angle is small (< 1 degree),
    //due to friction, the robot doesn't turn. To overcome that we add a small power min_aim_command
    //to make the robot turns
    double min_aim_command = 0.01; //0.05;

    limelightSteerCommand = 0.0;
    limelightDriveCommand = 0.0;

    if(debug) System.out.println("tv = "+tv);

    if (tv == 0){                                                 //if target is not found
      limelightHasValidTarget = false;

      steering_adjust = 0.3;                                      //seek for the target by spinning in place at a safe speed.
      limelightSteerCommand = steering_adjust;
      limelightDriveCommand = 0.0;

      tankDrive(0.0, steering_adjust);
 //       Timer.delay(0.05);;

      driveInput = 0.0;
      steerInput = steering_adjust;
      strafeInput = 0.0;

      return;
    }

    limelightHasValidTarget = true;                                   //target found
    
    //steering
    if (tx > 1.0){
      steering_adjust = KpAim * heading_error - min_aim_command;
    }
    else if (tx < -1.0){                                                            //add min_aim_command to overcome the friction if tx is too small
      steering_adjust = KpAim * heading_error + min_aim_command;
    }
    limelightSteerCommand = steering_adjust;

    //SmartDashboard.putNumber("limelightSteerCommand", limelightSteerCommand);

    //driving
    double distance_adjust = kpDistance * distance_error;
    limelightDriveCommand = distance_adjust;

    //SmartDashboard.putNumber("limelightDriveCommand", limelightDriveCommand);

    if (debug) System.out.println("limelightDriveCommand = " + limelightDriveCommand +
                                  ", ty = " + ty +
                                  ", limelightSteerCommand = " + limelightSteerCommand +
                                  ", tx = "+tx);
    if (debug) System.out.println("End update_Limelight_Tracking()");


  }

  public void driveToTarget(){
    /* update_Limelight_Tracking
     * With limelight assistance, drive the robot to target using arcade drive mode
    */
    update_Limelight_Tracking();

    if (limelightHasValidTarget){                                   //if found the target
      tankDrive(limelightDriveCommand * kTestSpeed, 
                  -limelightSteerCommand * kTestSpeed
                  );      //drive to that target
      
      SmartDashboard.putNumber("limelightDriveCommand", limelightDriveCommand);
      SmartDashboard.putNumber("limelightSteerCommand", -limelightSteerCommand);

      driveInput = limelightDriveCommand;
      steerInput = limelightSteerCommand;
      strafeInput = 0.0;
  
    }
    else{                                                           //target not found
      stop();                                          //stop the drivetrain

      driveInput = 0.0;
      steerInput = 0.0;
      strafeInput = 0.0;
    }

  }

  public void log(){
    /*
     * Display important values from NetworkTable generated by limelight camera
     * also the inputs from joystick
    */
    tv = table.getEntry("tv").getDouble(0);
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    ta = table.getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("ta", ta);

    SmartDashboard.putNumber("Drive Input", driveInput);
    SmartDashboard.putNumber("Strafe Input", strafeInput); 
    SmartDashboard.putNumber("Steer Input", steerInput);

    SmartDashboard.putNumber("Left Y value", -joyLeft.getY());
    SmartDashboard.putNumber("Right Y value", -joyRight.getY());
  }

  public void stop(){
    frontLeftMotor.set(0);
    rearLeftMotor.set(0);
    frontRightMotor.set(0);
    rearRightMotor.set(0);
    //test.set(0);
  }
}
