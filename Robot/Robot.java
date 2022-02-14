// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // motor controllers
  private CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax backLeft = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax backRight = new CANSparkMax(4, MotorType.kBrushless);

  // SpeedControllerGroup and DifferentialDrive
  private SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeft, backLeft);
  private SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRight, backRight);
  private DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  // variables to control the drive
  private final double kDeadBandLimit = 0.5; // deadband of the drive was .15
  private final double kDRIVE_SPEED = 0.75; // constant applies to lower the speed for better control
  private final double kDRIVE_TURN = 0.75; // constant applies to maintain the stability of the robot

  // Joysticks
  private Joystick joyLeft = new Joystick(0);
  private Joystick joyRight = new Joystick(1);

  // encoders
  private CANEncoder frontLeftEncoder =  frontLeft.getEncoder();
  private CANEncoder frontRightEncoder = frontRight.getEncoder();
  private CANEncoder backLeftEncoder = backLeft.getEncoder();
  private CANEncoder backRightEncoder = backRight.getEncoder();

  //According to Neo Specification (https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf)
  //Hall-Sensor Encoder Resolution: 42 counts (ticks) per rev
  // unit conversion from tick to feet
  // 42 ticks = (π * wheelDiameterInches) * (ft / 12 Inches)
  // 42 ticks = (π * 6 Inches) * (ft / 12 Inches)
  // ticks = 1/42 * (6 π / 12)
  // ticks = 1/42 * π/2
//  private final double kDriveTick2Feet = 1.0 / 42 * Math.PI / 2;
  private final double kDriveTick2Feet = 1.0 / 42 * 6 * Math.PI / 12 * 4.16;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //set inverted (*** CAUTION - MUST check after deploy *** : turn each wheel clockwise manually, 
    //if encoder value is positive: <speedController>.setInverted(false);
   // if encoder value is negative: <speedController>.setInverted(true);
         //invert the right side

    //set drive deadBand
    drive.setDeadband(kDeadBandLimit);

    enableMotors(true);

    frontRight.setInverted(true);       //invert the right side
    backRight.setInverted(true); 
    frontLeft.setInverted(false);
    backLeft.setInverted(false);
    
    //reset encoders to zero position
    frontLeftEncoder.setPosition(0.0);
    frontRightEncoder.setPosition(0.0);
    backLeftEncoder.setPosition(0.0);
    backRightEncoder.setPosition(0.0);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("joyLeft Y", joyLeft.getY());
    SmartDashboard.putNumber("joyRight Y", joyRight.getY());

    //measure the distance (in ft) how far the drive goes
    SmartDashboard.putNumber("Left Drive Encoder Value", frontLeftEncoder.getPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Left Drive Encoder Value", frontRightEncoder.getPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Left Drive Encoder Value", backLeftEncoder.getPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Drive Encoder Value", backRightEncoder.getPosition() * kDriveTick2Feet);
  }

  @Override
  public void autonomousInit() {
    enableMotors(true);
    //reset encoders to zero position

  }

  @Override
  public void autonomousPeriodic() {
    double leftPosition = frontLeftEncoder.getPosition()* kDriveTick2Feet;
    double rightPosition = frontRightEncoder.getPosition()* kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;

    //drive at 60% of power until 10 ft was 10 but changed to 5
    if (distance < 5.0) {
      drive.tankDrive(0.6, -0.6);
    } else {
      drive.tankDrive(0, 0);
    }

  }

  @Override
  public void teleopInit() {
    enableMotors(true);
  }

  @Override
  public void teleopPeriodic() {
    double leftSpeed = -joyLeft.getY() * kDRIVE_SPEED;  //negate because pushing joystick forward always return negative
    double rightSpeed = -joyRight.getY() * kDRIVE_TURN; 

    //double leftSpeed = speed + turn;
    //double rightSpeed = speed - turn;

    //drive.tankDrive(speed, turn);
    leftGroup.set(leftSpeed);
    rightGroup.set(rightSpeed);

    //if (Math.abs(speed) < kDeadBandLimit) speed = 0.0;
    //if (Math.abs(turn) < kDeadBandLimit)  turn = 0.0;


   
    
    
     //negate because pushing joystick upward always return negative

    /* Because we use drive.setDeadBand, we don't need the following codes
    //set deadBand
    if (Math.abs(speed) < kDeadBandLimit) speed = 0.0;
    if (Math.abs(turn) < kDeadBandLimit)  turn = 0.0;
    */

    /* Because we use drive.tankDrive(), we don't need the following codes
    double leftSpeed = speed + turn;
    double rightSpeed = speed - turn;
   
    drive.tankDrive(speed, turn);

    frontLeft.set(leftSpeed);
    backLeft.set(leftSpeed);
    frontRight.set(-rightSpeed);
    backRight.set(-rightSpeed);
    */

    
    

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
    enableMotors(true);
  }

  @Override
  public void testPeriodic() {
    double speed = -joyLeft.getY();

     // For Testing Purposes
      if (joyLeft.getRawButton(5)) {          //frontLeft fwd/backwd
      frontLeft.set(speed);
      } 
    
      else if (joyLeft.getRawButton(3)) {     //backLeft fwd/backwd
        backLeft.set(speed);
      }

      else if (joyLeft.getRawButton(6)) {     //frontRightt fwd/backwd
        frontRight.set(speed);
      }

      else if (joyLeft.getRawButton(4)) {     //backRight fwd/backwd
        backRight.set(speed);
      }

      else if(joyLeft.getRawButton(11)){       //turn left
        drive.tankDrive(0.0, -0.6);
      }

      else if(joyLeft.getRawButton(7)){       //turn right
        drive.tankDrive(0.6, 0.0);
      }

      else{
        frontLeft.set(0);
        backLeft.set(0);
        frontRight.set(0);
        backRight.set(0);
      }
    
  }

  private void enableMotors(boolean on) {
    CANSparkMax.IdleMode mode;

    if (on) {
      mode = CANSparkMax.IdleMode.kBrake;
    } else {
      mode = CANSparkMax.IdleMode.kCoast;
    }

    frontLeft.setIdleMode(mode);
    backLeft.setIdleMode(mode);
    frontRight.setIdleMode(mode);
    backRight.setIdleMode(mode);
  }

}
