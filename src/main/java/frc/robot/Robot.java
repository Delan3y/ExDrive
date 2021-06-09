// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.PWMTalonFX;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final Talon rightMotor1 = new Talon(12);
  private final Talon rightMotor2 = new Talon(13);

        //left motors
  private final Talon leftMotor1 = new Talon(14);
  private final Talon leftMotor2 = new Talon(15);
  
  private final SpeedController leftGroup = new SpeedControllerGroup(leftMotor1, leftMotor2);
  private final SpeedController rightGroup = new SpeedControllerGroup(rightMotor1, rightMotor2);
  private final XboxController xbox = new XboxController(1);
  DifferentialDrive drivetrain = new DifferentialDrive(leftGroup, rightGroup);

  //joystick 
  Joystick stick = new Joystick(0);
  

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    if(xbox.getBButtonPressed()){
      rightMotor1.set(0.5);
      rightMotor2.set(0.5);
      leftMotor1.set(-0.5);
      leftMotor2.set(-0.5);
    }
    else if(xbox.getAButtonPressed()){
      rightMotor1.set(0);
      rightMotor2.set(0);
      leftMotor1.set(0);
      leftMotor2.set(0);
    }
    else{
      rightMotor1.set(0);
      rightMotor2.set(0);
      leftMotor1.set(0);
      leftMotor2.set(0);
    }
    
    drivetrain.arcadeDrive(0.55*((stick.getY())), 0.55*(stick.getZ()));

  }
}
