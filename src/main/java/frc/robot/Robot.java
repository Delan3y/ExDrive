// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

//0= lifter; 1 = neo/spark motor; 2 = intake2; 3 = intake


/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {



        //drivetrain motors
    WPI_TalonFX motor1 = new WPI_TalonFX(0);
    WPI_TalonFX motor2 = new WPI_TalonFX(1);
    WPI_TalonFX motor3 = new WPI_TalonFX(2);
    WPI_TalonFX motor4 = new WPI_TalonFX(3);

    //CIM motors
    //private final VictorSP climber = new VictorSP(0);
    private final VictorSP intake = new VictorSP(2);
    private final VictorSP intake2 = new VictorSP(3);

    private final PWMSparkMax shooter = new PWMSparkMax(1);
    
  private final SpeedController leftGroup = new SpeedControllerGroup(motor1, motor2);
  private final SpeedController rightGroup = new SpeedControllerGroup(motor3, motor4);
  private final XboxController xbox = new XboxController(0);

  DifferentialDrive drivetrain = new DifferentialDrive(leftGroup, rightGroup);

  Faults _faults_1 = new Faults();
  Faults _faults_2 = new Faults();
  Faults _faults_3 = new Faults();
  Faults _faults_4 = new Faults();

  //joystick 
  Joystick _joystick = new Joystick(1);
  
  //Pneumatics
  private final Compressor comp = new Compressor();
  private final DoubleSolenoid solenoid = new DoubleSolenoid(0,1);

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    String work = "";

        /* get gamepad stick values */
        double forw = -0.35 * _joystick.getRawAxis(1); /* positive is forward */
        double turn = +0.35 * _joystick.getRawAxis(2); /* positive is right */
        boolean btn1 = _joystick.getRawButton(1); /* is button is down, print joystick values */
        
        /* deadband gamepad 10% */
        if (Math.abs(forw) < 0.10) {
            forw = 0;
        }
        if (Math.abs(turn) < 0.10) {
            turn = 0;
        }

        /* drive robot */
        if(xbox.getAButtonPressed())
        drivetrain.arcadeDrive(forw, turn);
        else if(xbox.getBButtonPressed())
        drivetrain.arcadeDrive(0.5*forw + forw,0.5*turn + turn);

        /*
         * [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for
         * RIGHT
         */
        work += " GF:" + forw + " GT:" + turn;

        /* get sensor values */
        // double leftPos = _leftFront.GetSelectedSensorPosition(0);
        // double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
        double motor1VelUnitsPer100ms = motor1.getSelectedSensorVelocity(0);
        double motor2VelUnitsPer100ms = motor2.getSelectedSensorVelocity(0);
        double motor3VelUnitsPer100ms = motor3.getSelectedSensorVelocity(0);
        double motor4VelUnitsPer100ms = motor4.getSelectedSensorVelocity(0);

        work += " L:" +  motor1VelUnitsPer100ms +"\n" + motor2VelUnitsPer100ms + " \nR:" + motor3VelUnitsPer100ms +"\n" + motor4VelUnitsPer100ms;

        //tiny green wheels
        if(xbox.getBumper(Hand.kLeft)){
          intake2.set(0.5);
        }
        else if(xbox.getBumper(Hand.kRight)){
          intake2.set(-0.5);
        }
        else
          intake2.set(0);

        //maroon wheels
        if(xbox.getXButton())
          intake.set(-0.6);
        else if(xbox.getYButton())
         intake.set(0.6);
        else
          intake.set(0);

        //shooter
        if(xbox.getAButton()){
          shooter.set(0.1);
        }
        else
          shooter.set(0);
        
        if(xbox.getStartButton())
          solenoid.set(DoubleSolenoid.Value.kForward);
        else if(xbox.getBackButton())
          solenoid.set(DoubleSolenoid.Value.kReverse);
        else
          solenoid.set(DoubleSolenoid.Value.kOff);

        /*
         * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
         */
        motor1.getFaults(_faults_1);
        motor2.getFaults(_faults_2);
        motor3.getFaults(_faults_3);
        motor4.getFaults(_faults_4);

        if (_faults_1.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        if (_faults_2.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }
        if (_faults_3.SensorOutOfPhase) {
          work += " L sensor is out of phase";
      }
      if (_faults_4.SensorOutOfPhase) {
          work += " R sensor is out of phase";
      }

        /* print to console if btn1 is held down */
        if (btn1) 
            System.out.println(work);
  }
  @Override
  public void robotInit() {
    /* factory default values */
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
    motor3.configFactoryDefault();
    motor4.configFactoryDefault();



    /* set up followers */
    //rghtFollower.follow(_rghtFront);
    //_leftFollower.follow(_leftFront);

    /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    motor3.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this
    motor4.setInverted(TalonFXInvertType.Clockwise);
    motor1.setInverted(TalonFXInvertType.Clockwise); // !< Update this
    motor2.setInverted(TalonFXInvertType.CounterClockwise);


    /*
     * set the invert of the followers to match their respective master controllers
     */
   // _rghtFollower.setInverted(InvertType.FollowMaster);
    //_leftFollower.setInverted(InvertType.FollowMaster);
/*
 * Talon FX does not need sensor phase set for its integrated sensor
 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
 * 
 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
 */
    // _rghtFront.setSensorPhase(true);
    // _leftFront.setSensorPhase(true);

    /*
     * WPI drivetrain classes defaultly assume left and right are opposite. call
     * this so we can apply + to both sides when moving forward. DO NOT CHANGE
     */
    drivetrain.setRightSideInverted(false);
}
  }
