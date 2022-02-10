// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// odom imports:
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
// wocky imports
import edu.wpi.first.wpilibj.Ultrasonic; 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick stick1 = new Joystick(0);
  private final Joystick stick2 = new Joystick(1);
  private final Timer m_timer = new Timer();
  
  // odom constructors:
  CANVenom canMotor1 = new CANVenom(0); // argument is motor ID number. get this by web browser: 10.15.12.2:5812
  CANVenom canMotor2 = new CANVenom(1); // argument is motor ID number. get this by web browser: 10.15.12.2:5812
  // wocky constructors:
  private final XboxController wockyStick = new XboxController(0);
  private final Ultrasonic sonic1 = new Ultrasonic(0, 1); 
  private final Ultrasonic sonic2 = new Ultrasonic(2, 3); 


  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightDrive.setInverted(false);
    canMotor1.setInverted(true);
    Ultrasonic.setAutomaticMode(true);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    // m_robotDrive.arcadeDrive(m_stick.getX(), m_stick.getX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    System.out.println("testInit called for jason!");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // System.out.println("testPeriodic called for odom!" + Timer.getMatchTime());
  
    // controlling the Venom motor with CAN
    canMotor1.enable();

    canMotor2.enable(); 

    double camM1Temp = canMotor1.getTemperature();
    double camM1Pos = canMotor1.getPosition();
    // System.out.println("camM1Temp = " + camM1Temp);
    // System.out.println("camM1Pos = " + camM1Pos);

    // joystick control test:
    // double rudyThrottle = m_stick.getThrottle();
    // // System.out.println("rudyThrottle = " + rudyThrottle);
    // double rudyTwist = m_stick.getTwist();
    // // System.out.println("rudyTwist = " + rudyTwist);
    // double rudyZ = m_stick.getZ();
    // double rudyX = m_stick.getX();
    // double rudyY = m_stick.getY();

    double[] controllerAxes = new double[6];

    for(int i=0;i<6;i++){
      controllerAxes[i]=wockyStick.getRawAxis(i);
    }

    // System.out.println("("+controllerAxes[0]+", "+controllerAxes[1]+", "
    // +controllerAxes[2]+", "+controllerAxes[3]+", "+controllerAxes[4]+", "+controllerAxes[5]+")");


   
    // System.out.println("POV: "+m_stick.getPOV());
  
    // print all the control modes:
    // for (CANVenom.ControlMode c : CANVenom.ControlMode.values())
    //   System.out.println(c);
    

    // canMotor1.SetCommand( ControlMode::kSpeedControl, 1000);     /// BAD old??? code
    // canMotor1.setCommand(ControlMode.SpeedControl, 6000.0*controllerAxes[2]);
    // canMotor1.setCommand(ControlMode.Proportional , -0.2 );
    // canMotor2.setCommand(ControlMode.Proportional , controllerAxes[1] );
    // canMotor2.setCommand(ControlMode.SpeedControl , -1250 );
    // canMotor1.setCommand(ControlMode.Proportional , controllerAxes[5] );
    // System.out.println(canMotor1.getSpeed() + " , " + canMotor2.getSpeed());
    // System.out.println(controllerAxes[1] + " , " + controllerAxes[5]);

    //;//-5000*controllerAxes[2];d
    // double speedRight = -2000.0*controllerAxes[5];//-5000*controllerAxes[3];
    double speed = 1.;
    boolean aState =false;
    boolean aPrevState = false;
    if(wockyStick.getRightBumper()){
      speed=2.;
    } else if (wockyStick.getLeftBumper()) {
      speed=0.5;
    } else {
      speed=1.;
    }
    if (wockyStick.getAButton()){
      if (Math.abs(controllerAxes[1])>0.1){
        canMotor1.setCommand(ControlMode.Proportional , speed*controllerAxes[1]/2.0 );
        canMotor2.setCommand(ControlMode.Proportional , speed*controllerAxes[1]/2.0 );
      }
    } else { // straight mode
      if(Math.abs(controllerAxes[1])>0.1){
        canMotor1.setCommand(ControlMode.Proportional , speed*controllerAxes[1]/2.0 );
      } 
      if(Math.abs(controllerAxes[5])>0.1){
        canMotor2.setCommand(ControlMode.Proportional , speed*controllerAxes[5]/2.0 );
      }
    }

    double range1 = sonic1.getRangeInches();
    double range2 = sonic2.getRangeInches();
    System.out.println(range1+", "+range2);
    
    //  else if (wockyStick.getBButton()) {
    //   speed = 0.5;
    // } else {
    //   speed=1.0;
    // }
    
    
   


 
   
    // canMotor1.setCommand(mode, command, kF, b);
    
  }
}
