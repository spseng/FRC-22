// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

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
  // Instantiate motors
  CANVenom canMotor1 = new CANVenom(0);
  CANVenom canMotor2 = new CANVenom(1);
  // Instantiate Spark motor controller
  private final Spark mySpark = new Spark(0);
  //Instantiate Xbox controller
  private final XboxController wockyStick = new XboxController(0);
  // untrasonics
  private final Ultrasonic sonic1 = new Ultrasonic(0, 1); 
  private final Ultrasonic sonic2 = new Ultrasonic(2, 3); 
  // default timer
  private final Timer m_timer = new Timer();
  // Turn abstraction
  private final void turn(double speed) {
    canMotor1.setCommand(ControlMode.Proportional , speed );
    canMotor2.setCommand(ControlMode.Proportional , -speed );
  }
  
  @Override
  public void robotInit() {

  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Enable can motors
    canMotor1.enable();
    canMotor2.enable();
  }

  @Override
  public void testPeriodic() {
    // Instantiate ranges
    double range1 = sonic1.getRangeInches();
    double range2 = sonic2.getRangeInches();
    
    // Define other starting constants
    double speed = 1.;
    
    // speed changing
    if(wockyStick.getRightBumper()){
      speed=2.;
    } else if (wockyStick.getLeftBumper()) {
      speed=0.5;
    } else {
      speed=1.;
    }
    
    // driving 
    if(Math.abs(controllerAxes[1])>0.1){
      canMotor1.setCommand(ControlMode.Proportional , speed*controllerAxes[1]/2.0 );
    } 
    if(Math.abs(controllerAxes[5])>0.1){
      canMotor2.setCommand(ControlMode.Proportional , speed*controllerAxes[5]/2.0 );
    }

    // Sonic control
    if(Math.pow(range1, 2)>Math.pow(range2,2)+2) {
      turn(-0.2);
      System.out.println("Turning Right");
    } else if (Math.pow(range1,2)+2<Math.pow(range2,2)) {
      turn(0.2);
      System.out.println("Turning Left");
    }
    // spark control
    if(wockyStick.getYButton()){
      mySpark.set(wockyStick.getRawAxis(2));
    } else if (wockyStick.getXButton()) {
      mySpark.set(-1);
    } else {
      mySpark.set(0);
    }
    
  }
}
