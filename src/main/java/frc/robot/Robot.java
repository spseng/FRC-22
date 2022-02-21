package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
import edu.wpi.first.wpilibj.Ultrasonic; 
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Robot extends TimedRobot {
  // Instantiate motors
  CANVenom canMotor1 = new CANVenom(1); //left
  CANVenom canMotor2 = new CANVenom(2); //right
  CANVenom canMotor3 = new CANVenom(3); //shoot
  // Instantiate Spark motor controller
  private final Spark mySpark = new Spark(0);
  private final Spark mySpark2 = new Spark(1);
  //Instantiate Xbox controller
  private final XboxController driveController = new XboxController(0);
  private final XboxController utilController = new XboxController(1);
  // untrasonics
  private final Ultrasonic sonic1 = new Ultrasonic(0, 1); 
  private final Ultrasonic sonic2 = new Ultrasonic(2, 3); 
  // sonic sensor
  private AnalogInput sharp = new AnalogInput(0);
  // Instantiate Pneumatics
  private final DoubleSolenoid solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM,7,6);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  // default timer
  private final Timer m_timer = new Timer();
  // Turn abstraction
  private final void turn(double speed) {
    canMotor1.setCommand(ControlMode.Proportional , speed );
    canMotor2.setCommand(ControlMode.Proportional , -speed );
    canMotor3.setCommand(ControlMode.Proportional , -speed );
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
    canMotor3.enable();
    compressor.enableDigital();
  }

  @Override
  public void testPeriodic() {
    // Instantiate ranges
    double range1 = sonic1.getRangeInches();
    double range2 = sonic2.getRangeInches();

    // Ir sensor
    // public double getDistance(){
    //   return(Math.pow(sharp.getAverageVoltage(), -1.2045)) *27.726;
    // }
    
    // Define other starting constants
    double speed = 1.;
    // start compressor
    
    //start solenoid code
    if(utilController.getPOV()==180){
      solenoid.set(Value.kReverse);
    }
    if(utilController.getPOV()==0){
      solenoid.set(Value.kForward);
    }

    // speed changing
    if(driveController.getRightBumper()){
      speed=2.;
    } else if (driveController.getLeftBumper()) {
      speed=0.5;
    } else {
      speed=1.;
    }
    
    // driving 
    if(Math.abs(driveController.getLeftY())>0.1){
      canMotor1.setCommand(ControlMode.Proportional , -speed*driveController.getLeftY() );
    } 
    if(Math.abs(driveController.getRightY())>0.1){
      canMotor2.setCommand(ControlMode.Proportional , speed*driveController.getRightY() );
    }
    if(Math.abs(utilController.getRightTriggerAxis())>0.1){
      canMotor3.setCommand(ControlMode.Proportional , -utilController.getRightTriggerAxis() );
    }
    // Sonic control
    if(Math.pow(range1, 2)>Math.pow(range2,2)+2) {
      double diff = (range1-range2)/range2;
      turn(-0.2*diff);
      System.out.println("Turning Right");
    } else if (Math.pow(range1,2)+2<Math.pow(range2,2)) {
      double diff = (range2-range1)/range1;
      turn(0.2*diff);
      System.out.println("Turning Left");
    }
    // spark control
    if(driveController.getPOV()==90){
      mySpark.set(driveController.getRawAxis(2));
      mySpark2.set(driveController.getRawAxis(2));
    } else if (driveController.getPOV()==270) {
      mySpark.set(-1);
      mySpark2.set(-1);
    } else {
      mySpark.set(0);
      mySpark2.set(0);
    }
    
  }
}
