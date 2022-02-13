package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  static Shooter shooter = new Shooter();
  static DriveTrain drive = new DriveTrain();
  static Intake intake = new Intake();
  static Climber climber = new Climber();

  ///////////////////////////////////////////////////////
  // Robot

  @Override
  public void robotInit() {
    drive.isDriverControlEnabled = true;
  }

  @Override
  public void robotPeriodic() {

  }

  ////////////////////////////////////////////////////////
  // Autonomous

  @Override
  public void autonomousInit() {
    drive.isDriverControlEnabled = false;
  }

  @Override
  public void autonomousPeriodic() {

  }

  ///////////////////////////////////////////////////////////
  // Tele-operated

  @Override
  public void teleopInit() {
    drive.isDriverControlEnabled = true;
  }

  @Override
  public void teleopPeriodic() {

  }

  /////////////////////////////////////////////////////////////
  // Test

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  //////////////////////////////////////////////////////////////
  // Disabled

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    
  }
}