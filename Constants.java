package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Constants {

  // 15 Motor Indexes
  // solenoid index // should be the pcm index
  // Two controllers and indexes
  // Math stuff: quadratic stuff
  // Shooter velocity and angle math, methods, that slope thing
  // (might even want this in the shooter class. no reason to have it in here)
  // - constants like distances and y height. change everything to inches. 32.174
  // in/s^2

  // GENERAL NOTES:
  /*
   * It may not be necessary to use quadratic controller for encoder counts for
   * the indexerMotor. just have (if encoders for motor == required encoders,
   * motor.set(0.0))
   * Design code so it only does the velocity calculation once. set variables to
   * the values once?
   * Instead of having the velocity calculation at all, it might be necessary to
   * just have a bunch of if statements. if required velocity is this, use this
   * speed for the motor
   * with known values. Calculations might be too much for the computer. Use
   * floats?
   * 
   * What if we had a timer that ran throughout the game that put the intake up at
   * a certain time when we're ready to climb
   */

  // Indexes
  public static int flMotorIndex;
  public static int blMotorIndex;
  public static int frMotorIndex;
  public static int brMotorIndex;

  public static int masterShooterMotorIndex;
  public static int slaveShooterMotorIndex;
  public static int hoodMotorIndex;
  public static int turretMotorIndex;

  public static int intakeMotorIndex;
  public static int indexerMotorIndex;
  public static int feederMotorIndex; // thing that keeps them from getting jammed

  public static int lClimberHeightMotorIndex;
  public static int lClimberAngleMotorIndex;
  public static int rClimberHeightMotorIndex;
  public static int rClimberAngleMotorIndex;

  public static int pcmIndex;
  public static int forwardSolenoidIndex;
  public static int reverseSolenoidIndex;

  public static int stickIndex;
  public static int xboxIndex;

  //////////////////////////////////////////////////////
  // Controllers

  public static Joystick stick = new Joystick(stickIndex);
  public static Joystick xbox = new Joystick(xboxIndex);

  //////////////////////////////////////////////////////
  // Math

  

  //////////////////////////////////////////////////////
  // Functions

  public static double quadraticPositionAndSpeed(double minimumMotorSpeed, double maximumMotorSpeed,
      double positionGoal, double currentPosition) {

    // position could be angle offset, encoder count, inches, etc.

    // maximum speed should be reached at the middle position

    // need to look at system of equations again and see if I want endpoint to just
    // be positionGoal with speed 0. (position, 0)

    double a = ((positionGoal * maximumMotorSpeed - minimumMotorSpeed * positionGoal
        - minimumMotorSpeed * (positionGoal / 2) + minimumMotorSpeed * (positionGoal / 2))
        / (positionGoal * (positionGoal / 2) * ((positionGoal / 2) - positionGoal)));

    double b = ((maximumMotorSpeed - a * (positionGoal / 2) * (positionGoal / 2) - minimumMotorSpeed)
        / (positionGoal / 2));

    double speed = a * currentPosition * currentPosition + b * currentPosition + minimumMotorSpeed;

    return speed; // value would be from 0.0 to 1.0 like any motor needs to be
  }

  
}