package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber {

    // two talons // control height
    // two spark maxes // control angle

    WPI_TalonFX lHeightMotor = new WPI_TalonFX(Constants.lClimberHeightMotorIndex);
    CANSparkMax lAngleMotor = new CANSparkMax(Constants.lClimberAngleMotorIndex, MotorType.kBrushless);

    WPI_TalonFX rHeightMotor = new WPI_TalonFX(Constants.rClimberAngleMotorIndex);
    CANSparkMax rAngleMotor = new CANSparkMax(Constants.rClimberAngleMotorIndex, MotorType.kBrushless);

    
}
