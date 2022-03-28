// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.Constants.DriveConstants;

public class Shooter extends SubsystemBase {
  CANSparkMax shooter1;
  CANSparkMax shooter2;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new CANSparkMax(Constants.SHOOTER1, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.SHOOTER2, MotorType.kBrushless);
    //started to use the seVelocity() command to have the motor move at a constant speed regardless of battery power
    shooter2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shootBall(double speed)
  {
    shooter1.set(speed);
    shooter2.set(speed);
  }

  public void stop()
  {
    shooter1.set(0);
    shooter2.set(0);
  }
}
