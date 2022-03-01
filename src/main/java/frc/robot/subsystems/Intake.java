// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  Spark intake;

  /** Creates a new Intake. */
  public Intake() {
    intake = new Spark(Constants.INTAKE);
    intake.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeBall(double speed)
  {
    //check the axis in drive station for right trigger 
    intake.set(speed);
  }
  public void stop()
  {
    intake.set(0);
  }
  
}
