// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private SparkFlex intakeMotor;
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor = new SparkFlex(62, MotorType.kBrushless);
  }

  public void intake() {
    intakeMotor.set(1);
  }

  public void intakeMedium() {
    intakeMotor.set(0.75);
  }

  public void intakeSlow() {
    intakeMotor.set(0.50);
  }

   public void stop() {
    intakeMotor.set(0);
  }

  public void outtake() {
    intakeMotor.set(-1);
  }

  public void outtakeMedium() {
    intakeMotor.set(-0.75);
  }
  public void outtakeSlow() {
    intakeMotor.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
