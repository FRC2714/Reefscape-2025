// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Align;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import java.util.function.BooleanSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_rightLimelight;
  private Limelight m_leftLimelight;
  private Align side;

  private PIDController xController, xL2Controller;
  private PIDController yController, yL2Controller;
  private PIDController thetaController, thetaL2Controller;

  private BooleanSupplier isL2;

  private double[] positions;

  public AlignToReef(
      DriveSubsystem m_drivetrain,
      Limelight m_rightLimelight,
      Limelight m_leftLimelight,
      BooleanSupplier isL2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_rightLimelight = m_rightLimelight;
    this.m_leftLimelight = m_leftLimelight;
    this.side = Limelight.SIDE;
    this.isL2 = isL2;

    xController = new PIDController(0.3, 0, 0);
    yController = new PIDController(0.45, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);

    xL2Controller = new PIDController(0.3, 0, 0);
    yL2Controller = new PIDController(0.45, 0, 0);
    thetaL2Controller = new PIDController(0.01, 0, 0);

    xL2Controller.setSetpoint(-0.5);
    yL2Controller.setTolerance(0.01);

    thetaL2Controller.setSetpoint(0);
    thetaL2Controller.enableContinuousInput(-180, 180);

    addRequirements(m_drivetrain);

    xController.setSetpoint(-0.35);
    yController.setSetpoint(0);
    thetaController.setSetpoint(0);
    thetaController.enableContinuousInput(-180, 180);

    xController.setTolerance(.06);
    xL2Controller.setTolerance(.06);
    yController.setTolerance(.05);
    thetaController.setTolerance(1);
    thetaL2Controller.setTolerance(1);

    positions = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Align is finished", false);
    side = Limelight.SIDE;

    // TODO: This can potentially be removed
    if (side == Align.RIGHT) {
      m_rightLimelight.setCoralTagPipelineRight();
      m_leftLimelight.setCoralTagPipelineRight();
      yController.setSetpoint(0.239);
      thetaController.setSetpoint(3);
      thetaL2Controller.setSetpoint(3);
      xL2Controller.setSetpoint(-0.5);
      yL2Controller.setSetpoint(0.239);
    } else if (side == Align.LEFT) {
      m_rightLimelight.setCoralTagPipelineLeft();
      m_leftLimelight.setCoralTagPipelineLeft();
      yController.setSetpoint(-0.235);
      thetaController.setSetpoint(-3);
      thetaL2Controller.setSetpoint(-3);
      xL2Controller.setSetpoint(-0.47);
      yL2Controller.setSetpoint(-0.235);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  // when both cameras see:
  // right align = right default (pipeline 1)
  // left align = left default (pipeline 2)
  // if both cameras see, the default camera will be set to whatever side the
  // driver wants to align to (ie. left bumper = left align = default left camera
  // which uses pipeline 1)
  @Override
  public void execute() {

    SmartDashboard.putNumber("Auto Align/X Setpoint", xController.getSetpoint());
    SmartDashboard.putNumber("Auto Align/Y Setpoint", yController.getSetpoint());
    SmartDashboard.putNumber("Auto Align/Rot Setpoint", thetaController.getSetpoint());

    if (m_leftLimelight.isTargetVisible()
        && m_rightLimelight.isTargetVisible()) { // if both are visible
      if (m_leftLimelight.getTargetID()
          == m_rightLimelight.getTargetID()) { // checks if both see same april tag
        if (side == Align.RIGHT) // driver align right so left camera
        {
          positions = LimelightHelpers.getBotPose_TargetSpace(m_leftLimelight.getName());

        } else // driver aligns left so left camera
        {
          positions = LimelightHelpers.getBotPose_TargetSpace(m_rightLimelight.getName());
        }
      }
    } else if ((m_leftLimelight
        .isTargetVisible())) { // if can only see left, then do whatever we did before
      positions = LimelightHelpers.getBotPose_TargetSpace(m_leftLimelight.getName());

    } else if ((m_rightLimelight.isTargetVisible())) { // same thing when the camera sees right
      positions = LimelightHelpers.getBotPose_TargetSpace(m_rightLimelight.getName());
    }

    if (positions != null) {
      SmartDashboard.putNumber("Auto Align/X Position", positions[2]);
      SmartDashboard.putNumber("Auto Align/Y Position", positions[0]);
      SmartDashboard.putNumber("Auto Align/Rot Position", positions[4]);
      m_drivetrain.drive(
          isL2.getAsBoolean()
              ? xL2Controller.calculate(positions[2])
              : xController.calculate(positions[2]),
          isL2.getAsBoolean()
              ? -yL2Controller.calculate(positions[0])
              : -yController.calculate(positions[0]),
          -thetaController.calculate(positions[4]),
          false);
    }
    SmartDashboard.putBoolean("X Align at setpoint", xController.atSetpoint());
    SmartDashboard.putBoolean("Y Align at setpoint", yController.atSetpoint());
    SmartDashboard.putBoolean("Rot Align at setpoint", thetaController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Align is finished", true);
    m_drivetrain.drive(0, 0, 0, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isL2.getAsBoolean() ? xL2Controller.atSetpoint() : xController.atSetpoint())
        && (isL2.getAsBoolean() ? yL2Controller.atSetpoint() : yController.atSetpoint())
        && thetaController.atSetpoint();
  }
}
