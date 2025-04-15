// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Align;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.ScoreLevel;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_rightLimelight;
  private Limelight m_leftLimelight;
  private Align side;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  private double[] positions;

  public AlignToReef(
      DriveSubsystem m_drivetrain, Limelight m_rightLimelight, Limelight m_leftLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_rightLimelight = m_rightLimelight;
    this.m_leftLimelight = m_leftLimelight;
    this.side = Limelight.SIDE;

    xController = new PIDController(0.3, 0, 0);
    yController = new PIDController(0.45, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);

    addRequirements(m_drivetrain);

    xController.setSetpoint(0);
    yController.setSetpoint(0);
    thetaController.setSetpoint(0);
    thetaController.enableContinuousInput(-180, 180);

    xController.setTolerance(.06);
    yController.setTolerance(.05);
    thetaController.setTolerance(1);

    positions = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Align is finished", false);
    side = Limelight.SIDE;

    if (StateMachine.LEVEL == ScoreLevel.L2) {
      if (side == Align.RIGHT) xController.setSetpoint(-0.5);
      else xController.setSetpoint(-0.47);
      yController.setTolerance(0.01);
    } else {
      xController.setSetpoint(-0.35);
      yController.setTolerance(0.06);
    }

    // TODO: This can potentially be removed
    if (side == Align.RIGHT) {
      m_rightLimelight.setCoralTagPipelineRight();
      m_leftLimelight.setCoralTagPipelineRight();
      yController.setSetpoint(0.239);
      thetaController.setSetpoint(3);
    } else if (side == Align.LEFT) {
      m_rightLimelight.setCoralTagPipelineLeft();
      m_leftLimelight.setCoralTagPipelineLeft();
      yController.setSetpoint(-0.235);
      thetaController.setSetpoint(-3);
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
          xController.calculate(positions[2]),
          -yController.calculate(positions[0]),
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
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
