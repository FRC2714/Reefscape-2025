// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCoralStation extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_limelight;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  private double[] positions;

  public AlignToCoralStation(DriveSubsystem m_drivetrain, Limelight m_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;

    xController = new PIDController(0.3, 0, 0);
    yController = new PIDController(0.2, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);

    addRequirements(m_drivetrain);

    xController.setSetpoint(-1.55); // TODO
    yController.setSetpoint(-0.5);
    thetaController.setSetpoint(-29);
    thetaController.enableContinuousInput(-180, 180);

    xController.setTolerance(.1);
    yController.setTolerance(.1);
    thetaController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Align is finished", false);
    m_limelight.setCoralStationPipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // when both cameras see:
  // right align = right default (pipelin 1)
  // left align = left default (pipelin 2)
  // if both cameras see, the default camera will be set to whatever side the
  // driver wants to align to (ie. left bumper = left align = default left camera
  // which uses pipeline 1)
  @Override
  public void execute() {
    if ((m_limelight.isTargetVisible())) { // if can only see left, then do whatever we did before
      positions = LimelightHelpers.getBotPose_TargetSpace(m_limelight.getName());
    }

    if (positions != null) {
      SmartDashboard.putNumber("Coral Auto Align/X Position", positions[2]);
      SmartDashboard.putNumber("Coral Auto Align/Y Position", positions[0]);
      SmartDashboard.putNumber("Coral Auto Align/Rot Position", positions[4]);
      m_drivetrain.drive(
          positions[2] > 0
              ? xController.calculate(positions[2])
              : -xController.calculate(positions[2]),
          yController.calculate(positions[0]),
          thetaController.calculate(positions[4]),
          false);
    }
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
