// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Align;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCoralStation extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_limelight;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  public AlignToCoralStation(DriveSubsystem m_drivetrain, Limelight m_limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;

    xController = new PIDController(0.5, 0, 0);
    yController = new PIDController(0.25, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);

    addRequirements(m_drivetrain);

    xController.setSetpoint(0.392); // TODO
    yController.setSetpoint(0);
    thetaController.setSetpoint(0);
    thetaController.enableContinuousInput(-180, 180);

    xController.setTolerance(.01);
    yController.setTolerance(.01);
    thetaController.setTolerance(.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
      updateThetaControllerSetpoint(m_limelight.getTargetID());

      m_drivetrain.drive(m_limelight.getDistanceToGoalMetersCoralStation() < 1 ? xController.calculate(m_limelight.getDistanceToGoalMetersCoralStation())
        : 0,
          m_limelight.getDistanceToGoalMetersCoralStation
          () < 0.9 ? yController.calculate(m_limelight.getXOffsetRadians())
              : 0,
          thetaController.calculate(m_drivetrain.getHeading()),
          false);
    } else {
      m_drivetrain.drive(0, 0, 0, true);
    }
  }

  private void updateThetaControllerSetpoint(int targetID) {
    switch (targetID) {
      case 2, 12 -> thetaController.setSetpoint(54.011);
      case 1, 13 -> thetaController.setSetpoint(-54.011);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}