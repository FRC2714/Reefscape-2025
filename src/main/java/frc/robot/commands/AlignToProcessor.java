// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToProcessor extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_backLimelight;

  // private Limelight m_rightLimelight;
  // private Limelight m_leftLimelight;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  public AlignToProcessor(DriveSubsystem m_drivetrain, Limelight m_backLimelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_backLimelight = m_backLimelight;

    xController = new PIDController(0.55, 0, 0); // tune these later
    yController = new PIDController(0.25, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);

    addRequirements(m_drivetrain);

    xController.setSetpoint(Units.inchesToMeters(13)); // tune later
    yController.setSetpoint(0);
    thetaController.setSetpoint(0);
    thetaController.enableContinuousInput(-180, 180);

    xController.setTolerance(.2);
    yController.setTolerance(.2);
    thetaController.setTolerance(.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_backLimelight.setProcessorTagPipeline();
  }

  @Override
  public void execute() {
    if (m_backLimelight.isTargetVisible()) // if the back LL can see something
    {
      if (m_backLimelight.getTargetID() == 3
          || m_backLimelight.getTargetID() == 16) // if the tags are teh processor
      // tags
      {
        thetaController.setSetpoint(90);
        m_drivetrain.drive(
            xController.calculate(
                m_backLimelight.getDistanceToGoalMeters()), // positive since the robot
            // should drive backwards i
            // think
            yController.calculate(m_backLimelight.getXOffsetRadians()),
            thetaController.calculate(m_drivetrain.getHeading()),
            false);
      }
    } else {
      m_drivetrain.drive(0, 0, 0, true);
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
