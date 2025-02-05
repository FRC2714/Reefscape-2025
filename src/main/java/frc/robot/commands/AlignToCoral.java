// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
<<<<<<< HEAD
import frc.robot.utils.LimelightHelpers.RawFiducial;
=======
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
>>>>>>> pathplanner

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToCoral extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_rightLimelight;
  private Limelight m_leftLimelight;

  // private Limelight m_rightLimelight;
  // private Limelight m_leftLimelight;

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  private int pipelineNum;
  private int closestTagID;

  public AlignToCoral(DriveSubsystem m_drivetrain, Limelight m_rightLimelight, Limelight m_leftLimelight,
      int pipelineNum) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_rightLimelight = m_rightLimelight;
    this.m_leftLimelight = m_leftLimelight;
    this.pipelineNum = pipelineNum;

<<<<<<< HEAD
    this.closestTagID = -1;

=======
>>>>>>> pathplanner
    xController = new PIDController(0.55, 0, 0); // tune these later
    yController = new PIDController(0.25, 0, 0);
    thetaController = new PIDController(0.01, 0, 0);

    addRequirements(m_drivetrain);

    xController.setSetpoint(Units.inchesToMeters(13));
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
    if (pipelineNum == 1) {
      m_rightLimelight.setCoralTagPipelineRight();
      m_leftLimelight.setCoralTagPipelineRight();
    } else if (pipelineNum == 2) {
      m_rightLimelight.setCoralTagPipelineLeft();
      m_leftLimelight.setCoralTagPipelineLeft();
    }

    double closestDistance = Double.MAX_VALUE;
    if (m_rightLimelight.isTargetVisible()) {
      double distance = m_rightLimelight.getDistanceToGoalMeters();
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTagID = m_rightLimelight.getTargetID();
      }
    }
    if (m_leftLimelight.isTargetVisible()) {
      double distance = m_leftLimelight.getDistanceToGoalMeters();
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTagID = m_leftLimelight.getTargetID();
      }
    }
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
<<<<<<< HEAD
    if (m_leftLimelight.isTargetVisible() && m_rightLimelight.isTargetVisible()) {// if both are visible
      if (m_leftLimelight.getTargetID() == m_rightLimelight.getTargetID()) { // checks if both see same april tag
        if (pipelineNum == 1) // driver align right so right camera
        {
          updateThetaControllerSetpoint(m_leftLimelight.getTargetID());

          m_drivetrain.drive(-xController.calculate(m_leftLimelight.getDistanceToGoalMeters()),
              yController.calculate(m_leftLimelight.getXOffsetRadians()),
              thetaController.calculate(m_drivetrain.getHeading()),
              false);
        } else // driver aligns left so left camera
        {
          updateThetaControllerSetpoint(m_rightLimelight.getTargetID());

          m_drivetrain.drive(-xController.calculate(m_rightLimelight.getDistanceToGoalMeters()),
              yController.calculate(m_rightLimelight.getXOffsetRadians()),
              thetaController.calculate(m_drivetrain.getHeading()),
              false);
        }
      } else {
        m_drivetrain.drive(0, 0, 0, true);
      }
    } else if ((m_leftLimelight.isTargetVisible())) { // if can only see left, then do whatever we did before
      updateThetaControllerSetpoint(m_leftLimelight.getTargetID());
=======
    PPHolonomicDriveController.overrideXFeedback(() -> {
      return 0.0;
    });
    PPHolonomicDriveController.overrideYFeedback(() -> {
      return 0.0;
    });
    PPHolonomicDriveController.overrideRotationFeedback(() -> {
      return 0.0;
    });
    if (m_leftLimelight.isTargetVisible()) {
      switch (m_leftLimelight.getTargetID()) {
        case 6:
        case 19:
          thetaController.setSetpoint(300);
          break;

        case 7:
        case 18:
          thetaController.setSetpoint(0);
          break;

        case 8:
        case 17:
          thetaController.setSetpoint(60);
          break;

        case 9:
        case 16:
          thetaController.setSetpoint(120);
          break;

        case 10:
        case 15:
          thetaController.setSetpoint(180);
          break;

        case 11:
        case 14:
          thetaController.setSetpoint(240);
          break;
      }
>>>>>>> pathplanner
      m_drivetrain.drive(-xController.calculate(m_leftLimelight.getDistanceToGoalMeters()),
          yController.calculate(m_leftLimelight.getXOffsetRadians()),
          thetaController.calculate(m_drivetrain.getHeading()),
          false);
<<<<<<< HEAD
    } else if ((m_rightLimelight.isTargetVisible())) { // same thing when the camera sees right
      updateThetaControllerSetpoint(m_rightLimelight.getTargetID());
=======
    } else if (m_rightLimelight.isTargetVisible()) {
      switch (m_rightLimelight.getTargetID()) {
        case 6:
        case 19:
          thetaController.setSetpoint(300);
          break;

        case 7:
        case 18:
          thetaController.setSetpoint(0);
          break;

        case 8:
        case 17:
          thetaController.setSetpoint(60);
          break;

        case 9:
        case 16:
          thetaController.setSetpoint(120);
          break;

        case 10:
        case 15:
          thetaController.setSetpoint(180);
          break;

        case 11:
        case 14:
          thetaController.setSetpoint(240);
          break;
      }
>>>>>>> pathplanner
      m_drivetrain.drive(-xController.calculate(m_rightLimelight.getDistanceToGoalMeters()),
          yController.calculate(m_rightLimelight.getXOffsetRadians()),
          thetaController.calculate(m_drivetrain.getHeading()),
          false);
    } else {
      m_drivetrain.drive(0, 0, 0, true);
    }
<<<<<<< HEAD

  }

  private void updateThetaControllerSetpoint(int targetID) {
    switch (targetID) {
      case 6, 19 -> thetaController.setSetpoint(300);
      case 7, 18 -> thetaController.setSetpoint(0);
      case 8, 17 -> thetaController.setSetpoint(60);
      case 9, 16 -> thetaController.setSetpoint(120);
      case 10, 15 -> thetaController.setSetpoint(180);
      case 11, 14 -> thetaController.setSetpoint(240);
    }
=======
    PPHolonomicDriveController.clearXFeedbackOverride();
    PPHolonomicDriveController.clearYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();
>>>>>>> pathplanner
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
