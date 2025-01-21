// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToCoral;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final AlgaeIntake m_algaeSubsystem = new AlgaeIntake();
  private final Limelight m_rightLimelight = new Limelight(LimelightConstants.kRightLimelightName,
                                                           LimelightConstants.kRightCameraHeight,
                                                           LimelightConstants.kRightMountingAngle,
                                                           LimelightConstants.kReefTagHeight);
  private final Limelight m_leftLimelight = new Limelight(LimelightConstants.kLeftLimelightName,
                                                         LimelightConstants.kLeftCameraHeight,
                                                         LimelightConstants.kLeftMountingAngle,
                                                         LimelightConstants.kReefTagHeight);

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();

    // Configure default commands
    // COMMENT OUT BEFORE RUNNING SYSID
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .onTrue(m_algaeSubsystem.intakeCommand())
        .onFalse(m_algaeSubsystem.stowCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.scoreAlgaeProcessor())
        .onFalse(m_algaeSubsystem.stowCommand());  

    m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.leftBumper().whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, LimelightConstants.kLeftReefBranchPipeline));
    m_driverController.rightBumper().whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, LimelightConstants.kRightReefBranchPipeline));

    m_driverController.x().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));


  }

  public void setTeleOpDefaultStates() {
    m_algaeSubsystem.stowCommand().schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    return autoChooser.getSelected();
  }
}
