// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Dragon;
import frc.robot.subsystems.CoralIntake;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToCoral;
import frc.robot.commands.AlignToProcessor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.superstructure.StateMachine;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.StateMachine.State;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight.Align;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final CoralIntake m_coralIntake = new CoralIntake();
  private final LED m_blinkin = new LED();
  private final Limelight m_rightLimelight = new Limelight(LimelightConstants.kRightLimelightName,
                                                           LimelightConstants.kRightCameraHeight,
                                                           LimelightConstants.kRightMountingAngle,
                                                           LimelightConstants.kReefTagHeight);
  private final Limelight m_leftLimelight = new Limelight(LimelightConstants.kLeftLimelightName,
                                                         LimelightConstants.kLeftCameraHeight,
                                                         LimelightConstants.kLeftMountingAngle,
                                                         LimelightConstants.kReefTagHeight);

  private final Limelight m_backLimelight = new Limelight(LimelightConstants.kBackLimelightName,
  LimelightConstants.kBackCameraHeight,
  LimelightConstants.kBackMountingAngle,
  LimelightConstants.kProcessorTagHeight);

  private final Elevator m_elevator = new Elevator();

  private final Dragon m_dragon = new Dragon();

  private final Superstructure m_superstructure = new Superstructure(
    m_algaeSubsystem, m_coralIntake, m_dragon, m_elevator, m_blinkin, m_leftLimelight, m_rightLimelight);

  private final StateMachine m_stateMachine = new StateMachine(m_superstructure);

  Joystick m_operatorController = new Joystick(1);
  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // Operator Controller
  private final JoystickButton L1Button = new JoystickButton(m_operatorController, 1); // L1
  private final JoystickButton L2Button = new JoystickButton(m_operatorController, 2); // L2
  private final JoystickButton L3Button = new JoystickButton(m_operatorController, 3); // L3
  private final JoystickButton L4Button = new JoystickButton(m_operatorController, 4); // L4
  private final JoystickButton coralStationButton = new JoystickButton(m_operatorController, 5); // Coral Station
  private final JoystickButton handoffButton = new JoystickButton(m_operatorController, 6); // L4
  private final JoystickButton stowButton = new JoystickButton(m_operatorController, 8); // Stow
  
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
      .leftTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(m_stateMachine.algaeIntakeSelectCommand(State.EXTAKE))
      .onFalse(m_stateMachine.algaeIntakeSelectCommand(State.STOW));

    m_driverController
      .rightTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(m_stateMachine.algaeIntakeSelectCommand(State.INTAKE))
      .onFalse(m_stateMachine.algaeIntakeSelectCommand(State.STOW));


    m_driverController.leftBumper()
      .onTrue(m_stateMachine.extakeCoral())
      .onFalse(m_stateMachine.stowCoralIntake());

    // Force Actions
    m_driverController.povLeft()
      .whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, Align.LEFT));
    m_driverController.povRight()
      .whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, Align.RIGHT));
  // TODO: add pov up down for coral station and processor
    // Additional
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    
    // Stages
    L1Button.onTrue(m_stateMachine.scoreLevel(State.L1));
    L2Button.onTrue(m_stateMachine.scoreLevel(State.L2));
    L3Button.onTrue(m_stateMachine.scoreLevel(State.L3));
    L4Button.onTrue(m_stateMachine.scoreLevel(State.L4));
    stowButton.onTrue(m_stateMachine.stowElevator());
    handoffButton.onTrue(m_stateMachine.coralHandoff());

    coralStationButton.onTrue(m_coralIntake.intakeCommand());

    m_driverController.leftBumper()
    .whileTrue(m_stateMachine.coralHandoff());
 
  }

  public void setTeleOpDefaultStates() {
    m_stateMachine.algaeIntakeSelectCommand(State.STOW).schedule();
    m_stateMachine.stowElevator().schedule();
    m_stateMachine.coralIntakeSelectCommand(State.STOW).schedule();
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

  public boolean validTarget()
  {
    if(m_leftLimelight.isTargetVisible() && m_rightLimelight.isTargetVisible()) //if both r able to see
    {
        if(m_leftLimelight.getTargetID() == m_rightLimelight.getTargetID()) //if both see the same tag
        {
          return true;
        }
        return false;
    }
    else if(m_leftLimelight.isTargetVisible()) //if only the left is able to see
    {
      return true;
    }
    else if(m_rightLimelight.isTargetVisible()) //if only the right is able to see
    {
      return true;
    }
    return false;
  }
}
