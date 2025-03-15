// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToCoral;
import frc.robot.commands.AlignToCoralStation;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Dragon;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Align;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.ScoreLevel;
import frc.robot.subsystems.drive.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralIntake m_coralIntake = new CoralIntake();
  private final Elevator m_elevator = new Elevator();
  private final Dragon m_dragon = new Dragon();
  private final Climber m_climber = new Climber();

  private final Limelight m_rightLimelight =
      new Limelight(
          LimelightConstants.kRightLimelightName,
          LimelightConstants.kRightCameraHeight,
          LimelightConstants.kRightMountingPitch,
          LimelightConstants.kReefTagHeight);
  private final Limelight m_leftLimelight =
      new Limelight(
          LimelightConstants.kLeftLimelightName,
          LimelightConstants.kLeftCameraHeight,
          LimelightConstants.kLeftMountingPitch,
          LimelightConstants.kReefTagHeight);

  private final Limelight m_backLimelight =
      new Limelight(
          LimelightConstants.kBackLimelightName,
          LimelightConstants.kBackCameraHeight,
          LimelightConstants.kBackMountingPitch,
          LimelightConstants.kCoralStationTagHeight);

  private final StateMachine m_stateMachine =
      new StateMachine(m_dragon, m_elevator, m_coralIntake, m_climber);

  // Mech2d Stuff
  // private final Mech2dManager m_mech2dManager = new Mech2dManager(m_elevator,
  // m_dragon, m_coralIntake, m_algaeIntake);

  // public Mech2dManager getMech2dManager() {
  // return m_mech2dManager;
  // }

  Joystick m_leftController = new Joystick(1); // operator controller 1
  Joystick m_rightController = new Joystick(2); // operator controller 2

  // The driver's controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  // Operator Controller
  private final Trigger L1Button = new Trigger(() -> m_rightController.getRawAxis(1) > 0.25); // L1
  private final Trigger L2Button = new Trigger(() -> m_rightController.getRawAxis(0) < -0.25); // L2
  private final Trigger L3Button = new Trigger(() -> m_rightController.getRawAxis(0) > 0.25); // L3
  private final Trigger L4Button = new Trigger(() -> m_rightController.getRawAxis(1) < -0.25); // L4

  private final Trigger rumble = new Trigger(m_coralIntake::shouldRumble);

  private final JoystickButton coralExtakeButton = new JoystickButton(m_rightController, 2);
  private final JoystickButton overrideStateMachineButton =
      new JoystickButton(m_rightController, 7); // L4
  private final JoystickButton autoHandoffButton = new JoystickButton(m_rightController, 10);
  private final JoystickButton handoffButton = new JoystickButton(m_rightController, 1);
  private final JoystickButton stowButton = new JoystickButton(m_rightController, 4); // Stow
  private final Trigger loadCoralButton = new JoystickButton(m_rightController, 8);
  private final Trigger coralOnDragonButton = new JoystickButton(m_rightController, 9);
  private final JoystickButton climbDeployToggleButton = new JoystickButton(m_rightController, 11);
  private final JoystickButton sheeshButton = new JoystickButton(m_rightController, 12);
  private final JoystickButton intakeOneCoralButton = new JoystickButton(m_rightController, 3);
  // private final JoystickButton setStartingConfigButton = new JoystickButton(m_rightController,
  // 5);
  private final JoystickButton removeAlgaeHighLevelButton =
      new JoystickButton(m_rightController, 6);
  private final JoystickButton removeAlgaeLowLevelButton = new JoystickButton(m_rightController, 5);

  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    NamedCommands.registerCommand(
        "Dragon standby", m_stateMachine.dragonStandbySequence().withTimeout(0.3));
    NamedCommands.registerCommand("Score Coral", m_stateMachine.scoreCoralAuto());
    NamedCommands.registerCommand("L4", m_stateMachine.scoreReadyL4Sequence(ScoreLevel.L4));
    NamedCommands.registerCommand("L3", m_stateMachine.scoreReadySequence(ScoreLevel.L3));
    NamedCommands.registerCommand("L2", m_stateMachine.scoreReadySequence(ScoreLevel.L2));
    NamedCommands.registerCommand("L1", m_stateMachine.scoreReadySequence(ScoreLevel.L1));
    NamedCommands.registerCommand("Intake Coral", m_stateMachine.intakeSequenceAuto());
    NamedCommands.registerCommand("Handoff", m_stateMachine.handoffSequence());
    NamedCommands.registerCommand("Extake Coral", m_stateMachine.extakeCoral().withTimeout(2));
    NamedCommands.registerCommand("Enable Auto Handoff", m_stateMachine.enableAutoHandoff());
    NamedCommands.registerCommand("Disable Auto Handoff", m_stateMachine.disableAutoHandoff());
    NamedCommands.registerCommand("Idle", m_stateMachine.idleSequence());
    NamedCommands.registerCommand(
        "Flip Heading", new InstantCommand(() -> m_robotDrive.flipHeading()));
    NamedCommands.registerCommand(
        "Set Align Right", new InstantCommand(() -> Limelight.setSIDE(Align.RIGHT)));
    NamedCommands.registerCommand(
        "Set Align Left", new InstantCommand(() -> Limelight.setSIDE(Align.LEFT)));
    NamedCommands.registerCommand(
        "Align to Coral Station",
        new AlignToCoralStation(m_robotDrive, m_backLimelight).withTimeout(1));
    NamedCommands.registerCommand(
        "Auto align",
        new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight).withTimeout(1.5));
    configureButtonBindings();

    // Configure default commands
    // COMMENT OUT BEFORE RUNNING SYSID
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    final int[] stalkNumbers = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    for (int i = 0; i < stalkNumbers.length; i++) {
      int number = stalkNumbers[i]; // Capture the number for the lambda
      new JoystickButton(m_leftController, i + 1) // i + initial button number
          .onTrue(
              new InstantCommand(
                  () -> {
                    SmartDashboard.putNumber("Reef Stalk Number", number);
                    if (number % 2 == 0) {
                      Limelight.setSIDE(Align.LEFT);
                    } else {
                      Limelight.setSIDE(Align.RIGHT);
                    }
                  }));
    }

    // Driver Controller Actions
    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .onTrue(m_stateMachine.intakeCoral());

    m_driverController.a().onTrue(m_stateMachine.scoreCoral()).onFalse(m_stateMachine.stopScore());

    m_driverController
        .rightBumper()
        .whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight));

    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(new AlignToCoralStation(m_robotDrive, m_backLimelight));

    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // Stages
    L1Button.onTrue(m_stateMachine.setLevel(ScoreLevel.L1));
    L2Button.onTrue(m_stateMachine.setLevel(ScoreLevel.L2));
    L3Button.onTrue(m_stateMachine.setLevel(ScoreLevel.L3));
    L4Button.onTrue(m_stateMachine.setLevel(ScoreLevel.L4));
    stowButton.onTrue(m_stateMachine.idle());

    handoffButton.onTrue(m_stateMachine.handoffManual());

    autoHandoffButton
        .onTrue(m_stateMachine.enableAutoHandoff())
        .onFalse(m_stateMachine.disableAutoHandoff());

    coralExtakeButton.onTrue(m_stateMachine.extakeCoral());
    climbDeployToggleButton
        .onTrue(m_stateMachine.deployClimber())
        .onFalse(m_stateMachine.retractClimber());
    sheeshButton.onTrue(m_stateMachine.climb());
    intakeOneCoralButton.onTrue(m_stateMachine.oneCoralBetweenIntake());
    removeAlgaeHighLevelButton.onTrue(m_stateMachine.removeAlgae(ScoreLevel.ALGAE_HIGH));
    removeAlgaeLowLevelButton.onTrue(m_stateMachine.removeAlgae(ScoreLevel.ALGAE_LOW));
    // setStartingConfigButton.onTrue(m_stateMachine.setAutonomousSetup());

    if (Robot.isSimulation()) {
      coralOnDragonButton
          .onTrue(new InstantCommand(() -> m_dragon.coralOnDragonTrue()))
          .onFalse(new InstantCommand(() -> m_dragon.coralOnDragonFalse()));
      loadCoralButton
          .onTrue(new InstantCommand(() -> m_coralIntake.setLoadedTrue()))
          .onFalse(new InstantCommand(() -> m_coralIntake.setLoadedFalse()));
    }

    overrideStateMachineButton
        .onTrue(m_stateMachine.enableManualOverride())
        .onFalse(m_stateMachine.disableManualOverride());

    rumble.onTrue(
        new StartEndCommand(
                () -> {
                  m_driverController.setRumble(RumbleType.kLeftRumble, 1.0);
                  m_driverController.setRumble(RumbleType.kRightRumble, 1.0);
                },
                () -> {
                  m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);
                  m_driverController.setRumble(RumbleType.kRightRumble, 0.0);
                })
            .withTimeout(0.5));
  }

  public Command setButtonStates() {
    return new InstantCommand(
            () -> {
              if (overrideStateMachineButton.getAsBoolean()) {
                m_stateMachine.enableManualOverride().schedule();
              } else {
                m_stateMachine.disableManualOverride().schedule();
              }
              if (autoHandoffButton.getAsBoolean()) {
                m_stateMachine.enableAutoHandoff().schedule();
              } else {
                m_stateMachine.disableAutoHandoff().schedule();
              }
              m_stateMachine.setTeleOpDefaultStates().schedule();
            })
        .ignoringDisable(true);
  }

  public Command setTeleOpDefaultStates() {
    return new InstantCommand(
        () -> {
          m_stateMachine.setTeleOpDefaultStates().schedule();
        });
  }

  public Command setAutonomousDefaultStates() {
    return new InstantCommand(
        () -> {
          m_robotDrive.flipHeading();
          m_stateMachine.setAutonomousDefaultStates().schedule();
        });
  }

  public Command homingSequence() {
    return m_stateMachine.homingSequence();
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
