// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToCoral;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Dragon;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Limelight.Align;
import frc.robot.subsystems.LED;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final AlgaeIntake m_algaeIntake = new AlgaeIntake();
  private final CoralIntake m_coralIntake = new CoralIntake();
  private final Elevator m_elevator = new Elevator();
  private final Dragon m_dragon = new Dragon();
  private final Climber m_climber = new Climber();

  private final LED m_blinkin = new LED();
  private final Limelight m_rightLimelight = new Limelight(
      LimelightConstants.kRightLimelightName,
      LimelightConstants.kRightCameraHeight,
      LimelightConstants.kRightMountingAngle,
      LimelightConstants.kReefTagHeight);
  private final Limelight m_leftLimelight = new Limelight(
      LimelightConstants.kLeftLimelightName,
      LimelightConstants.kLeftCameraHeight,
      LimelightConstants.kLeftMountingAngle,
      LimelightConstants.kReefTagHeight);

  private final Limelight m_backLimelight = new Limelight(LimelightConstants.kBackLimelightName,
      LimelightConstants.kBackCameraHeight,
      LimelightConstants.kBackMountingAngle,
      LimelightConstants.kProcessorTagHeight);

  private final StateMachine m_stateMachine = new StateMachine(
      m_dragon, m_elevator, m_coralIntake, m_algaeIntake, m_climber);

  // Mech2d Stuff
  private final Mech2dManager m_mech2dManager = new Mech2dManager(m_elevator, m_dragon, m_coralIntake, m_algaeIntake);

  public Mech2dManager getMech2dManager() {
    return m_mech2dManager;
  }

  Joystick m_reefController = new Joystick(1); // operator controller 1
  Joystick m_mechanismController = new Joystick(2); // operator controller 2
  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // Operator Controller
  private final JoystickButton L1Button = new JoystickButton(m_mechanismController, 1); // L1
  private final JoystickButton L2Button = new JoystickButton(m_mechanismController, 2); // L2
  private final JoystickButton L3Button = new JoystickButton(m_mechanismController, 3); // L3
  private final JoystickButton L4Button = new JoystickButton(m_mechanismController, 4); // L4
  private final JoystickButton coralIntakeButton = new JoystickButton(m_mechanismController, 8); // Coral Station
  private final JoystickButton coralExtakeButton = new JoystickButton(m_mechanismController, 7);
  private final JoystickButton manualOverrideButton = new JoystickButton(m_mechanismController, 12); // L4
  private final JoystickButton handoffButton = new JoystickButton(m_mechanismController, 6);
  private final JoystickButton stowButton = new JoystickButton(m_mechanismController, 5); // Stow
  private final JoystickButton loadCoralButton = new JoystickButton(m_mechanismController, 10); // Stow
  private final JoystickButton coralonDragonButton = new JoystickButton(m_mechanismController, 9); // Stow
  private final JoystickButton climbButton = new JoystickButton(m_mechanismController, 1);

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

    NamedCommands.registerCommand("Score Coral", m_stateMachine.scoreCoral());
    NamedCommands.registerCommand("Intake Algae",
        m_stateMachine.intakeAlgae().withTimeout(1).andThen(m_stateMachine.stowAlgae()));
    NamedCommands.registerCommand("Extake Algae",
        m_stateMachine.extakeAlgae().withTimeout(1).andThen(m_stateMachine.stowAlgae()));
    NamedCommands.registerCommand("L4", m_stateMachine.setL4());
    NamedCommands.registerCommand("L3", m_stateMachine.setL3());
    NamedCommands.registerCommand("L2", m_stateMachine.setL2());
    NamedCommands.registerCommand("L1", m_stateMachine.setL1());
    NamedCommands.registerCommand("Intake Coral", m_stateMachine.intakeCoral());
    NamedCommands.registerCommand("Extake Coral", m_stateMachine.extakeCoral());

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

    // Driver Controller Actions
    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .onTrue(m_stateMachine.extakeAlgae())
        .onFalse(m_stateMachine.stowAlgae());

    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .onTrue(m_stateMachine.intakeAlgae())
        .onFalse(m_stateMachine.stowAlgae());

    m_driverController.leftBumper()
        .onTrue(m_stateMachine.scoreCoral());

    // // Force Actions
    m_driverController.povLeft()
        .whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, Align.LEFT));

    m_driverController.povRight()
        .whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, Align.RIGHT));
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // // Stages
    L1Button.onTrue(m_stateMachine.setL1());
    L2Button.onTrue(m_stateMachine.setL2());
    L3Button.onTrue(m_stateMachine.setL3());
    L4Button.onTrue(m_stateMachine.setL4());
    stowButton.onTrue(m_stateMachine.idle());

    handoffButton.onTrue(m_stateMachine.handoffManual());

    coralIntakeButton.onTrue(m_stateMachine.intakeCoral());
    coralExtakeButton.onTrue(m_stateMachine.extakeCoral());
    climbButton.onTrue(m_stateMachine.deployClimber()).onFalse(m_stateMachine.retractClimber());

    // L4Button.onTrue(m_stateMachine.deployClimber());
    // L3Button.onTrue(m_stateMachine.retractClimber());
    // //L2Button.onTrue(m_stateMachine.stowClimber());

    // Reef Branches for HUD
    int[] stalkNumbers = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

    for (int i = 0; i < stalkNumbers.length; i++) {
      final int number = stalkNumbers[i]; // Capture the number for the lambda
      new JoystickButton(m_reefController, i + 1) // i + initial button number
          .onTrue(new InstantCommand(() -> {
            SmartDashboard.putNumber("Reef Stalk Number", number);
          }));

      if (Robot.isSimulation()) {
        coralonDragonButton.onTrue(new InstantCommand(() -> m_dragon.coralOnDragonTrue()))
            .onFalse(new InstantCommand(() -> m_dragon.coralonDragonFalse()));
        loadCoralButton.onTrue(new InstantCommand(() -> m_coralIntake.setLoadedTrue()))
            .onFalse(new InstantCommand(() -> m_coralIntake.setLoadedFalse()));
      }

      manualOverrideButton.onTrue(m_stateMachine.enableManualOverride())
          .onFalse(m_stateMachine.disableManualOverride());
    }
  }

  public void setTeleOpDefaultStates() {
    m_stateMachine.setDefaultStates().schedule();
    m_blinkin.setOrange(); // default lights are orange
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
