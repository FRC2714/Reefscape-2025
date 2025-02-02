// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.subsystems.Limelight.Align;
import frc.robot.subsystems.superstructure.StateMachine;
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
    m_algaeIntake, m_coralIntake, m_dragon, m_elevator, m_blinkin, m_leftLimelight, m_rightLimelight, m_backLimelight
  );

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
      .onTrue(m_algaeIntake.moveToScore())
      .onFalse(m_algaeIntake.moveToStow());

    m_driverController
      .rightTrigger(OIConstants.kTriggerButtonThreshold)
      .onTrue(m_algaeIntake.moveToIntake())
      .onFalse(m_algaeIntake.moveToStow());


    m_driverController.leftBumper()
      .onTrue(m_stateMachine.scoreCoral());

    // Force Actions
    m_driverController.povLeft()
      .whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, Align.LEFT));

    m_driverController.povRight()
      .whileTrue(new AlignToCoral(m_robotDrive, m_rightLimelight, m_leftLimelight, Align.RIGHT));
  // TODO: add pov up down for coral station and processor
    // Additional
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    
    // Stages
    L1Button.onTrue(m_stateMachine.elevator_moveToL1());
    L2Button.onTrue(m_stateMachine.elevator_moveToL2()); 
    L3Button.onTrue(m_stateMachine.elevator_moveToL3());
    L4Button.onTrue(m_stateMachine.elevator_moveToL4());
    stowButton.onTrue(m_stateMachine.elevator_moveToStow());
    handoffButton.onTrue(m_stateMachine.handoffCoral());
    coralStationButton.onTrue(m_coralIntake.moveToIntake());
  }

  public void setTeleOpDefaultStates() {
    m_algaeIntake.moveToStow().schedule();
    m_stateMachine.elevator_moveToStow().schedule();
    m_coralIntake.moveToStow().schedule();
    m_blinkin.setOrange(); //default lights are orange
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
