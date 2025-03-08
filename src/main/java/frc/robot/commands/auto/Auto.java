package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LimelightConstants;
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

public class Auto {
  public static Auto m_autonomousInstance;
  private final SendableChooser<Command> autoChooser;

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

  public void setupNamedCommands() {
    NamedCommands.registerCommand(
        "Dragon standby", m_stateMachine.dragonStandbySequence().withTimeout(0.3));
    NamedCommands.registerCommand(
        "Score Coral", m_stateMachine.dragonScoreSequence().withTimeout(1));
    NamedCommands.registerCommand("L4", m_stateMachine.scoreReadySequence(ScoreLevel.L4));
    NamedCommands.registerCommand("L3", m_stateMachine.scoreReadySequence(ScoreLevel.L3));
    NamedCommands.registerCommand("L2", m_stateMachine.scoreReadySequence(ScoreLevel.L2));
    NamedCommands.registerCommand("L1", m_stateMachine.scoreReadySequence(ScoreLevel.L1));
    NamedCommands.registerCommand("Intake Coral", m_stateMachine.intakeSequence());
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
  }

  public void configureAutoBuilder() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      throw new RuntimeException(e);
    }
    AutoBuilder.configure(
        m_robotDrive::getEstimatedPose, // Robot pose supplier
        m_robotDrive
            ::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        m_robotDrive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            m_robotDrive.driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual
        // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic
            // drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        m_robotDrive // Reference to this subsystem to set requirements
        );
  }

  public Auto() {
    setupNamedCommands();
    configureAutoBuilder();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("New Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    return autoChooser.getSelected();
  }
}
