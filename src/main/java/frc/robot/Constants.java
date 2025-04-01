// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// ! Update Values for AlphaBot
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.92;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.491162);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.491162);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 4;

    public static final boolean kGyroReversed = false;

    public static final double kTurningEncoderPositionFactor = 2 * Math.PI;
    public static final double kDriveEncoderVelocityFactor =
        ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;

    public static final SimpleMotorFeedforward kDriveFeedforward =
        new SimpleMotorFeedforward(0.22022, 2.4995, 0.26113);
    public static final SimpleMotorFeedforward kTurningFeedforward =
        new SimpleMotorFeedforward(0, 0, 0.1);
  }

  public static final class ClimberConstants {
    public static final int kPivotMotorCanId = 51;

    public static final double kPivotThreshold = 50; // 81:1 gear ratio
    public static final double kPivotReduction = 1;

    public static final double kDeploySpeed = 1;
    public static final double kRetractSpeed = -1;
    public static final double kClimbSetpoint = 2;

    public static final class PivotSetpoints {
      public static final double kDeploy = 307.6;
      public static final double kRetract = 0;

      public static final double kMaxAngle = 315;
      public static final double kMinAngle = 0;
    }
  }

  public static final class AlgaeIntakeConstants {
    public static final int kRollerMotorCanId = 9;
    public static final int kPivotMotorCanId = 10;

    public static final double kPivotThreshold = 5; // TODO

    public static final double kPivotMinAngle = 0; // TODO
    public static final double kPivotMaxAngle = 0; // TODO

    public static final class PivotSetpoints {
      public static final double kStow = 15; // TODO
      public static final double kIntake = 0; // TODO
      public static final double kExtake = 0; // TODO
      public static final double kScore = 7; // TODO
      public static final double kClimb = 3; // TODO
    }

    public static final class RollerSetpoints {
      public static final double kIntake = 0.5; // TODO
      public static final double kExtake = -0.5; // TODO
      public static final double kStop = 0; // TODO
    }
  }

  public static final class CoralIntakeConstants {
    public static final int kRollerMotorCanId = 11;
    public static final int kPivotMotorCanId = 12;
    public static final int kIndexerMotorCanId = 13;

    public static final double kPivotThreshold = 5; // TODO

    public static final int kBeamBreakDioChannel = 0;
    public static final double kP = 0.06;
    public static final double kG = 0.2;

    public static final double kPivotReduction = 1;

    public static final class PivotSetpoints {
      // Zero offset in Hardware Client is 10
      public static final double kStow = 10.5;
      public static final double kIntake = 69;
      public static final double kOneCoralInBetweenIntake = 87.2;
      public static final double kExtake = 90;
      public static final double kHandoff = 77;
      public static final double kPoop = 80;

      public static final double kZeroOffsetDegrees = 270;
      public static final double kClimb = 100;
    }

    public static final class RollerSetpoints {
      public static final double kIntake = 0.6;
      public static final double kExtake = -0.4;
      public static final double kStop = 0;
      public static final double kPrePoop = -0.2;
    }
  }

  public static final class SimulationRobotConstants {
    // ! Remove *20 from all values for final Robot Testing
    public static final double kPixelsPerMeter = 1 * 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = Units.inchesToMeters(35.2 * 20); // m
    public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(66.7 * 20); // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg

    public static final double kPivotReduction = 60; // 60:1
    public static final double kPivotLength = Units.inchesToMeters(10); // m
    public static final double kPivotMass = 4.3; // Kg

    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = Units.inchesToMeters(15); // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);

    public static final double kCoralIntakeMinAngleRads = Units.degreesToRadians(0);
    public static final double kCoralIntakeMaxAngleRads =
        Units.degreesToRadians(360 * kIntakeReduction);

    public static final double kIntakeShortBarLength = Units.inchesToMeters(11);
    public static final double kIntakeLongBarLength = Units.inchesToMeters(13);
    public static final double kCoralIntakeLength = Units.inchesToMeters(11 * 20);
    public static final double kCoralStandLength = Units.inchesToMeters(35.2 * 20);
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(90);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps =
        NeoVortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.038 * 2;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kTurningMotorReduction = 64 / 14;
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;

    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class NeoVortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  // ! Update Values for AlphaBot
  public static final class LimelightConstants {
    public static final Matrix<N3, N1> m_stateStdDevs =
        VecBuilder.fill(0.15, 0.15, 0.00001); // TODO: needs tuning
    public static final Matrix<N3, N1> m_visionStdDevs =
        VecBuilder.fill(0.00001, 0.00001, 999999); // TODO: needs
    // tuning
    public static final String kRightLimelightName = "limelight-right";
    public static final String kLeftLimelightName = "limelight-left";
    public static final String kBackLimelightName = "limelight-back";

    public static final double kRightCameraHeight = 19.942862; // 0.5065486948 meters
    public static final double kLeftCameraHeight = 19.942862; // 0.5065486948 meters

    // Parallel to elevator
    public static final double kRightCameraXOffset = 12.153079; // 0.3086882066 meters
    public static final double kLeftCameraXOffset = -12.153079; // -0.3086882066 meters
    public static final double kBackCameraXOffset = 0; // -0.3086882066 meters

    // Perpendicular to elevator
    public static final double kRightCameraYOffset = 11.940763; // 0.3032953802 meters
    public static final double kLeftCameraYOffset = 11.940763; // 0.3032953802 meters
    public static final double kBackCameraYOffset = -3.086657; // -0.3086882066 meters

    public static final double kBackCameraHeight = 38.868062; // TODO

    public static final double kRightMountingPitch = -45;
    public static final double kLeftMountingPitch = -45;

    public static final double kRightMountingYaw = -24.499987;
    public static final double kLeftMountingYaw = 180 - 24.499987;

    public static final double kBackMountingPitch = 20; // TODO

    public static final double kBackMountingYaw = 0; // TODO

    public static final double kReefTagHeight = 12;
    public static final double kProcessorTagHeight = 0; // tune later
    public static final double kCoralStationTagHeight = 53.25; // tune later
    public static final int kProcessorPipeline = 0; // TBD
    public static final int kRightReefBranchPipeline = 1;
    public static final int kLeftReefBranchPipeline = 2;
    public static final int kRedPosePipeline = 3; // TBD
    public static final int kBluePosePipeline = 4; // TBD
    public static final int kCoralStationPipeline = 5;

    public static final double kCoralStationDistanceThreshold = 0; // TODO: tune
  }

  // ! Update Values for AlphaBot
  public static final class ElevatorConstants {

    public static final int kElevatorMotorCanId = 14;
    public static final int kElevatorFollowerCanId = 15;

    public static final double kSetpointThreshold = 0.1;

    public static final double kMinLimit = 0;
    public static final double kMaxLimit = 9.62;

    public static final double kP = .3;
    public static final double kG = .3;

    public static final double kClearToStowDragonLevel = 9.62;

    public static final class ElevatorLevels {
      public static final double kStow = 0;
      public static final double kPoop = 0;
      public static final double kHandoff = 0;

      public static final double kLevel1 = 0;
      public static final double kLevel2 = 0;
      public static final double kLevel3 = 0;
      public static final double kLevel4 = 9.62;
      public static final double kAlgaeLow = 0;
      public static final double kAlgaeHigh = 4.89;
    }
  }

  public static final class DragonConstants {

    public static final int kPivotMotorCanId = 16;
    public static final int kPivotRollerMotorCanId = 17;
    public static final double kPivotReduction = 1;
    public static final double kPivotThreshold = 5; // TODO: tune
    public static final double kRollerCurrentThreshold = 5; // tune

    public static final double kPivotMinAngle = 10.0;
    public static final double kPivotMaxAngle = 250.0;
    public static final double kClearFromElevatorAngle = 165.5;
    public static final double kClearFromReefAngle = 50;
    public static final double kClearToScoreL4Angle = 70.7;

    public static final double kP = 0.015;
    public static final double kG = -0.25;

    public static final class LaserCanConstants {
      // TODO(jan): Tune
      public static final Distance kL4MinDetectionDistance = Millimeters.of(135);
      public static final Distance kL4MaxDetectionDistance = Millimeters.of(200);
    }

    public static final class PivotSetpoints {
      // Zero offset in Hardware Client is 10
      public static final double kStartingConfig = 245;
      public static final double kStow = 27.25;
      public static final double kHandoff = 222; // fix this later
      public static final double kLevel1 = 47.13;
      public static final double kLevel2 = 100.3;
      public static final double kLevel3 = 35;
      public static final double kLevel4 = 101.7;
      public static final double kClimb = 190;
      public static final double kAlgaeHigh = 89.4; // TODO: fix this later
      public static final double kAlgaeLow = 89.4; // TODO: fix this later
      public static final double kAlgaeReady = 220;
      public static final double kRetract = 49.3;
    }

    public static final class RollerSetpoints {
      public static final double kIntake = -0.1; // TODO: tune (pos when getting from station)
      public static final double kExtake = .7; // TODO: tune ts (neg when scoring)
      public static final double kStop = 0;
      public static final double kHold = -0.2;
    }
  }

  public static final class LEDConstants {
    public static final int kBlinkinPort = 2;

    public static final double kBlue = 0.87;
    public static final double kGreen = 0.77;
    public static final double kWhite = 0.93;
    public static final double kYellow = 0.69;
    public static final double kHeartbeatRed = -0.25;
    public static final double kViolet = 0.91;
    public static final double kOrange = 0.65;
  }
}
;
