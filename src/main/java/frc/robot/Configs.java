package frc.robot;


import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.DragonConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class Climber {
        public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();

        static {
        // Configure basic setting of the arm motor
                pivotConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true).voltageCompensation(12);
                pivotConfig.absoluteEncoder
                        .positionConversionFactor(1)
                        .inverted(true)
                        .zeroOffset(0); //tune later
                pivotConfig
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // Set PID values for position control. We don't need to pass a closed
                // loop slot, as it will default to slot 0.
                        .p(0.1)
                        .outputRange(-1, 1)
                        .maxMotion
                        .maxVelocity(4200)
                        .maxAcceleration(6000)
                        .allowedClosedLoopError(0.5);
        }
    }

    public static final class AlgaeSubsystem {
        public static final SparkFlexConfig rollerConfig = new SparkFlexConfig();
        public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();

        static {
        // Configure basic setting of the arm motor
                pivotConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).inverted(true).voltageCompensation(12);
                pivotConfig.absoluteEncoder
                        .positionConversionFactor(1)
                        .inverted(true)
                        .zeroOffset(0); //tune later
                pivotConfig
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // Set PID values for position control. We don't need to pass a closed
                // loop slot, as it will default to slot 0.
                        .p(0.1)
                        .outputRange(-1, 1)
                        .maxMotion
                        .maxVelocity(4200)
                        .maxAcceleration(6000)
                        .allowedClosedLoopError(0.5);

                rollerConfig.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
        }
    }

    public static final class CoralIntake {
                public static final SparkFlexConfig rollerConfig = new SparkFlexConfig();
                public static final SparkFlexConfig indexerConfig = new SparkFlexConfig();
                public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
                public static final SparkFlexConfig pivotFollowerConfig = new SparkFlexConfig();
        
                static {
                        // Configure basic setting of the arm motor
                        pivotConfig.smartCurrentLimit(40)
                                .idleMode(IdleMode.kBrake)
                                .inverted(true)
                                .voltageCompensation(12);
                        pivotConfig.absoluteEncoder
                                .positionConversionFactor(360 / CoralIntakeConstants.kPivotReduction)
                                .inverted(true)
                                .zeroOffset(0)
                                .zeroCentered(false); //tune later
                        pivotConfig
                                .closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // Set PID values for position control. We don't need to pass a closed
                        // loop slot, as it will default to slot 0.
                                .p(0.1)
                                .outputRange(-1, 1)
                                .maxMotion
                                .maxVelocity(4200)
                                .maxAcceleration(6000)
                                .allowedClosedLoopError(0.5);
                        
                        pivotFollowerConfig.smartCurrentLimit(40)
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .voltageCompensation(12)
                                .follow(CoralIntakeConstants.kPivotMotorCanId);
                
                        // Configure basic settings of the intake motor
                        rollerConfig
                                .inverted(false)
                                .idleMode(IdleMode.kCoast)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12);

                        // Configure indexer
                        indexerConfig
                                .inverted(false)
                                .smartCurrentLimit(40)
                                .idleMode(IdleMode.kBrake);

                }
   }

    public static final class Elevator {
        public static final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig elevatorFollowerConfig = new SparkFlexConfig();
        static {
                elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

                elevatorConfig
                        .limitSwitch
                        .reverseLimitSwitchEnabled(true)
                        .reverseLimitSwitchType(Type.kNormallyOpen);

                elevatorConfig
                        .limitSwitch
                        .forwardLimitSwitchEnabled(true)
                        .forwardLimitSwitchType(Type.kNormallyOpen);

                elevatorConfig
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        .p(.1)
                        .outputRange(-1, 1)
                        .maxMotion
                        // Set MAXMotion parameters for position control
                        .maxVelocity(4200)
                        .maxAcceleration(6000)
                        .allowedClosedLoopError(0.5);
                elevatorFollowerConfig
                        .follow(ElevatorConstants.kElevatorMotorCanId)
                        .inverted(true);
        }
    }

    public static final class Dragon {
        public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
        public static final SparkFlexConfig pivotRollerConfig = new SparkFlexConfig();

        static {

                pivotConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

                pivotConfig
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // Set PID values for position control
                        .p(.1)
                        .outputRange(-1, 1)
                        .maxMotion
                        //Set MAXMotion parameters for position control
                        .maxVelocity(4200)
                        .maxAcceleration(6000)
                        .allowedClosedLoopError(0.5);
                pivotConfig
                        .absoluteEncoder
                        // .positionConversionFactor(1)
                        .zeroOffset(0)
                        .inverted(true)
                        .positionConversionFactor(360 / DragonConstants.kPivotReduction);

                pivotRollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);


        }
    }
}
