// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Configs;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.CoralIntakeConstants.PivotSetpoints;
import frc.robot.Constants.CoralIntakeConstants.RollerSetpoints;
import frc.robot.Constants.SimulationRobotConstants;

public class CoralIntake extends SubsystemBase {
  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.

  private enum CoralIntakeSetpoint {
    STOW,
    HANDOFF,
    INTAKE,
    EXTAKE,
    POOP
  }

  public enum CoralIntakeState
  {
    STOW,
    INTAKE_READY,
    INTAKE,
    EXTAKE_READY,
    EXTAKE,
    HANDOFF_READY,
    HANDOFF,
    POOP_READY,
    POOP_SCORE
  }

  private CoralIntakeSetpoint m_coralIntakeSetpoint;
  private CoralIntakeState m_coralIntakeState;
  

  private double pivotCurrentTarget = PivotSetpoints.kStow;

  private SparkFlex pivotMotor = new SparkFlex(CoralIntakeConstants.kPivotMotorCanId, MotorType.kBrushless);
  private SparkFlex pivotFollowerMotor = new SparkFlex(CoralIntakeConstants.kPivotFollowerMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private SparkFlex rollerMotor = new SparkFlex(CoralIntakeConstants.kRollerMotorCanId, MotorType.kBrushless);
  private SparkFlex indexerMotor = new SparkFlex(CoralIntakeConstants.kIndexerMotorCanId, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  private DigitalInput beamBreak = new DigitalInput(CoralIntakeConstants.kBeamBreakDioChannel);

  private double pivotReference = 0;

  // Simulation setup and variables
  private DCMotor armMotorModel = DCMotor.getNeoVortex(1);
  private SparkFlexSim armMotorSim;
  private final SingleJointedArmSim m_intakeSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kIntakeReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kIntakeLength, SimulationRobotConstants.kIntakeMass),
          SimulationRobotConstants.kCoralIntakeLength,
          SimulationRobotConstants.kCoralIntakeMinAngleRads,
          SimulationRobotConstants.kCoralIntakeMaxAngleRads,
          true,
          SimulationRobotConstants.kCoralIntakeMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d setup for subsytem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Coral Intake Root", 25, 25);
  private final MechanismLigament2d intakePivotMechanism =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Coral Pivot",
              SimulationRobotConstants.kCoralIntakeLength
                  * SimulationRobotConstants.kPixelsPerMeter,
                  CoralIntakeConstants.PivotSetpoints.kZeroOffsetDegrees));

  public CoralIntake() {
    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
  
    rollerMotor.configure(
        Configs.CoralIntake.rollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    indexerMotor.configure(
          Configs.CoralIntake.indexerConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    pivotMotor.configure(
        Configs.CoralIntake.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotFollowerMotor.configure(
      Configs.CoralIntake.pivotFollowerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    m_coralIntakeSetpoint = CoralIntakeSetpoint.STOW;
    m_coralIntakeState = CoralIntakeState.STOW;

    // Display mechanism2d
    SmartDashboard.putData("Coral Intake", m_mech2d);

    // Initialize Simulation values
    armMotorSim = new SparkFlexSim(pivotMotor, armMotorModel);
  }

  /** Set the arm motor position. This will use closed loop position control. */
  private void moveToSetpoint() {
    pivotController.setReference(pivotCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  private BooleanSupplier atSetpoint() {
    return () -> Math.abs(pivotCurrentTarget - pivotEncoder.getPosition()) <= CoralIntakeConstants.kPivotThreshold;
  }

  private Command setCoralIntakeSetpointCommand(CoralIntakeSetpoint state) {
    return new InstantCommand(() -> m_coralIntakeSetpoint = state);
  }

  private Command setCoralIntakeStateCommand(CoralIntakeState state) {
    return new InstantCommand(() -> m_coralIntakeState = state);
  }

  private Command setPivotCommand(CoralIntakeSetpoint setpoint) {
    return new SequentialCommandGroup(
      setCoralIntakeSetpointCommand(setpoint),
      new InstantCommand(
      () -> {
        switch (m_coralIntakeSetpoint) {
          case STOW:
            pivotCurrentTarget = PivotSetpoints.kStow;
            break;
          case HANDOFF:
            pivotCurrentTarget = PivotSetpoints.kHandoff;
            break;
          case INTAKE:
            pivotCurrentTarget = PivotSetpoints.kIntake;
            break;
          case EXTAKE:
            pivotCurrentTarget = PivotSetpoints.kExtake;
            break;
          case POOP:
            pivotCurrentTarget = PivotSetpoints.kEject;
            break;

        }}),
        new InstantCommand(() -> moveToSetpoint()),
        new WaitUntilCommand(atSetpoint())
      );
  }

  /** Set the pivot motor power in the range of [-1, 1]. */
  private void setRollerPower(double power) {
    rollerMotor.set(power);

    // TODO: Control this separately
    indexerMotor.set(power);
  }

  private Command setRollerPowerCommand(double power) {
    return new InstantCommand(() -> setRollerPower(power));
  }

  public Command intake() {
    return new ParallelCommandGroup(
      setPivotCommand(CoralIntakeSetpoint.INTAKE),
      setRollerPowerCommand(RollerSetpoints.kIntake)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.INTAKE));
  }

  public Command intakeReady() {
    return new ParallelCommandGroup(
        setPivotCommand(CoralIntakeSetpoint.INTAKE),
        setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.INTAKE_READY));
  }

  public Command extakeReady() {
    return new ParallelCommandGroup(
        setPivotCommand(CoralIntakeSetpoint.EXTAKE),
        setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.EXTAKE_READY));
  }

  public Command extake() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        setPivotCommand(CoralIntakeSetpoint.EXTAKE),
        setRollerPowerCommand(RollerSetpoints.kStop)
      ),
      setRollerPowerCommand(RollerSetpoints.kExtake)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.EXTAKE));
  }

  public Command stow() {
    return new ParallelCommandGroup(
      setPivotCommand(CoralIntakeSetpoint.STOW),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.STOW));
  }

  public Command handoffReady()
  {
    return new ParallelCommandGroup(
      setPivotCommand(CoralIntakeSetpoint.HANDOFF),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.HANDOFF_READY));
  }

  public Command handoff() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        setPivotCommand(CoralIntakeSetpoint.HANDOFF),
        setRollerPowerCommand(RollerSetpoints.kStop)
      ),
      setRollerPowerCommand(RollerSetpoints.kExtake)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.HANDOFF));
  }

  public Command poopReadyL1()
  {
    return new ParallelCommandGroup(
      setPivotCommand(CoralIntakeSetpoint.POOP),
      setRollerPowerCommand(RollerSetpoints.kStop)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.POOP_READY));
  }

  public Command poopL1() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        setPivotCommand(CoralIntakeSetpoint.POOP),
        setRollerPowerCommand(RollerSetpoints.kStop)
      ),
      setRollerPowerCommand(RollerSetpoints.kExtake)
    ).andThen(setCoralIntakeStateCommand(CoralIntakeState.POOP_SCORE));
  }


  public boolean isLoaded() {
    return !beamBreak.get();
  }

  public CoralIntakeSetpoint getSetpoint()
  {
    return m_coralIntakeSetpoint;
  }

  public CoralIntakeState getState()
  {
    return m_coralIntakeState;
  }


  /**
   * Command to run when the intake is not actively running. When in the "hold" state, the intake
   * will stay in the "hold" position and run the motor at its "hold" power to hold onto the ball.
   * When in the "stow" state, the intake will stow the arm in the "stow" position and stop the
   * motor.
   */

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("CoralIntake/Beam Break", beamBreak.get());

    // Display subsystem values
    SmartDashboard.putNumber("CoralIntake/Pivot/Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("CoralIntake/Roller/Applied Output", rollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("CoralIntake/Indexer/Applied Output", indexerMotor.getAppliedOutput());
    SmartDashboard.putNumber("CoralIntake/Pivot/Pivot setpoint", pivotReference);

    SmartDashboard.putString("Dragon State", m_coralIntakeSetpoint.toString());
    SmartDashboard.putBoolean("Coral Intake Pivot at Setpoint?", atSetpoint().getAsBoolean());

    // Update mechanism2d
    intakePivotMechanism.setAngle(CoralIntakeConstants.PivotSetpoints.kZeroOffsetDegrees + pivotEncoder.getPosition());
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_intakeSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_intakeSim.setInput(armMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_intakeSim.update(0.020);

    // Iterate the arm SPARK simulation
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_intakeSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java
  }
}