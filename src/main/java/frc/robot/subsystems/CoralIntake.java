// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.SimulationRobotConstants;

public class CoralIntake extends SubsystemBase {
  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.

  private SparkFlex pivotMotor = new SparkFlex(CoralIntakeConstants.kPivotMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private SparkFlex topRollerMotor = new SparkFlex(CoralIntakeConstants.kTopRollerMotorCanId, MotorType.kBrushless);
  private SparkFlex bottomRollerMotor = new SparkFlex(CoralIntakeConstants.kBottomRollerMotorCanId, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

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
  
    topRollerMotor.configure(
        Configs.CoralIntake.topRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    bottomRollerMotor.configure(
          Configs.CoralIntake.bottomRollerConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    pivotMotor.configure(
        Configs.CoralIntake.pivotConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Display mechanism2d
    SmartDashboard.putData("Coral Intake", m_mech2d);

    // Initialize Simulation values
    armMotorSim = new SparkFlexSim(pivotMotor, armMotorModel);
  }



  public Command intakeCommand() {
    return this.run(
        () -> {
          setRollerPower(CoralIntakeConstants.RollerSetpoints.kForward);
          setPivotPosition(CoralIntakeConstants.PivotSetpoints.kIntakeShelf);
        });
  }

  public Command extakeCommand() {
    return this.run(
        () -> {
          setRollerPower(CoralIntakeConstants.RollerSetpoints.kReverse);
          setPivotPosition(CoralIntakeConstants.PivotSetpoints.kExtakeFloor);
        });
  }

  public Command stowCommand() {
    return this.run(
      () -> {
        setRollerPower(CoralIntakeConstants.RollerSetpoints.kStop);
        setPivotPosition(CoralIntakeConstants.PivotSetpoints.kStow);
      });
  }

  public Command handoffCommand()
  {
    return this.run(
      () -> {
        setRollerPower(CoralIntakeConstants.RollerSetpoints.kReverse);
        setPivotPosition(CoralIntakeConstants.PivotSetpoints.kHandoff);
      });
  }


  /**
   * Command to run when the intake is not actively running. When in the "hold" state, the intake
   * will stay in the "hold" position and run the motor at its "hold" power to hold onto the ball.
   * When in the "stow" state, the intake will stow the arm in the "stow" position and stop the
   * motor.
   */

  /** Set the pivot motor power in the range of [-1, 1]. */
  private void setRollerPower(double power) {
    topRollerMotor.set(power);
  }

  /** Set the arm motor position. This will use closed loop position control. */
  private void setPivotPosition(double position) {
    pivotReference = position;
    pivotController.setReference(position, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void periodic() {

    // Display subsystem values
    SmartDashboard.putNumber("CoralIntake/Pivot/Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("CoralIntake/Applied Output", topRollerMotor.getAppliedOutput());
    SmartDashboard.putNumber("CoralIntake/Pivot/Pivot setpoint", pivotReference);

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