package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Dragon;
import frc.robot.subsystems.Elevator;

public class Mech2dManager extends SubsystemBase {
    private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Superstructure Root", 24.8, 0);
    private final MechanismRoot2d m_coralRoot = m_mech2d.getRoot("Coral Root", 20, 0);

    // Elevator visualization
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
        new MechanismLigament2d(
            "Elevator",
            SimulationRobotConstants.kMinElevatorHeightMeters,
            90));

    // Dragon visualization
    private final MechanismLigament2d m_dragonMech2d = m_elevatorMech2d.append(
        new MechanismLigament2d(
            "Pivot",
            SimulationRobotConstants.kPivotLength * SimulationRobotConstants.kPixelsPerMeter,
            180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));

    // Coral intake visualization
    private final MechanismLigament2d coralStand =
    m_coralRoot.append(
      new MechanismLigament2d(
        "Coral Stand", SimulationRobotConstants.kCoralStandLength, 90)
    );
    private final MechanismLigament2d m_coralIntakeMech2d = coralStand.append(
        new MechanismLigament2d(
            "Coral Pivot", SimulationRobotConstants.kCoralIntakeLength,
            CoralIntakeConstants.PivotSetpoints.kZeroOffsetDegrees));
                // Method to set an offset for a MechanismLigament2d

    private final Elevator m_elevator;
    private final Dragon m_dragon;
    private final CoralIntake m_coral;

    public Mech2dManager(Elevator elevator, Dragon dragon, CoralIntake coral) {
        m_elevator = elevator;
        m_dragon = dragon;
        m_coral = coral;
        SmartDashboard.putData("Superstructure", m_mech2d);
    }

    @Override
    public void periodic() {
        // Update elevator visualization
        double elevatorHeight = SimulationRobotConstants.kMinElevatorHeightMeters
                + (m_elevator.getElevatorPosition() / SimulationRobotConstants.kElevatorGearing)
                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI);
        m_elevatorMech2d.setLength(elevatorHeight);

        // Update dragon visualization
        double dragonAngle =
            180
                - ( // mirror the angles so they display in the correct direction
                Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                    + Units.rotationsToDegrees(
                        m_dragon.getPivotPosition() / SimulationRobotConstants.kPivotReduction))
                - 180;
        m_dragonMech2d.setAngle(dragonAngle);

        // Update coral visualization
        double coralAngle = CoralIntakeConstants.PivotSetpoints.kZeroOffsetDegrees + m_coral.getPosition();
        m_coralIntakeMech2d.setAngle(coralAngle);

        // Debug values
        SmartDashboard.putNumber("Mech2D/Elevator Height", elevatorHeight);
        SmartDashboard.putNumber("Mech2D/Dragon Angle", dragonAngle);
        SmartDashboard.putNumber("Mech2D/Coral Angle", coralAngle);
    }
}