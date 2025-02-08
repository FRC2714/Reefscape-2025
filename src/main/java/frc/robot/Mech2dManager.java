// Mech2dManager.java
package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SimulationRobotConstants;
import frc.robot.subsystems.Dragon;
import frc.robot.subsystems.Elevator;

public class Mech2dManager extends SubsystemBase {
    // The overall mechanism
    private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Superstructure Root", 24.8, 0);

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

    // References to subsystems
    private final Elevator m_elevator;
    private final Dragon m_dragon;

    public Mech2dManager(Elevator elevator, Dragon dragon) {
        m_elevator = elevator;
        m_dragon = dragon;

        // Put the mechanism on the dashboard
        SmartDashboard.putData("Superstructure", m_mech2d);
    }

    @Override
    public void periodic() {
        // Update elevator visualization
        m_elevatorMech2d.setLength(
            SimulationRobotConstants.kMinElevatorHeightMeters
            + (m_elevator.getElevatorPosition() / SimulationRobotConstants.kElevatorGearing)
            * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));

        // Update dragon visualization
        m_dragonMech2d.setAngle(
            180
            - (
                Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    m_dragon.getPivotPosition() / SimulationRobotConstants.kPivotReduction))
            - 0
        );
    }
}