import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.Field;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Dragon;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.State;

@Execution(ExecutionMode.SAME_THREAD)
public class StateMachineTests {
    private static CommandScheduler m_scheduler = CommandScheduler.getInstance();
    private static AlgaeIntake m_algaeIntake = new AlgaeIntake();
    private static CoralIntake m_coralIntake = new CoralIntake();
    private static Elevator m_elevator = new Elevator();
    private static Dragon m_dragon = new Dragon();
    private static Climber m_climber = new Climber();

    private static StateMachine m_stateMachine = new StateMachine(m_dragon, m_elevator, m_coralIntake, m_algaeIntake,
            m_climber);

    void setState(State state) {
        try {

            Field field = StateMachine.class.getDeclaredField("m_state");
            field.setAccessible(true);
            field.set(m_stateMachine, state);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        m_scheduler.cancelAll();
    }

    void runScheduler() {
        for (int i = 0; i < 20; i++) {
            m_scheduler.run();
            Timer.delay(0.020); // Simulate 20ms periodic
        }
    }

    void assertState(State state) {
        assertEquals(state, m_stateMachine.getState());
    }

    void assertState(State state, String message) {
        assertEquals(state, m_stateMachine.getState(), message);
    }

    void assertCommandHasNoEffect(State expectedState, Command... command) {
        for (Command c : command) {
            c.schedule();
            runScheduler();
            assertState(expectedState, c.getName() + " should have no effect while in " + expectedState.toString());
        }
    }

    @Test
    void idleToIntake() {
        setState(State.IDLE);
        m_coralIntake.setLoadedFalse();

        m_stateMachine.intakeCoral().schedule();
        runScheduler();
        assertState(State.INTAKE, "INTAKE should be reachable from IDLE");
    }

    @Test
    void idleInvalidTransitions() {
        setState(State.IDLE);
        assertCommandHasNoEffect(State.IDLE,
                m_stateMachine.extakeCoral(),
                m_stateMachine.setL1(),
                m_stateMachine.setL2(),
                m_stateMachine.setL3(),
                m_stateMachine.setL4(),
                m_stateMachine.scoreCoral());
    }

    @Test
    void intakeToIdle() {
        setState(State.INTAKE);
        m_coralIntake.setLoadedFalse();

        m_stateMachine.idle().schedule();
        runScheduler();
        assertState(State.IDLE, "IDLE should be reachable from INTAKE");
    }

    @Test
    void intakeToHandoff() {
        setState(State.IDLE);
        m_coralIntake.setLoadedFalse();
        m_stateMachine.setAutoHandoff(true);

        m_stateMachine.intakeCoral().schedule();
        runScheduler();
        m_coralIntake.setLoadedTrue();
        runScheduler();
        assertState(State.HANDOFF,
                "INTAKE should transition to HANDOFF after loading coral if auto handoff is enabled");
    }

    @Test
    void intakeToPoopStandby() {
        setState(State.IDLE);
        m_coralIntake.setLoadedFalse();
        m_stateMachine.setAutoHandoff(false);

        m_stateMachine.intakeCoral().schedule();
        runScheduler();
        m_coralIntake.setLoadedTrue();
        runScheduler();
        assertState(State.POOP_STANDBY,
                "INTAKE should transition to POOP_STANDBY after loading coral if auto handoff is disabled");
    }

    @Test
    void intakeInvalidTransitions() {
        setState(State.INTAKE);
        m_coralIntake.setLoadedFalse();
        assertCommandHasNoEffect(State.INTAKE,
                m_stateMachine.extakeCoral(),
                m_stateMachine.setL1(),
                m_stateMachine.setL2(),
                m_stateMachine.setL3(),
                m_stateMachine.setL4(),
                m_stateMachine.scoreCoral());
    }

    // TODO(jan): extake tests

    @Test
    void poopStandbyToPoopReady() {
        setState(State.POOP_STANDBY);
        m_coralIntake.setLoadedTrue();

        m_stateMachine.setL1().schedule();
        runScheduler();
        assertState(State.POOP_READY, "POOP_READY should be reachable from POOP_STANDBY via setL1()");
    }

    @Test
    void poopStandbyToHandoff() {
        setState(State.POOP_STANDBY);
        m_coralIntake.setLoadedTrue();

        m_stateMachine.setL2().schedule();
        runScheduler();
        assertState(State.HANDOFF, "HANDOFF should be reachable from POOP_STANDBY via setL2()");

        setState(State.POOP_STANDBY);
        m_stateMachine.setL3().schedule();
        runScheduler();
        assertState(State.HANDOFF, "HANDOFF should be reachable from POOP_STANDBY via setL3()");

        setState(State.POOP_STANDBY);
        m_stateMachine.setL4().schedule();
        runScheduler();
        assertState(State.HANDOFF, "HANDOFF should be reachable from POOP_STANDBY via setL4()");
    }

    @Test
    void poopStandbyToExtake() {
        setState(State.POOP_STANDBY);
        m_coralIntake.setLoadedTrue();

        m_stateMachine.extakeCoral().schedule();
        runScheduler();
        assertState(State.EXTAKE, "EXTAKE should be reachable from POOP_STANDBY via extakeCoral()");
    }

    @Test
    void poopStandbyInvalidTransitions() {
        setState(State.POOP_STANDBY);
        m_coralIntake.setLoadedTrue();
        assertCommandHasNoEffect(State.POOP_STANDBY,
                m_stateMachine.idle(),
                m_stateMachine.intakeCoral(),
                m_stateMachine.scoreCoral());
    }

    @Test
    void poopReadyToPoopStandby() {
        setState(State.POOP_READY);
        m_coralIntake.setLoadedTrue();

        m_stateMachine.idle().schedule();
        runScheduler();
        assertState(State.POOP_STANDBY,
                "POOP_STANDBY should be reachable from POOP_READY via idle() if coral is loaded");
    }

    @Test
    void poopReadyToHandoff() {
        setState(State.POOP_STANDBY);
        m_coralIntake.setLoadedTrue();

        m_stateMachine.setL2().schedule();
        runScheduler();
        assertState(State.HANDOFF, "HANDOFF should be reachable from POOP_STANDBY via setL2()");

        setState(State.POOP_STANDBY);
        m_stateMachine.setL3().schedule();
        runScheduler();
        assertState(State.HANDOFF, "HANDOFF should be reachable from POOP_STANDBY via setL3()");

        setState(State.POOP_STANDBY);
        m_stateMachine.setL4().schedule();
        runScheduler();
        assertState(State.HANDOFF, "HANDOFF should be reachable from POOP_STANDBY via setL4()");
    }

    @Test
    void poopReadyToPoopScore() {
        setState(State.POOP_READY);
        m_coralIntake.setLoadedTrue();

        m_stateMachine.scoreCoral().schedule();
        runScheduler();
        assertState(State.POOP_SCORE, "POOP_SCORE should be reachable from POOP_READY via scoreCoral()");
    }

    @Test
    void poopReadyToIdle() {
        setState(State.POOP_READY);
        m_coralIntake.setLoadedFalse();

        m_stateMachine.idle().schedule();
        runScheduler();
        assertState(State.IDLE, "IDLE should be reachable from POOP_READY via idle() if coral is not loaded");
    }

    @Test
    void poopReadyInvalidTransitions() {
        setState(State.POOP_READY);
        m_coralIntake.setLoadedTrue();
        assertCommandHasNoEffect(State.POOP_READY,
                m_stateMachine.intakeCoral(),
                m_stateMachine.extakeCoral(),
                m_stateMachine.setL1());
    }
}
