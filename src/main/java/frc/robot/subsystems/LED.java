package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED extends SubsystemBase {
    private Spark m_blinkin;

    public LED() {
        m_blinkin = new Spark(LEDConstants.kBlinkinPort); //TBD
    }

    public void set(double value) {
        m_blinkin.set(value);
    }

    public void setBlue() { //Sample Color/Pattern
        this.set(LEDConstants.kBlue); 
    }


}
