package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


//current LED status lights
//flashing red: process of aligning
//green: finished alinging
//yellow: has a valid target

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

    public void setGreen()
    {
        this.set(LEDConstants.kGreen);
    }
    
    public void setWhite()
    {
        this.set(LEDConstants.kWhite);
    }

    public void setYellow()
    {
        this.set(LEDConstants.kYellow);
    }

    public void setHeartBeatRed()
    {
        this.set(LEDConstants.kHeartbeatRed);
    }

    public void setViolet()
    {
        this.set(LEDConstants.kViolet);
    }

    public void setOrange()
    {
        this.set(LEDConstants.kOrange);
    }



}
