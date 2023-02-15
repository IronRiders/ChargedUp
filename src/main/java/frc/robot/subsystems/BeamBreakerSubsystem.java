package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class BeamBreakerSubsystem {
    private DigitalInput beamBreakerSensor;

    public BeamBreakerSubsystem() {
        beamBreakerSensor = new DigitalInput(Constants.BEAM_BREAKER_SENSOR);
    }

    public boolean isBeamBroken() {
        return beamBreakerSensor.get();
    }
}