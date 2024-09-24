package frc.robot.subsystems.Arm;


/**
 * Acts like a header file where important feedback data is stored and, 
 * functions/methods that are utilized by the Arm Subsystem.
 * @author Tony C.
 * 
 */
public interface ArmIO{

    public class ArmIOInputs {
        public double armPositionRads; 
        public double armEncoderPositionRads;
        // Constructor
        public ArmIOInputs() {
            armPositionRads = 0.0; 
            armEncoderPositionRads = 0.0; 
        } 
    }
        
    public void stop();

    public void setPositionControl(double positionRotations);

    public void setMotionControl(double positionRotations);
}
