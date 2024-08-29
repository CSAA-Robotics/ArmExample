package frc.robot.subsystems.Arm;

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
    
    public void updateInputs ( ArmIOInputs inputs );
    
    public void stop();

    public void setPositionControl(double positionRotations);

    public void setMotionControl(double positionRotations);
}
