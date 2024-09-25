/**
 * @file ArmIO.java
 * @brief Header file that initializes methods as well as stores data for tracking
 */

package frc.robot.subsystems.Arm;

/**
 * Acts like a header file where important feedback data is stored and, 
 * functions/methods that are utilized by the Arm Subsystem.
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
    public void updateInputs( ArmIOInputs inputs ); 
        
    public void stop();

    public void setPositionControl(double positionRotations);

    public void setMotionControl(double positionRotations);
}
