// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmIO.ArmIOInputs;
/**
 * The subsystem class that is responsible for the Arm
 */
public class ArmSubsystem extends SubsystemBase {

    // Decleration
    private final ArmIO io; 
    private ArmIOInputs inputs; 
    
    // Restriction Variables
    private final double first_threshold_positionRads = 0.1; 
    private final double second_threshold_positionRads = 0.01; 
    
    /**
     * Associates the Arm Input/Output object with the Arm Subsystem
     * @param io The Arm INPUT/OUTPUT object
     */
    public ArmSubsystem(ArmIO io) {
        this.io = io; 
        inputs = new ArmIOInputs(); 
    }
    
    /**
     * Method that will be called once per scheduler run. 
     */
    @Override
    public void periodic() {
        // Process Inputs
        io.updateInputs( inputs );
        
        double desired_positionRads = 12; 
        double current_positionRads = inputs.armEncoderPositionRads;
        double difference_positionRads = Math.abs( desired_positionRads - current_positionRads );

        /**
         * If the difference between the desired and current is: 
         * Large ( diff > first_threshold ): Uses MotionMagic Voltage 
         * Small ( diff <= first_threshold ): Checks Difference with new Threshold
         * 
         * If the difference type is Small, it considers the difference again:
         * Large ( diff > second_threshold ): Uses Position Voltage
         * Small ( diff <= second_threshold ): Motors Stop
         */
        if ( difference_positionRads < first_threshold_positionRads ) {
            // Second Threshold Checkpoint
            if ( difference_positionRads < second_threshold_positionRads ) {
                io.stop(); 
            }  
            else {
                io.setPositionControl( Units.radiansToRotations( desired_positionRads ) );
            }
        } 
        else {
            io.setMotionControl( Units.radiansToRotations( desired_positionRads ) );
        }
    }
}
