// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmIO.ArmIOInputs;

public class ArmSubsystem extends SubsystemBase {

    // Decleration
    private final ArmIO io; 
    private ArmIOInputs inputs; 
    
    // Constuctor 
    public ArmSubsystem(ArmIO io) {
        this.io = io; 
        inputs = new ArmIOInputs(); 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Command moveArm( int angleRad ) {
        return runOnce(
            () -> {
                if ( Math.abs(angleRad - inputs.armPositionRads) < 0.1 ) {
                     if ( Math.abs(angleRad - inputs.armPositionRads ) < 0.01 ) {
                        this.io.setPositionControl(angleRad);
                     } else {
                        this.io.stop();
                     }
                } else {
                    this.io.setMotionControl(angleRad);
                }
            }
        );
    }


}
