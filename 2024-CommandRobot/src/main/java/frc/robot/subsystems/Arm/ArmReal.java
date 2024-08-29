package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;

public class ArmReal implements ArmIO {
    // Decleration 
    public final TalonFX masterMotor; 
    public final TalonFX followerMotor; 
    public final CANcoder armEncoder; 

    // Configuration
    private final TalonFXConfiguration armConfig; 
    private final CANcoderConfiguration encoderConfig; 

    // Status Signals 
    private final StatusSignal<Double> armPositionRotation; 
    private final StatusSignal<Double> armEncoderPositionRotation; 

    // PID Controllers Optimization
    private final PositionVoltage pPos;       // Holding the Arm in Place
    private final MotionMagicVoltage pMnPos;  // Moving the Arm
    
    // Constructor 
    public ArmReal() {
        // Motor Defintions
        masterMotor = new TalonFX( ArmConstants.kMasterID );
        followerMotor = new TalonFX( ArmConstants.kFollowerID );
        // Encoder Definition
        armEncoder = new CANcoder( ArmConstants.kEncoder );
        // Motor Configuration 
        armConfig = new TalonFXConfiguration(); 
        armConfig.CurrentLimits.SupplyCurrentLimit = 50; // Check Breaker Panel 
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
        armConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5; 
        armConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5; 
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  // Sets direction to be Positive

        // Encoder Configuration
        encoderConfig = new CANcoderConfiguration(); 
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; 
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 

        // Applying Configuration to Motors
        masterMotor.getConfigurator().apply( armConfig );
        followerMotor.getConfigurator().apply( armConfig );

        // Applying Configuration to Encoder
        armEncoder.getConfigurator().apply( encoderConfig );

        // Creating a Control Group 
        followerMotor.setControl( new Follower( masterMotor.getDeviceID(), true ) );

        // Status Signal Defintions
        armPositionRotation = masterMotor.getPosition(); 
        armEncoderPositionRotation = armEncoder.getPosition(); 

        /* Speed up signals to an appropriate rate */
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            armPositionRotation, 
            armEncoderPositionRotation
        );

        // PID Controller Defintions
        pPos = new PositionVoltage(
            0, 
            0, 
            false, 
            0, 
            0, 
            false, 
            false, 
            false
        );
        pMnPos = new MotionMagicVoltage(
            0, 
            false, 
            0, 
            1, 
            false, 
            false, 
            false
        );
        // Modifying PID Values 

        // Hold the ARM (PID VALUES)
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot0.kG = 0.35; // to hold the arm weight
        armConfig.Slot0.kP = 60; // 100; // adjust PID
        armConfig.Slot0.kI = 0;
        armConfig.Slot0.kD = 0.02;
        armConfig.Slot0.kS = 0;
        armConfig.Slot0.kV = 0;
        armConfig.Slot0.kA = 0;

        // Move the arm (PID VALUES)
        armConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Slot1.kG = 0.35; // to hold the arm weight
        armConfig.Slot1.kP = 60; // 100; // adjust PID
        armConfig.Slot1.kI = 0;
        armConfig.Slot1.kD = 0;
        armConfig.Slot1.kS = 0;
        armConfig.Slot1.kV = 8; // 8.3; // move velocity
        armConfig.Slot1.kA = 0.2; // 0.2; // move accerleration
    }

    // Instance (ArmIO) Functions Defintion

    public void updateInputs( ArmIOInputs inputs ) {
        inputs.armPositionRads = Units.rotationsToRadians( armPositionRotation.getValue() );
        inputs.armEncoderPositionRads = Units.rotationsToRadians( armEncoderPositionRotation.getValue() );
        return; 
    }

    public void stop() {
        masterMotor.setControl( new NeutralOut() );
        return; 
    }

    public void setPositionControl( double positionRotations ) {
        masterMotor.setControl( pPos.withPosition( positionRotations ) );
        return; 
    }

    public void setMotionControl( double positionRotations ) {
        masterMotor.setControl( pMnPos.withPosition( positionRotations ) );
        return;  
    }
}
