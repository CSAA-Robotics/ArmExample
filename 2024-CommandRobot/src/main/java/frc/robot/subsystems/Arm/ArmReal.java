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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Arm.ArmConstants;

public class ArmReal implements ArmIO {
    // Motor and Encoder Decleration
    public final TalonFX masterMotor; 
    public final TalonFX followerMotor; 
    public final CANcoder armEncoder; 

    // Motor and Encoder Configuration Settings
    private final TalonFXConfiguration armConfig; 
    private final CANcoderConfiguration encoderConfig; 

    // Status Signals 
    private final StatusSignal<Double> armPositionRotation; 
    private final StatusSignal<Double> armEncoderPositionRotation; 

    // PID Controllers Optimization
    private final PositionVoltage pPos;       // Holding the Arm in Place
    private final MotionMagicVoltage pMnPos;  // Moving the Arm
    
    /**
     * Constructor for Defining and Initializing Motors and other items responsible for the Arm
     */
    public ArmReal() {
        // Motor and Encoder Definition
        masterMotor = new TalonFX( ArmConstants.ARM_MASTER_ID ); 
        followerMotor = new TalonFX( ArmConstants.ARM_FOLLOWER_ID ); 
        armEncoder = new CANcoder( ArmConstants.ARM_ENCODER_ID ); 
        // Setting the Follower Motor to follow the Master Motor 
        followerMotor.setControl( new Follower( ArmConstants.ARM_MASTER_ID, true ) ); 

        // Arm Encoder Configuration Settings Definition
        armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;   // Clockwise position as Positive
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Set to Brake Mode
        // Configure the feedback sensor to use the CANcoder
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.ARM_ENCODER_ID;

        // Applying Motor Configuration to the Motors
        masterMotor.getConfigurator().apply( armConfig ); // For Master Motor
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        followerMotor.getConfigurator().apply( armConfig ); // For Follower Motor


    }
}
