/**
 * @file ArmReal.java
 * @brief All the devices and their methods are defined here
 */

package frc.robot.subsystems.Arm;

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

/** This class is where all the devices and defined and configured and all the necessary functions
 * needed to maintain/run the Arm Subsystem Class. In essense, it contains all the functions/methods for
 * the Arm Subsystem class.
 */
public class ArmReal implements ArmIO {
    // Motor and Encoder Decleration
    public final TalonFX masterMotor; 
    public final TalonFX followerMotor; 
    public final CANcoder armEncoder; 

    // Motor and Encoder Configuration Settings
    private final TalonFXConfiguration armConfig; 
    private final CANcoderConfiguration encoderConfig; 

    // PID Controllers Optimization
    PositionVoltage pPos = new PositionVoltage( 0, 0, false, 0, 0, false , false, false );       // Holding the Arm in Place
    MotionMagicVoltage pMnPos = new MotionMagicVoltage( 0, false, 0, 1, false, false, false );  // Moving the Arm
    
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

        // Arm Motor Configuration Settings Definition
        armConfig = new TalonFXConfiguration();
        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;   // Clockwise position as Positive
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Set to Brake Mode
        // Configure the feedback sensor to use the CANcoder
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armConfig.Feedback.FeedbackRemoteSensorID = ArmConstants.ARM_ENCODER_ID;

        // PID Configurations - Holding the Arm
        armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Accounting for Gravity
        armConfig.Slot0.kG = 0.35; // Gravity Feedfroward/Feeback Gain
        armConfig.Slot0.kP = 60; // Proportional Gain
        armConfig.Slot0.kI = 0; // Intergal Gain
        armConfig.Slot0.kD = 0.02; // Derivative Gain
        armConfig.Slot0.kS = 0; // Static Feedforward Gain
        armConfig.Slot0.kV = 0; // Velocity Feedforward Gain
        armConfig.Slot0.kA = 0; // Acceleration Feedforward Gain

        // PID Configurations - Moving the Arm
        armConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine; // Accounting for Gravity
        armConfig.Slot1.kG = 0.35; // Gravity Feedfroward/Feeback Gain
        armConfig.Slot1.kP = 60; // Proportional Gain
        armConfig.Slot1.kI = 0; // Intergal Gain
        armConfig.Slot1.kD = 0; // Derivative Gain
        armConfig.Slot1.kS = 0; // Static Feedforward Gain
        armConfig.Slot1.kV = 8; // Velocity Feedforward Gain
        armConfig.Slot1.kA = 0.2; // Acceleration Feedforward Gain

        // Motion Magic Configurations
        armConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; 
        armConfig.MotionMagic.MotionMagicAcceleration = 2; 
        armConfig.MotionMagic.MotionMagicJerk = 10; 

        // Applying Motor Configuration to the Motors
        masterMotor.getConfigurator().apply( armConfig ); // For Master Motor
        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        followerMotor.getConfigurator().apply( armConfig ); // For Follower Motor

        // Arm Encoder Configuration
        encoderConfig = new CANcoderConfiguration(); 
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // Specify the Range
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // Same as Master Motor
        encoderConfig.MagnetSensor.MagnetOffset = 0.0; // Add any required offset for zeroing the sensor

        // Applying Encoder Configuration to the Motors
        armEncoder.getConfigurator().apply( encoderConfig ); 
    }

    /**
     * Sets the following Motors to a Netural Mode, which stops all the motors from running
     * @return
     */
    @Override
    public void stop() {
        masterMotor.setControl( new NeutralOut() ); 
        followerMotor.setControl( new NeutralOut() ); 
    }

    /**
     * This method uses a simple position control to move the arm to a 
     * specific position (in rotations). It sets the desired position for both the leader 
     * and follower motors using the PositionVoltage control mode.
     * @param positionRotations The desired position that the arm would like to go to
     * @return
     */
    @Override
    public void setPositionControl( double positionRotations ) {
        masterMotor.setControl( pPos.withPosition( positionRotations ) );
        followerMotor.setControl( pPos.withPosition( positionRotations ) );
    }

    /**
     * This method uses Motion Magic to move the arm smoothly to a specific position, 
     * considering the velocity and acceleration limits defined in the configuration.
     * @param positionRotations The specific position that the arm would like to move to
     * @return
     */
    @Override 
    public void setMotionControl( double positionRotations ) {
        masterMotor.setControl( pMnPos.withPosition( positionRotations ) );
        followerMotor.setControl( pMnPos.withPosition( positionRotations ) );  
    }
}
