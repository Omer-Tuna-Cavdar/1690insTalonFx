package frc.robot.Subsytems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    private static Pivot instance;
    private final DigitalInput irBeamBreak;

    private final TalonFX pivotMotor1;
    private final TalonFX pivotMotor2;
    private final CANcoder pivotCANcoder;
    private MotionMagicDutyCycle motionMagicControl;
    private final TalonFXConfiguration pivotMotorConfig;
    private double holdPosition;  // Target position to hold

    public Pivot() {
        pivotMotor1 = new TalonFX(Constants.PivotConstants.kPivotMotor1CanId);
        pivotMotor2 = new TalonFX(Constants.PivotConstants.kPivotMotor2CanId);
        pivotCANcoder = new CANcoder(Constants.PivotConstants.kPivotCANCoderId);
        irBeamBreak = new DigitalInput(Constants.PivotConstants.kPivotIrBeamBreakPort);

        // Configure the CANcoder
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        pivotCANcoder.getConfigurator().apply(cancoderConfig);

        // Configure the TalonFX motors
        pivotMotorConfig = new TalonFXConfiguration();

        // Set up the feedback configuration to use the fused CANcoder
        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotCANcoder.getDeviceID();
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotMotorConfig.Feedback.SensorToMechanismRatio = Constants.PivotConstants.kSensorToMechanismRatio;

        // Configure PID constants (to be tuned later)
        pivotMotorConfig.Slot0.kP = Constants.PivotConstants.kP;
        pivotMotorConfig.Slot0.kI = Constants.PivotConstants.kI;
        pivotMotorConfig.Slot0.kD = Constants.PivotConstants.kD;
        pivotMotorConfig.Slot0.kV = Constants.PivotConstants.kFF; // Feedforward gain
        pivotMotorConfig.Slot0.kS = Constants.PivotConstants.kS;  // Static friction compensation
        
        // Configure Motion Magic parameters (to be tuned later)
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = Constants.PivotConstants.kMotionMagicAcceleration;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotConstants.kMotionMagicCruiseVelocity;

        // Apply the configuration to both motors
        pivotMotor1.getConfigurator().apply(pivotMotorConfig);
        pivotMotor2.getConfigurator().apply(pivotMotorConfig);

        // Set neutral modes
        pivotMotor1.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor2.setNeutralMode(NeutralModeValue.Brake);

        // Set pivotMotor2 to follow pivotMotor1 using Follower control
        Follower followerControl = new Follower(pivotMotor1.getDeviceID(), true);
        pivotMotor2.setControl(followerControl);

        // Initialize Motion Magic control without a target position
        motionMagicControl = new MotionMagicDutyCycle(Constants.PivotConstants.kStowPosition);

        // Initialize hold position
        holdPosition = getPosition();
    }

    public static synchronized Pivot getInstance() {
        if (instance == null) {
            instance = new Pivot();
        }
        return instance;
    }

    public void setPosition(double position) {
        holdPosition = position; // Update hold position
        motionMagicControl = motionMagicControl.withPosition(position);
        pivotMotor1.setControl(motionMagicControl);
    }

    public double getPosition() {
        // Get the position from the TalonFX, which uses the fused CANcoder
        return pivotMotor1.getPosition().getValue();
    }

    public boolean isAtPositionSetpoint(double position) {
        return Math.abs(getPosition() - position) < Constants.PivotConstants.kPivotErrorMargin;
    }

    // Commands
    public Command rotateToPosition(double position) {
        return this.run(() -> setPosition(position))
                .until(() -> isAtPositionSetpoint(position));
    }

    public Command stow() {
        return rotateToPosition(Constants.PivotConstants.kStowPosition);
    }

    @Override
    public void periodic() {
        // Continuously hold the target position
        setPosition(holdPosition);

        // Update telemetry
        double currentPosition = getPosition();
        SmartDashboard.putNumber("Pivot Position", currentPosition);
        Logger.recordOutput("Pivot Position", currentPosition);
    }

    public boolean isPivotBeamBroken() {
        return !irBeamBreak.get(); // IR Beam Break is active-low
    }

    public void resetPivotEncoder() {
        // Create a new configuration object to reset the PositionOffset to zero
        TalonFXConfiguration resetConfig = new TalonFXConfiguration();
        resetConfig.Feedback.FeedbackRotorOffset = 0.0; // Sets offset to zero, effectively resetting encoder reading
        
        // Apply the configuration to reset pivotMotor1’s encoder
        pivotMotor1.getConfigurator().apply(resetConfig);
    
        // Reset CANcoder's absolute position if it's also used in feedback
        pivotCANcoder.setPosition(0.0);
    }
    
    
}
