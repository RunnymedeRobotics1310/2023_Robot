package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private static final MotorType motorType              = MotorType.kBrushless;

    // Arm extension motor
    private final CANSparkMax      armExtendMotor         = new CANSparkMax(ArmConstants.ARM_EXTEND_MOTOR_PORT, motorType);

    // Arm lift motors
    private final CANSparkMax      armLiftMotor           = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT, motorType);
    private final CANSparkMax      armLiftFollower        = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT + 1, motorType);

    // Pincher motor
    private final CANSparkMax      pincherMotor           = new CANSparkMax(ArmConstants.PINCHER_MOTOR_PORT, motorType);

    /*
     * Current Motor Speeds
     */
    private double                 armExtendSpeed         = 0;
    private double                 armLiftSpeed           = 0;
    private double                 pincherSpeed           = 0;

    /*
     * Encoders
     */
    private RelativeEncoder        armExtendEncoder       = armExtendMotor.getEncoder();
    private RelativeEncoder        armLiftEncoder         = armLiftMotor.getEncoder();
    private RelativeEncoder        pincherEncoder         = pincherMotor.getEncoder();

    private double                 armExtendEncoderOffset = 0;
    private double                 armLiftEncoderOffset   = 0;
    private double                 pincherEncoderOffset   = 0;

    /*
     * Limit Switches
     */

    /** The game piece detector is an infra-red sensor that is normally open */
    private SparkMaxLimitSwitch    gamePieceDetector      = pincherMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    /** The pincher open detector is a hall effect limit switch that is normally open */
    private SparkMaxLimitSwitch    pincherOpenDetector    = pincherMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    /** The arm retracted detector is a hall effect limit switch that is normally open */
    private SparkMaxLimitSwitch    armRetractedDetector   = armExtendMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    /** The arm down detector is an infra-red limit switch plugged into the RoboRio */
    private DigitalInput           armDownDetector        = new DigitalInput(ArmConstants.ARM_DOWN_LIMIT_SWITCH_DIO_PORT);

    /** Creates a new ArmSubsystem */
    public ArmSubsystem() {

        armLiftFollower.follow(armLiftMotor);

    }

    /**
     * Gets the extend motor encoder.
     *
     * @return the extend motor encoder position
     */
    public double getArmExtendEncoder() {
        return armExtendEncoder.getPosition() + armExtendEncoderOffset;
    }

    /**
     * Gets the lift motor encoder. (top lift motor)
     *
     * @return the lift motor encoder position
     */
    public double getArmLiftEncoder() {
        return armLiftEncoder.getPosition() + armLiftEncoderOffset;
    }

    /**
     * Gets the pincher motor encoder. (top lift motor)
     *
     * @return the pincher motor encoder position
     */
    public double getPincherEncoder() {
        return pincherEncoder.getPosition() + pincherEncoderOffset;
    }

    /** Determine if the arm is fully down */
    public boolean isArmDown() {

        // NOTE: the arm down detector is an infra-red sensor
        // that "isPressed" when the infra-red detects an object.
        // The switch is normally open so pulled-to-high (1) when
        // no object is detected, and pulled-to-low (0) when an
        // when an object is detected and the switch is closed.
        return !armDownDetector.get();
    }

    /** Determine if the arm is fully retracted */
    public boolean isArmRetracted() {

        // NOTE: the arm retracted detector is a hall-effect
        // magnetic sensor that "isPressed" when the sensor detects a magnet.
        // This sensor is not inverted because the CANLimitSwitch
        // is configured for a normally open switch.
        return armRetractedDetector.isPressed();
    }

    /** Determine if a game piece is detected in the pincher */
    public boolean isGamePieceDetected() {

        // NOTE: the game piece detector is an infra-red sensor
        // that "isPressed" when the infra-red detects an object.
        // This sensor is not inverted because the CANLimitSwitch
        // is configured for a normally open switch.
        return gamePieceDetector.isPressed();
    }

    /** Determine if the pincher fully open */
    public boolean isPincherOpen() {

        // NOTE: the arm retracted detector is a hall-effect
        // magnetic sensor that "isPressed" when the sensor detects a magnet.
        // This sensor is not inverted because the CANLimitSwitch
        // is configured for a normally open switch.
        return pincherOpenDetector.isPressed();
    }

    /**
     * Set the speed of the extension motor
     *
     * @param speed
     */
    public void setArmExtendSpeed(double speed) {

        armExtendSpeed = checkArmExtendLimits(speed);
        armExtendMotor.set(armExtendSpeed);
    }

    /**
     * Set the speed of the lift motors
     *
     * @param speed
     */
    public void setArmLiftSpeed(double speed) {

        armLiftSpeed = checkArmLiftLimits(speed);
        armLiftMotor.set(armLiftSpeed);
    }

    /**
     * Set the speed of the pincher motor
     *
     * @param speed
     */
    public void setPincherSpeed(double speed) {

        pincherSpeed = checkPincherLimits(speed);
        pincherMotor.set(pincherSpeed);
    }

    /** Set the armliftEncoderValue to the value specified */
    public void setArmLiftEncoder(double encoderValue) {
        armLiftEncoderOffset = encoderValue - armLiftEncoder.getPosition();
    }

    /** Set the armliftEncoderValue to the value specified */
    public void setArmExtendEncoder(double encoderValue) {
        armExtendEncoderOffset = encoderValue - armExtendEncoder.getPosition();
    }

    /** Set the pincherEncoderValue to the value specified */
    public void setPincherEncoder(double encoderValue) {
        pincherEncoderOffset = encoderValue - pincherEncoder.getPosition();
    }

    /** Safely stop the all arm motors from moving */
    public void stop() {
        setArmLiftSpeed(0);
        setArmExtendSpeed(0);
        setPincherSpeed(0);
    }

    @Override
    public void periodic() {

        /*
         * Safety-check all of the motors speeds, and
         * set the motor outputs.
         *
         * This is required because a command may set the motor speed
         * at the beginning and may not ever set it again. The periodic
         * loop checks the limits every loop.
         */
        armLiftSpeed = checkArmLiftLimits(armLiftSpeed);
        armLiftMotor.set(armLiftSpeed);

        armExtendSpeed = checkArmExtendLimits(armExtendSpeed);
        armExtendMotor.set(armExtendSpeed);

        pincherSpeed = checkPincherLimits(pincherSpeed);
        pincherMotor.set(pincherSpeed);

        /*
         * Update the SmartDashboard
         */

        SmartDashboard.putNumber("Lift Motor", armLiftSpeed);
        SmartDashboard.putNumber("Extend  Motor", armExtendSpeed);
        SmartDashboard.putNumber("Pincher  Motor", pincherSpeed);

        SmartDashboard.putNumber("Lift Encoder", getArmLiftEncoder());
        SmartDashboard.putNumber("Extend Encoder", getArmExtendEncoder());
        SmartDashboard.putNumber("Pincher Encoder", getPincherEncoder());

        SmartDashboard.putNumber("Raw Lift Encoder", armLiftEncoder.getPosition());
        SmartDashboard.putNumber("Raw Extend Encoder", armExtendEncoder.getPosition());
        SmartDashboard.putNumber("Raw Pincher Encoder", pincherEncoder.getPosition());

        SmartDashboard.putBoolean("Arm Down", isArmDown());
        SmartDashboard.putBoolean("Arm Retracted", isArmRetracted());
        SmartDashboard.putBoolean("Game Piece", isGamePieceDetected());
        SmartDashboard.putBoolean("Pincher Open", isPincherOpen());
    }

    /**
     * Check the arm extend limits and return the appropriate
     * output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the arm extender based on the
     * current extender position.
     */
    private double checkArmExtendLimits(double inputSpeed) {

        boolean atLimit = false;

        /*
         * Extending
         *
         * If the arm encoder count is larger than the extend limit
         * and the speed is positive (extending) then stop
         */

        // FIXME: set the atLimit value based on the above comment ^^^

        /*
         * Retracting
         *
         * If the retraction limit is detected, zero the encoder
         *
         * If the retraction limit is detected and the speed
         * is negative (retracting), then stop
         */

        // FIXME: set the atLimit value based on the above comment ^^^

        if (atLimit) {
            return 0;
        }

        return inputSpeed;
    }

    /**
     * Check the arm lift limits and return the appropriate
     * output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the arm lift based on the
     * current lift position.
     */
    private double checkArmLiftLimits(double inputSpeed) {

        boolean atLimit = false;

        /*
         * Extending
         *
         * If the arm encoder count is larger than the extend limit
         * and the speed is positive (extending) then stop
         */

        // FIXME: set the atLimit value based on the above comment ^^^

        /*
         * Retracting
         *
         * If the retraction limit is detected, zero the encoder
         *
         * If the retraction limit is detected and the speed
         * is negative (retracting), then stop
         */

        // FIXME: set the atLimit value based on the above comment ^^^

        if (atLimit) {
            return 0;
        }

        return inputSpeed;
    }

    /**
     * Check the pincher limits and return the appropriate
     * output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the pincher based on the
     * current pincher position.
     */
    private double checkPincherLimits(double inputSpeed) {

        boolean atLimit = false;

        /*
         * Closing
         *
         * If the pincher encoder count is larger than the max pinch limit
         * and the speed is positive (closing) then stop
         */

        // FIXME: set the atLimit value based on the above comment ^^^

        /*
         * Opening
         *
         * If the open limit is detected, zero the encoder
         *
         * If the pincher limit is detected and the speed
         * is negative (opening), then stop
         */

        // FIXME: set the atLimit value based on the above comment ^^^

        if (atLimit) {
            return 0;
        }

        return inputSpeed;
    }
}
