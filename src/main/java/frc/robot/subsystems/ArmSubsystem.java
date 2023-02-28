package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    public static enum Pose {
        SCORE_HIGH,
        SCORE_MID,
        SUBSTATION_PICKUP,
        SCORE_LOW,
        COMPACT,
        GROUND_PICKUP,
        DRIVING_WITH_ITEM
    }

    public static enum HeldItemState {
        NONE,
        CONE,
        CUBE
    }

    private              Pose          pose;
    private              HeldItemState heldItemState;
    private static final MotorType     motorType = MotorType.kBrushless;

    /*
     * Arm lift motors and encoder
     */
    private final CANSparkMax armLiftMotor    = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT, motorType);
    private final CANSparkMax armLiftFollower = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT + 1, motorType);

    private IdleMode armLiftIdleMode = null;
    private double   armLiftSpeed    = 0;

    // Arm lift encoder
    private RelativeEncoder armLiftEncoder = armLiftMotor.getEncoder();

    private double armLiftEncoderOffset = 0;

    /*
     * Arm extend motor and encoder
     */
    private final CANSparkMax armExtendMotor = new CANSparkMax(ArmConstants.ARM_EXTEND_MOTOR_PORT, motorType);

    private double armExtendSpeed = 0;

    // Arm lift encoder
    private RelativeEncoder armExtendEncoder = armExtendMotor.getEncoder();

    private double armExtendEncoderOffset = 0;

    /*
     * Pincher motor and encoder
     */
    private final CANSparkMax pincherMotor = new CANSparkMax(ArmConstants.PINCHER_MOTOR_PORT, motorType);

    private double pincherSpeed = 0;

    // Pincher encoder
    private RelativeEncoder pincherEncoder = pincherMotor.getEncoder();

    private double pincherEncoderOffset = 0;

    /*
     * Limit Switches
     */
    /** The arm down detector is an infra-red limit switch plugged into the RoboRio */
    private DigitalInput armDownDetector = new DigitalInput(ArmConstants.ARM_DOWN_LIMIT_SWITCH_DIO_PORT);

    /**
     * The arm retracted detector is a hall effect limit switch that is normally open, plugged into the arm extender SparkMAX
     * reverse limit.
     */
    private SparkMaxLimitSwitch armRetractedDetector = armExtendMotor.getReverseLimitSwitch(Type.kNormallyClosed);

    /**
     * The pincher open detector is a hall effect limit switch that is normally open, plugged into the pincher SparkMAX reverse
     * limit.
     */
    private SparkMaxLimitSwitch pincherOpenDetector = pincherMotor.getReverseLimitSwitch(Type.kNormallyClosed);

    /**
     * The game piece detector is an infra-red sensor that is normally open, plugged into the pincher SparkMAX forward limit.
     */
    private SparkMaxLimitSwitch gamePieceDetector = pincherMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    /** Creates a new ArmSubsystem */
    public ArmSubsystem() {

        /*
         * Arm Lift
         */
        // Ensure that following is disabled
        armLiftMotor.follow(ExternalFollower.kFollowerDisabled, 0);

        // Set the polarity on the motors
        armLiftMotor.setInverted(ArmConstants.ARM_LIFT_MOTOR_REVERSED);
        armLiftFollower.setInverted(ArmConstants.ARM_LIFT_MOTOR_REVERSED);

        setArmLiftIdleMode(IdleMode.kBrake);

        setArmLiftEncoder(0);

        /*
         * Arm Extend
         */
        // Set the polarity on the motor
        armExtendMotor.setInverted(ArmConstants.ARM_EXTEND_MOTOR_REVERSED);

        // Set the arm extender to always brake
        armExtendMotor.setIdleMode(IdleMode.kBrake);

        armRetractedDetector.enableLimitSwitch(false);

        setArmExtendEncoder(0);

        /*
         * Pincher
         */
        // Set the polarity on the motor
        pincherMotor.setInverted(ArmConstants.PINCHER_MOTOR_REVERSED);

        // Set the arm extender to always brake
        pincherMotor.setIdleMode(IdleMode.kBrake);

        setPincherEncoder(0);
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
     * Gets the extend motor encoder.
     *
     * @return the extend motor encoder position
     */
    public double getArmExtendEncoder() {
        return armExtendEncoder.getPosition() + armExtendEncoderOffset;
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
     * Set the speed of the lift motors
     *
     * @param speed
     */
    public void setArmLiftSpeed(double speed) {

        setArmLiftIdleMode(IdleMode.kBrake);

        armLiftSpeed = checkArmLiftLimits(speed);
        armLiftMotor.set(armLiftSpeed);
        armLiftFollower.set(armLiftSpeed);
    }

    public void setArmLiftTestSpeed(double armLiftMotorSpeed, double armLiftFollowerSpeed) {

        setArmLiftIdleMode(IdleMode.kCoast);

        armLiftMotor.set(armLiftMotorSpeed);
        armLiftFollower.set(armLiftFollowerSpeed);
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
        armLiftFollower.set(armLiftSpeed);

        armExtendSpeed = checkArmExtendLimits(armExtendSpeed);
        armExtendMotor.set(armExtendSpeed);

        pincherSpeed = checkPincherLimits(pincherSpeed);
        pincherMotor.set(pincherSpeed);

        /*
         * Update the SmartDashboard
         */

        SmartDashboard.putNumber("Lift Motor", armLiftSpeed);
        SmartDashboard.putNumber("Lift Encoder", Math.round(getArmLiftEncoder() * 100) / 100);
        SmartDashboard.putNumber("Raw Lift Encoder", Math.round(armLiftEncoder.getPosition() * 100) / 100);

        SmartDashboard.putNumber("Extend  Motor", armExtendSpeed);
        SmartDashboard.putNumber("Extend Encoder", Math.round(getArmExtendEncoder() * 100) / 100);
        SmartDashboard.putNumber("Raw Extend Encoder", Math.round(armExtendEncoder.getPosition() * 100) / 100);

        SmartDashboard.putNumber("Pincher  Motor", pincherSpeed);
        SmartDashboard.putNumber("Pincher Encoder", Math.round(getPincherEncoder() * 100) / 100);
        SmartDashboard.putNumber("Raw Pincher Encoder", Math.round(pincherEncoder.getPosition() * 100) / 100);

        SmartDashboard.putBoolean("Arm Down", isArmDown());
        SmartDashboard.putBoolean("Arm Retracted", isArmRetracted());
        SmartDashboard.putBoolean("Game Piece", isGamePieceDetected());
        SmartDashboard.putBoolean("Pincher Open", isPincherOpen());
    }

    private void setArmLiftIdleMode(IdleMode idleMode) {

        // If the idle mode is already set to the supplied
        // value, there is nothing to do.
        if (armLiftIdleMode == idleMode) {
            return;
        }

        armLiftIdleMode = idleMode;

        armLiftMotor.setIdleMode(idleMode);
        armLiftFollower.setIdleMode(idleMode);
    }

    /**
     * Check the arm lift limits and return the appropriate output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the arm lift based on the current lift position.
     */
    private double checkArmLiftLimits(double inputSpeed) {

        boolean atLimit = false;
        /*
         * Lifting
         *
         * If the arm encoder count is larger than the lift limit
         * and the speed is positive (lifting) then stop
         */

        if ((armLiftEncoder.getPosition() > ArmConstants.ARM_LIFT_LIMIT) && (armLiftSpeed > 0)) {
            atLimit = true;
        }

        /*
         * Lowering
         *
         * If the lowering limit is detected, zero the encoder
         *
         * If the lowering limit is detected and the speed
         * is negative (lowering), then stop
         */

        if (armDownDetector.get()) {
            setArmLiftEncoder(0);
        }
        if ((armLiftEncoder.getPosition() < ArmConstants.ARM_LOWER_LIMIT) && (armLiftSpeed < 0)) {
            atLimit = true;
        }

        if (atLimit) {
            return 0;
        }

        return inputSpeed;
    }

    /**
     * Check the arm extend limits and return the appropriate output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the arm extender based on the current extender position.
     */
    private double checkArmExtendLimits(double inputSpeed) {

        boolean atLimit = false;

        /*
         * Extending
         *
         * If the arm encoder count is larger than the extend limit
         * and the speed is positive (extending) then stop
         */

        if ((armExtendEncoder.getPosition() > ArmConstants.ARM_EXTEND_LIMIT) && (armExtendSpeed > 0)) {
            atLimit = true;
        }

        /*
         * Retracting
         *
         * If the retraction limit is detected, zero the encoder
         *
         * If the retraction limit is detected and the speed
         * is negative (retracting), then stop
         */

        if (armRetractedDetector.isPressed()) {
            setArmExtendEncoder(0);
        }
        if ((armLiftEncoder.getPosition() < ArmConstants.ARM_RETRACT_LIMIT) && (armExtendSpeed < 0)) {
            atLimit = true;
        }

        if (atLimit) {
            return 0;
        }

        return inputSpeed;
    }

    /**
     * Check the pincher limits and return the appropriate output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the pincher based on the current pincher position.
     */
    private double checkPincherLimits(double inputSpeed) {

        boolean atLimit = false;

        /*
         * Closing
         *
         * If the pincher encoder count is larger than the max pinch limit
         * and the speed is positive (closing) then stop
         */

        if ((pincherEncoder.getPosition() < ArmConstants.PINCHER_CLOSE_LIMIT) && (pincherSpeed > 0)) {
            atLimit = true;
        }

        /*
         * Opening
         *
         * If the open limit is detected, zero the encoder
         *
         * If the pincher limit is detected and the speed
         * is negative (opening), then stop
         */

        if (pincherOpenDetector.isPressed()) {
            setPincherEncoder(0);
        }
        if ((pincherEncoder.getPosition() > ArmConstants.PINCHER_OPEN_LIMIT) && (armExtendSpeed < 0)) {
            atLimit = true;
        }

        if (atLimit) {
            return 0;
        }

        return inputSpeed;
    }

    public void enterPose(Pose pose) {
        // fixme: configure arm and pincher for every pose
        // save pose
    }

    public Pose getPose() {
        return pose;
    }

    public void grabCone() {
        // fixme: close pincer on cone
        // update held item state
    }

    public void grabCube() {
        // fixme: close pincer on cube
        // update held item state
    }

    public HeldItemState getHeldItemState() {
        return heldItemState;
    }
}
