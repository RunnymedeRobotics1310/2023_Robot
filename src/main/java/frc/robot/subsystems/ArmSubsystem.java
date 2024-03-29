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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants.GamePiece;

public class ArmSubsystem extends SubsystemBase {

    private static final MotorType motorType               = MotorType.kBrushless;

    /*
     * Arm lift motors and encoder
     */
    private final CANSparkMax      armLiftMotor            = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT, motorType);
    private final CANSparkMax      armLiftFollower         = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT + 1, motorType);

    private IdleMode               armLiftIdleMode         = null;
    private double                 armLiftSpeed            = 0;

    // Arm lift encoder
    private RelativeEncoder        armLiftEncoder          = armLiftMotor.getEncoder();
    private double                 armLiftEncoderOffset    = 0;

    // Arm lift PID
    private boolean                armLiftPidEnabled       = false;
    private double                 armLiftPidAngleSetpoint = ArmConstants.ARM_DOWN_ANGLE_DEGREES;
    // Arm Test Mode
    private boolean                armTestMode             = false;

    /*
     * Arm extend motor and encoder
     */
    private final CANSparkMax      armExtendMotor          = new CANSparkMax(ArmConstants.ARM_EXTEND_MOTOR_PORT, motorType);
    private double                 armExtendSpeed          = 0;

    // Arm lift encoder
    private RelativeEncoder        armExtendEncoder        = armExtendMotor.getEncoder();
    private double                 armExtendEncoderOffset  = 0;

    /*
     * Pincher motor and encoder
     */
    private final CANSparkMax      pincherMotor            = new CANSparkMax(ArmConstants.PINCHER_MOTOR_PORT, motorType);
    private double                 pincherSpeed            = 0;

    // Pincher encoder
    private RelativeEncoder        pincherEncoder          = pincherMotor.getEncoder();
    private double                 pincherEncoderOffset    = 0;

    /*
     * Limit Switches
     */
    /** The arm down detector is an infra-red limit switch plugged into the RoboRio */
    private DigitalInput           armDownDetector         = new DigitalInput(ArmConstants.ARM_DOWN_LIMIT_SWITCH_DIO_PORT);

    /**
     * The arm retracted detector is a hall effect limit switch that is normally open, plugged into
     * the arm extender SparkMAX
     * reverse limit.
     */
    // private SparkMaxLimitSwitch armExtendLimitDetector = armExtendMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private DigitalInput           armExtendLimitDetector  = new DigitalInput(ArmConstants.ARM_EXTEND_LIMIT_SWITCH_PORT);

    /**
     * The pincher open detector is a hall effect limit switch that is normally open, plugged into
     * the pincher SparkMAX reverse
     * limit.
     */
    private SparkMaxLimitSwitch    pincherOpenDetector     = pincherMotor.getReverseLimitSwitch(Type.kNormallyOpen);

    /**
     * The game piece detector is an infra-red sensor that is normally open, plugged into the
     * pincher SparkMAX forward limit.
     */
    private SparkMaxLimitSwitch    gamePieceDetector       = pincherMotor.getForwardLimitSwitch(Type.kNormallyOpen);

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

        armLiftPidAngleSetpoint = ArmConstants.ARM_DOWN_ANGLE_DEGREES;
        setArmLiftPidEnabled(true);

        /*
         * Arm Extend
         */
        // Set the polarity on the motor
        armExtendMotor.setInverted(ArmConstants.ARM_EXTEND_MOTOR_REVERSED);

        // Set the arm extender to always brake
        armExtendMotor.setIdleMode(IdleMode.kBrake);

        // Set the arm as slightly extended on power up.
        setArmExtendEncoder(2);

        /*
         * Pincher
         */
        // Set the polarity on the motor
        pincherMotor.setInverted(ArmConstants.PINCHER_MOTOR_REVERSED);

        // The pincher motor is a neo550 which should be current limited
        // when stalled in order to not burn out the motor
        pincherMotor.setSmartCurrentLimit(ArmConstants.PINCHER_MOTOR_CURRENT_LIMIT);

        // Set the arm extender to always brake
        pincherMotor.setIdleMode(IdleMode.kBrake);

        pincherOpenDetector.enableLimitSwitch(true);

        // Disable the forward limit switch because it is
        // used to detect a game piece instead
        gamePieceDetector.enableLimitSwitch(false);

        // Set the pincher to the minimum inside the frame distance
        // on power up
        setPincherEncoder(ArmConstants.MIN_PINCHER_INSIDE_FRAME_POSITION);
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
     * Gets the lift arm angle in degrees where 0 = vertical and 90 deg = parallel to the floor.
     *
     * @return the lift angle in degrees (rounded to 2 decimal places)
     */
    public double getArmLiftAngle() {
        return Math.round(
            ((getArmLiftEncoder() * ArmConstants.ARM_DEGREES_PER_ENCODER_COUNT)
                + ArmConstants.ARM_DOWN_ANGLE_DEGREES) * 100)
            / 100d;

    }

    public double getArmLiftAngleSetpoint() {
        return armLiftPidAngleSetpoint;
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

    public Constants.GameConstants.GamePiece getHeldGamePiece() {

        if (gamePieceDetector.isPressed()) {

            if (Math.abs(getPincherEncoder() - GamePiece.CONE.pincherEncoderCount) <= 8) {
                return GamePiece.CONE;
            }

            if (Math.abs(getPincherEncoder() - GamePiece.CUBE.pincherEncoderCount) <= 8) {
                return GamePiece.CUBE;
            }

        }
        return GamePiece.NONE;
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

    /**
     * Determine if the arm is at the supplied angle in degrees, within the arm lift angle tolerance
     *
     * @param angle to compare
     * @return {@code true} if at the angle, {@code false} otherwise
     */
    public boolean isArmAtLiftAngle(double angle) {

        if (Math.abs(angle - getArmLiftAngle()) <= ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES) {
            return true;
        }
        return false;
    }

    /**
     * determine if the arm is at the supplied position in encoder counts, within the arm extend
     * position tolerance
     *
     * @param position to compare
     * @return {@code true} if at extension, {@code false} otherwise
     */
    public boolean isAtExtendPosition(double position) {

        if (Math.abs(position - getArmExtendEncoder()) <= ArmConstants.ARM_EXTEND_POSITION_TOLERANCE) {
            return true;
        }
        return false;
    }

    /**
     * determine if the pincher is at the supplied position in encoder counts, within the pincher
     * position tolerance
     *
     * @param position to compare
     * @return {@code true} if at position, {@code false} otherwise
     */
    public boolean isAtPincherPosition(double position) {

        if (Math.abs(position - getPincherEncoder()) <= ArmConstants.PINCHER_POSITION_TOLERANCE) {
            return true;
        }
        return false;
    }

    /**
     * Determine if the pincher is currently inside the frame and can be safely lowered
     *
     * @return {@code true} if inside frame, {@code false} otherwise
     */
    public boolean isPincherInsideFrame() {

        // The pincher encoder must be higher than the min frame position (within tolerance)
        if (getPincherEncoder() > (ArmConstants.MIN_PINCHER_INSIDE_FRAME_POSITION - ArmConstants.PINCHER_POSITION_TOLERANCE)) {
            return true;
        }

        return false;
    }

    public boolean isInPosition(Constants.ArmPosition position) {
        return isAtExtendPosition(position.extension) && isArmAtLiftAngle(position.angle);
    }

    /** Determine if the arm is at the upper encoder limit */
    public boolean isArmAtUpperLimit() {

        // There is no upper arm limit switch - the arm upper
        // limit is a soft limit that uses the arm encoder counts

        // The upper limit is larger than the cone top scoring position
        return getArmLiftEncoder() > ArmConstants.ARM_LIFT_LIMIT_ENCODER_VALUE;
    }

    /** Determine if the arm is fully extended. This safety mechanism keeps the arm to 4 ft. */
    public boolean isArmAtExtendLimit() {

        // The arm extension uses a limit switch as a backup for the arm extension limit.
        // This limit switch is used for both the forward and reverse limits.
        // The limit switch cannot really determine whether the arm is fully extended
        // or fully retracted.

        // Hopefully the armExtenderEncoder is approximately accurate,
        // and the limit detector is a backup if the encoder counts are
        // not quite accurate (but are close). Anything less than
        // half of the full extend value is considered a retract limit.

        // Also stop when the encoder counts are reached.
        if (getArmExtendEncoder() >= ArmConstants.ARM_EXTEND_LIMIT_ENCODER_VALUE) {
            return true;
        }

        return false;
    }

    /**
     * Determine if the arm is inside the frame
     *
     * @return {@code true} if inside the frame, {@code false} otherwise
     */
    public boolean isArmInsideFrame() {

        if (getArmLiftAngle() > ArmConstants.CLEAR_FRAME_ARM_ANGLE) {
            return false;
        }

        if (getArmExtendEncoder() > ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {
            return false;
        }

        return true;
    }

    /** Determine if the arm is fully retracted */
    public boolean isArmRetracted() {

        // NOTE: the arm retracted detector is a hall-effect
        // magnetic sensor that "isPressed" when the sensor detects a magnet.
        // This sensor is not inverted because the CANLimitSwitch
        // is configured for a normally open switch.

        return !armExtendLimitDetector.get();
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

    /** Determine if the pincher fully closed */
    public boolean isPincherAtCloseLimit() {

        // There is no pincher close limit switch - the pincher close
        // limit is a soft limit that uses the pincher encoder counts.

        // NOTE; The pincher should never hit the close limit because
        // it should be closing on a game piece at the correct
        // encoder counts

        return getPincherEncoder() > ArmConstants.PINCHER_CLOSE_LIMIT_ENCODER_VALUE;
    }

    public void moveArmLiftToAngle(double angle) {

        // if the arm lift pid is not enabled,
        // this routine does nothing
        if (!armLiftPidEnabled) {
            return;
        }

        setArmLiftIdleMode(IdleMode.kBrake);

        armLiftPidAngleSetpoint = Math.max(0, Math.min(angle, 110));

        armTestMode             = false;
    }

    /**
     * Set the arm pid enabled
     *
     * @param speed
     */
    public void setArmLiftPidEnabled(boolean enabled) {

        // If already set, there is nothing to do.
        if (armLiftPidEnabled == enabled) {
            return;
        }

        armLiftPidEnabled = enabled;

        if (armLiftPidEnabled) {
            armLiftPidAngleSetpoint = getArmLiftAngle();
        }
        else {
            armLiftSpeed = 0;
        }
    }

    /**
     * Set the speed of the lift motors
     *
     * @param speed
     */
    public void setArmLiftSpeed(double speed) {

        // Do not set the speed if the pid is enabled
        if (armLiftPidEnabled) {
            return;
        }

        setArmLiftIdleMode(IdleMode.kBrake);
        armTestMode  = false;

        armLiftSpeed = checkArmLiftLimits(speed);

        // Add the hold value to hold the arm steady at the angle
        double outputArmSpeed = armLiftSpeed + calcArmLiftHoldSpeed();

        armLiftMotor.set(outputArmSpeed);
        armLiftFollower.set(outputArmSpeed);

        SmartDashboard.putNumber("Arm Lift output", outputArmSpeed);
    }

    public void setArmLiftTestSpeed(double armLiftMotorSpeed, double armLiftFollowerSpeed) {

        setArmLiftIdleMode(IdleMode.kCoast);

        armTestMode       = true;
        armLiftPidEnabled = false;

        double motorSpeed = 0;

        if (armLiftMotorSpeed != 0) {
            motorSpeed = armLiftMotorSpeed + calcArmLiftHoldSpeed();
            armLiftMotor.set(motorSpeed);
            SmartDashboard.putNumber("Arm Lift output", motorSpeed);
        }

        if (armLiftFollowerSpeed != 0) {
            motorSpeed = armLiftFollowerSpeed + calcArmLiftHoldSpeed();
            armLiftFollower.set(motorSpeed);
            SmartDashboard.putNumber("Arm Lift output", motorSpeed);
        }
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
        stopArm();
        setArmExtendSpeed(0);
        setPincherSpeed(0);
    }

    public void stopArm() {
        // The arm is controlled by a PID controller. Stop the arm by setting the angle to the
        // current angle.
        setArmLiftPidEnabled(true);
        moveArmLiftToAngle(getArmLiftAngle());
    }

    @Override
    public void periodic() {

        /*
         * Check for any encoder resets based on limit switches
         */
        if (isArmDown()) {
            setArmLiftEncoder(0);
        }

        if (isArmRetracted()) {
            setArmExtendEncoder(0);
        }

        if (isPincherOpen()) {
            setPincherEncoder(0);
        }

        /*
         * Safety-check all of the motors speeds, and
         * set the motor outputs.
         *
         * This is required because a command may set the motor speed
         * at the beginning and may not ever set it again. The periodic
         * loop checks the limits every loop.
         */
        if (armLiftPidEnabled) {
            calculateArmLiftPidOutput();
        }
        else {
            if (!armTestMode) {
                setArmLiftSpeed(armLiftSpeed);
            }
        }

        setArmExtendSpeed(armExtendSpeed);
        setPincherSpeed(pincherSpeed);

        /*
         * Update the SmartDashboard
         */

        SmartDashboard.putNumber("Lift Motor", armLiftSpeed);
        SmartDashboard.putNumber("Lift Encoder", Math.round(getArmLiftEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Raw Lift Encoder", Math.round(armLiftEncoder.getPosition() * 100) / 100d);
        SmartDashboard.putNumber("Arm Lift Angle", getArmLiftAngle());
        SmartDashboard.putBoolean("Arm PID Enabled", armLiftPidEnabled);
        SmartDashboard.putNumber("Arm PID Target Angle", armLiftPidAngleSetpoint);


        SmartDashboard.putNumber("Extend  Motor", armExtendSpeed);
        SmartDashboard.putNumber("Extend Encoder", Math.round(getArmExtendEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Raw Extend Encoder", Math.round(armExtendEncoder.getPosition() * 100) / 100d);

        SmartDashboard.putNumber("Pincher  Motor", pincherSpeed);
        SmartDashboard.putNumber("Pincher Encoder", Math.round(getPincherEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Raw Pincher Encoder", Math.round(pincherEncoder.getPosition() * 100) / 100d);

        SmartDashboard.putBoolean("Arm Down", isArmDown());
        SmartDashboard.putBoolean("Arm Up Limit", isArmAtUpperLimit());
        SmartDashboard.putBoolean("Arm Retracted", isArmRetracted());
        SmartDashboard.putBoolean("Arm Extend Limit", isArmAtExtendLimit());
        SmartDashboard.putBoolean("Pincher Open", isPincherOpen());
        SmartDashboard.putBoolean("Pincher Closed", isPincherAtCloseLimit());
        SmartDashboard.putBoolean("Game Piece", isGamePieceDetected());
        SmartDashboard.putString("Current Game Piece", getHeldGamePiece().toString());

    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append(getArmLiftAngle()).append("deg")
            .append(", Ext ").append(Math.round(armExtendEncoder.getPosition() * 100) / 100d)
            .append(", Pinch ").append(Math.round(pincherEncoder.getPosition() * 100) / 100d)
            .append(", HeldGamePiece ").append(getHeldGamePiece());

        return sb.toString();
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

    private void calculateArmLiftPidOutput() {

        // This method will set the arm lift motor speed based on the
        // angle error.

        double angleError = armLiftPidAngleSetpoint - getArmLiftAngle();

        armLiftSpeed = angleError * ArmConstants.ARM_LIFT_PID_P;

        // Limit the output
        armLiftSpeed = checkArmLiftLimits(armLiftSpeed);

        // Add the hold value to hold the arm steady at the angle
        double outputArmSpeed = armLiftSpeed + calcArmLiftHoldSpeed();

        armLiftMotor.set(outputArmSpeed);
        armLiftFollower.set(outputArmSpeed);

        SmartDashboard.putNumber("Arm Lift output", outputArmSpeed);

    }

    /**
     * Check the arm lift limits and return the appropriate output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the arm lift based on the current lift position.
     */
    private double checkArmLiftLimits(double inputSpeed) {

        /*
         * Lifting
         *
         * If the arm encoder count is larger than the lift limit
         * and the speed is positive (lifting) then stop
         */
        if (inputSpeed > 0) {

            if (isArmAtUpperLimit()) {
                return 0;
            }

            // Slow down if approaching the limit
            if (Math.abs(getArmLiftAngle() - ArmConstants.ARM_LIFT_LIMIT_DEGREES) < ArmConstants.ARM_LIFT_SLOW_ZONE_DEGREES) {

                return Math.min(inputSpeed, ArmConstants.MAX_LIFT_SLOW_ZONE_SPEED);
            }

            // Limit the up speed to the max
            return Math.min(inputSpeed, ArmConstants.MAX_LIFT_UP_SPEED);
        }

        /*
         * Lowering
         *
         * If the lowering limit is detected and the speed
         * is negative (lowering), then stop
         */
        if (inputSpeed < 0) {

            if (isArmDown()) {
                return 0;
            }

            // Slow down if approaching the limit
            if (Math.abs(getArmLiftAngle() - ArmConstants.ARM_DOWN_ANGLE_DEGREES) < ArmConstants.ARM_LIFT_SLOW_ZONE_DEGREES) {

                return Math.max(inputSpeed, -ArmConstants.MAX_LIFT_SLOW_ZONE_SPEED);
            }

            // Limit the down speed to the max
            return Math.max(inputSpeed, -ArmConstants.MAX_LIFT_DOWN_SPEED);
        }

        return 0;
    }

    /**
     * Check the arm extend limits and return the appropriate output speed based on the limits
     *
     * @param inputSpeed
     * @return output speed for the arm extender based on the current extender position.
     */
    private double checkArmExtendLimits(double inputSpeed) {

        /*
         * Extending
         *
         * If the arm encoder count is larger than the extend limit
         * and the speed is positive (extending) then stop
         */

        if (inputSpeed > 0) {

            if (isArmAtExtendLimit()) {
                return 0;
            }

            // do not extend the arm if it's already as far as it can go, and it's in the frame
            if (isArmInsideFrame() && getArmExtendEncoder() >= ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {
                return 0;
            }

            // Slow down if approaching the limit
            if (Math.abs(getArmExtendEncoder()
                - ArmConstants.ARM_EXTEND_LIMIT_ENCODER_VALUE) < ArmConstants.ARM_EXTEND_SLOW_ZONE_ENCODER_VALUE) {

                return Math.min(inputSpeed, ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED);
            }
        }

        /*
         * Retracting
         *
         * If the retraction limit is detected, zero the encoder
         *
         * If the retraction limit is detected and the speed
         * is negative (retracting), then stop
         */

        if (inputSpeed < 0) {

            if (isArmRetracted()) {
                return 0;
            }

            // Slow down if approaching the limit
            if (Math.abs(getArmExtendEncoder()) < ArmConstants.ARM_EXTEND_SLOW_ZONE_ENCODER_VALUE) {

                return Math.max(inputSpeed, -ArmConstants.MAX_EXTEND_SLOW_ZONE_SPEED);
            }

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

        /*
         * Closing
         *
         * If the pincher encoder count is larger than the max pinch limit
         * and the speed is positive (closing) then stop
         */

        if (inputSpeed > 0) {

            if (isPincherAtCloseLimit()) {
                return 0;
            }

            // NOTE: Do not slow down when closing the pincher on a cone because the power may be
            // needed.
            if (!isGamePieceDetected()) {

                // Slow down if approaching the limit
                if (Math.abs(getPincherEncoder()
                    - ArmConstants.PINCHER_CLOSE_LIMIT_ENCODER_VALUE) < ArmConstants.PINCHER_SLOW_ZONE_ENCODER_VALUE) {

                    return Math.min(inputSpeed, ArmConstants.MAX_PINCHER_SLOW_ZONE_SPEED);
                }
            }

        }

        /*
         * Opening
         *
         * If the open limit is detected, zero the encoder
         *
         * If the pincher limit is detected and the speed
         * is negative (opening), then stop
         */

        if (inputSpeed < 0) {

            if (isPincherOpen()) {
                return 0;
            }

            // if we are in the frame, don't open the pincher beyond the limit.
            if (isArmInsideFrame() && pincherEncoder.getPosition() <= ArmConstants.MIN_PINCHER_INSIDE_FRAME_POSITION) {
                return 0;
            }

            // Slow down if approaching the limit
            if (Math.abs(getPincherEncoder()) < ArmConstants.PINCHER_SLOW_ZONE_ENCODER_VALUE) {

                return Math.max(inputSpeed, -ArmConstants.MAX_PINCHER_SLOW_ZONE_SPEED);
            }

        }

        return inputSpeed;
    }

    /**
     * calculate the amount of torque to apply to the motor to hold the arm steady at any angle or
     * arm extension.
     *
     * @return the speed adjustment to overcome gravity
     */
    private double calcArmLiftHoldSpeed() {

        /*
         * If at the limits, then do not hold the motor
         */
        if (isArmDown() || isArmAtUpperLimit()) {
            return 0;
        }

        /*
         * When the arm is not intended to move, some force is required in
         * order to hold the arm steady.
         *
         * The following measurements were obtained using experiments with the arm:
         *
         * Arm Angle = 90 degrees
         *
         * .............Motor Speed
         *
         * Extension ..No Cone ..With Cone
         *
         * Retracted ____.06 ______.08
         * Extended _____.10 ______.14
         *
         */

        /*
         * The angle multiplier is based on the amount of force required to
         * overcome gravity at this angle.
         *
         * At zero degrees, straight down, no additional force is required to
         * overcome gravity.
         *
         * At 90 degrees, parallel to the floor, the speeds measured above^^^ are required
         * to hold a retracted and extended arm parallel to the floor.
         *
         * The amount of force required follows the sin (trigonometry) function
         * at angles other than 90 degrees.
         */
        double angleMultiplier  = Math.sin(Math.toRadians(getArmLiftAngle()));

        /*
         * The extension multiplier is based on the amount of additional force required as the
         * arm extends.
         *
         * Based on the above observations, the amount of force required when the arm
         * is extended is approximately 1.7 times the amount required when retracted.
         *
         * No cone. : .10 (extended) / .06 (retracted) = 1.67
         * With cone: .14 (extended) / .08 (retracted) = 1.75
         *
         * We are choosing a value in between at 1.7
         */
        double extendMultiplier = 1 + (getArmExtendEncoder() / ArmConstants.ARM_EXTEND_LIMIT_ENCODER_VALUE * .7);

        /*
         * The base compensation comes from the retracted arm values in the table
         */
        double baseCompensation = 0.06;

        if (getHeldGamePiece() == GamePiece.CONE) {
            baseCompensation += 0.02;
        }

        /*
         * The total compensation is the multiplication of the base compensation
         * and the angle and extension adjustments.
         */
        return baseCompensation * angleMultiplier * extendMultiplier;
    }
}
