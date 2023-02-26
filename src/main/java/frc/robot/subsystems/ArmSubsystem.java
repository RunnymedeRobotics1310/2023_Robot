package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private static final MotorType motorType              = MotorType.kBrushless;

    // Arm lift motors
    private final CANSparkMax      armLiftMotor           = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT, motorType);
    private final CANSparkMax      armLiftFollower        = new CANSparkMax(ArmConstants.ARM_LIFT_MOTOR_PORT + 1, motorType);

    private RelativeEncoder        armLiftEncoder         = armLiftMotor.getEncoder();

    private double                 armLiftEncoderOffset   = 0;

    private double                 armLiftSpeed           = 0;

    private IdleMode               currentArmLiftIdleMode = null;
    /*
     * Limit Switches
     */

    /** The arm down detector is an infra-red limit switch plugged into the RoboRio */
    private DigitalInput           armDownDetector        = new DigitalInput(ArmConstants.ARM_DOWN_LIMIT_SWITCH_DIO_PORT);

    /** Creates a new ArmSubsystem */
    public ArmSubsystem() {

        // Ensure that following is disabled
        armLiftMotor.follow(ExternalFollower.kFollowerDisabled, 0);

        // Set the polarity on the motors
        armLiftMotor.setInverted(ArmConstants.ARM_LIFT_MOTOR_REVERSED);
        armLiftFollower.setInverted(ArmConstants.ARM_LIFT_MOTOR_REVERSED);

        setArmLiftIdleMode(IdleMode.kBrake);

        // Setting both encoders to 0
        setArmLiftEncoder(0);

    }

    /**
     * Gets the lift motor encoder. (top lift motor)
     *
     * @return the lift motor encoder position
     */
    public double getArmLiftEncoder() {
        return armLiftEncoder.getPosition() + armLiftEncoderOffset;
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

    /** Set the armliftEncoderValue to the value specified */
    public void setArmLiftEncoder(double encoderValue) {
        armLiftEncoderOffset = encoderValue - armLiftEncoder.getPosition();
    }

    /** Safely stop the all arm motors from moving */
    public void stop() {
        setArmLiftSpeed(0);
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

        /*
         * Update the SmartDashboard
         */

        SmartDashboard.putNumber("Lift Motor", armLiftSpeed);

        SmartDashboard.putNumber("Lift Encoder", getArmLiftEncoder());

        SmartDashboard.putNumber("Raw Lift Encoder", armLiftEncoder.getPosition());

        SmartDashboard.putBoolean("Arm Down", isArmDown());
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

    private void setArmLiftIdleMode(IdleMode idleMode) {

        if (currentArmLiftIdleMode != null && currentArmLiftIdleMode == idleMode) {
            return;
        }

        currentArmLiftIdleMode = idleMode;

        armLiftMotor.setIdleMode(idleMode);
        armLiftFollower.setIdleMode(idleMode);
    }
}
