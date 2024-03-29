package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final CANSparkMax     leftPrimaryMotor         = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT,
        MotorType.kBrushless);

    private final CANSparkMax     leftFollowerMotor        = new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT + 1,
        MotorType.kBrushless);

    // The motors on the right side of the drive.
    private final CANSparkMax     rightPrimaryMotor        = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT,
        MotorType.kBrushless);

    private final CANSparkMax     rightFollowerMotor       = new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT + 1,
        MotorType.kBrushless);

    private double                leftSpeed                = 0;
    private double                rightSpeed               = 0;

    private IdleMode              idleMode                 = null;

    // Encoders
    private final RelativeEncoder leftEncoder              = leftPrimaryMotor.getEncoder();
    private final RelativeEncoder rightEncoder             = rightPrimaryMotor.getEncoder();

    private double                leftEncoderOffset        = 0;
    private double                rightEncoderOffset       = 0;

    // Conversion from volts to distance in cm
    // Volts distance
    // 0.12 30.5 cm
    // 2.245 609.6 cm
    private final AnalogInput     ultrasonicDistanceSensor = new AnalogInput(0);

    private final double          ULTRASONIC_M             = (609.6 - 30.5) / (2.245 - .12);
    private final double          ULTRASONIC_B             = 609.6 - ULTRASONIC_M * 2.245;

    /*
     * Gyro
     */
    private AHRS                  navXGyro                 = new AHRS() {
                                                               // Override the "Value" in the gyro
                                                               // sendable to use the angle instead
                                                               // of
                                                               // the yaw.
                                                               // Using the angle makes the gyro
                                                               // appear
                                                               // in the correct position accounting
                                                               // for the
                                                               // offset. The yaw is the raw sensor
                                                               // value which appears incorrectly on
                                                               // the dashboard.
                                                               @Override
                                                               public void initSendable(SendableBuilder builder) {
                                                                   builder.setSmartDashboardType("Gyro");
                                                                   builder.addDoubleProperty("Value", this::getAngle, null);
                                                               }
                                                           };

    private double                gyroHeadingOffset        = 0;
    private double                gyroPitchOffset          = 0;

    private enum GyroAxis {
        YAW, PITCH, ROLL
    }

    private final ArmSubsystem armSubsystem;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(ArmSubsystem armSubsystem) {

        this.armSubsystem = armSubsystem;

        // Ensure that following is disabled
        leftPrimaryMotor.follow(ExternalFollower.kFollowerDisabled, 0);
        rightPrimaryMotor.follow(ExternalFollower.kFollowerDisabled, 0);

        // Set the polarity on the motors
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        setIdleMode(IdleMode.kBrake);
        resetEncoders();
    }

    /**
     * Calibrate Gyro
     * <p>
     * This routine calibrates the gyro. The robot must not be moved during the calibrate routine
     * which lasts about 10 seconds
     */
    public void calibrateGyro() {

        gyroHeadingOffset = 0;

        navXGyro.calibrate();
    }

    /**
     * Reset Gyro
     * <p>
     * This routine resets the gyro angle to zero.
     * <p>
     * NOTE: This is not the same as calibrating the gyro.
     */
    public void resetGyro() {

        setGyroHeading(0);
        setGyroPitch(0);
    }

    /**
     * Set Gyro Heading
     * <p>
     * This routine sets the gyro heading to a known value.
     */
    public void setGyroHeading(double heading) {

        // Clear the current offset.
        gyroHeadingOffset = 0;

        // Adjust the offset so that the heading is now the current heading.
        gyroHeadingOffset = heading - getHeading();

        // Send the offset to the navX in order to have the
        // compass on the dashboard appear at the correct heading.
        navXGyro.setAngleAdjustment(gyroHeadingOffset);

        // FIXME: (low) Hack
        if (heading == 0) {
            resetEncoders();
        }
    }

    /**
     * Set Gyro Pitch
     * <p>
     * This routine sets the gyro pitch to a known value.
     */
    public void setGyroPitch(double pitch) {

        // Clear the current offset.
        gyroPitchOffset = 0;

        // Adjust the offset so that the heading is now the current heading.
        gyroPitchOffset = pitch - getPitch();
    }

    /**
     * Gets the heading of the robot.
     *
     * @return heading in the range of 0 - 360 degrees
     */
    public double getHeading() {

        double gyroAngle = getRawGyroAngle(GyroAxis.YAW);

        // adjust by the offset that was saved when the gyro
        // heading was last set.
        gyroAngle += gyroHeadingOffset;

        // The angle can be positive or negative and extends beyond 360 degrees.
        double heading = gyroAngle % 360.0;

        if (heading < 0) {
            heading += 360;
        }

        // round to two decimals
        return Math.round(heading * 100) / 100d;
    }

    private double getRawGyroAngle(GyroAxis gyroAxis) {

        switch (gyroAxis) {
        case YAW:
            return navXGyro.getYaw();
        case PITCH:
            // Pitch and roll are reversed
            // in the vertical mounting of the rio
            return navXGyro.getRoll();
        case ROLL:
            return navXGyro.getPitch();
        default:
            return 0;
        }
    }

    public double getPitch() {

        double gyroPitch = getRawGyroAngle(GyroAxis.PITCH);

        // adjust by the offset that was saved when the gyro
        // pitch was last set.
        gyroPitch += gyroPitchOffset;

        // Round to two decimals
        return Math.round(gyroPitch * 100) / 100d;
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderValue() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getEncoderDistanceCm() {

        return getAverageEncoderValue() * DriveConstants.CM_PER_ENCODER_COUNT;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return leftEncoder.getPosition() + leftEncoderOffset;
    }

    /**
     * Gets the left velocity.
     *
     * @return the left drive encoder speed
     */
    public double getLeftEncoderSpeed() {
        return leftEncoder.getVelocity();
    }

    /**
     * Gets the right velocity.
     *
     * @return the right drive encoder speed
     */
    public double getRightEncoderSpeed() {
        return rightEncoder.getVelocity();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return rightEncoder.getPosition() + rightEncoderOffset;
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {

        leftEncoderOffset  = -leftEncoder.getPosition();
        rightEncoderOffset = -rightEncoder.getPosition();
    }

    public double getUltrasonicDistanceCm() {

        double ultrasonicVoltage = ultrasonicDistanceSensor.getVoltage();

        double distanceCm        = ULTRASONIC_M * ultrasonicVoltage + ULTRASONIC_B;

        return Math.round(distanceCm);
    }

    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        // when this method is called, the motors should be set to brake.
        setIdleMode(IdleMode.kBrake);

        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        checkMotorSpeedLimits();

        leftPrimaryMotor.set(this.leftSpeed);
        leftFollowerMotor.set(this.leftSpeed);

        rightPrimaryMotor.set(this.rightSpeed);
        rightFollowerMotor.set(this.rightSpeed);
    }

    public void setTestMotorSpeeds(double leftPrimaryMotorSpeed, double leftFollowerMotorSpeed,
        double rightPrimaryMotorSpeed, double rightFollowerMotorSpeed) {

        // When this method is called, the motors should be set to coast.
        setIdleMode(IdleMode.kCoast);

        leftPrimaryMotor.set(leftPrimaryMotorSpeed);
        leftFollowerMotor.set(leftFollowerMotorSpeed);

        rightPrimaryMotor.set(rightPrimaryMotorSpeed);
        rightFollowerMotor.set(rightFollowerMotorSpeed);
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    @Override
    public void periodic() {

        // Check the motor speed limits every loop
        setMotorSpeeds(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left  Motor", leftSpeed);

        SmartDashboard.putNumber("Right Encoder", Math.round(getRightEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Left Encoder", Math.round(getLeftEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Avg Encoder", Math.round(getAverageEncoderValue() * 100) / 100d);
        SmartDashboard.putNumber("Distance (cm)", Math.round(getEncoderDistanceCm() * 10) / 10d);
        SmartDashboard.putNumber("Right Velocity", Math.round(getRightEncoderSpeed() * 100) / 100d);
        SmartDashboard.putNumber("Left Velocity", Math.round(getLeftEncoderSpeed() * 100) / 100d);

        SmartDashboard.putNumber("Ultrasonic Voltage", ultrasonicDistanceSensor.getVoltage());
        SmartDashboard.putNumber("Ultrasonic Distance (cm)", Math.round(getUltrasonicDistanceCm()));

        SmartDashboard.putData("Gyro", navXGyro);
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Pitch", getPitch());

        // SmartDashboard.putNumber("Gyro Raw Yaw", getRawGyroAngle(GyroAxis.YAW));
        // SmartDashboard.putNumber("Gyro Raw Pitch", getRawGyroAngle(GyroAxis.PITCH));
        // SmartDashboard.putNumber("Gyro Raw Roll", getRawGyroAngle(GyroAxis.ROLL));
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("Head ").append(getHeading())
            .append(", Pitch ").append(getPitch())
            .append(", Ultrasonic dist ").append(Math.round(getUltrasonicDistanceCm()))
            .append(", Drive dist ").append(Math.round(getEncoderDistanceCm() * 10) / 10d);

        return sb.toString();
    }

    private void setIdleMode(IdleMode idleMode) {

        // If the idle mode is already set to the supplied
        // value, there is nothing to do.
        if (this.idleMode == idleMode) {
            return;
        }

        this.idleMode = idleMode;

        leftPrimaryMotor.setIdleMode(idleMode);
        leftFollowerMotor.setIdleMode(idleMode);

        rightPrimaryMotor.setIdleMode(idleMode);
        rightFollowerMotor.setIdleMode(idleMode);
    }

    private void checkMotorSpeedLimits() {

        // If the arm is not retracted, then the max speed is limited

        if (armSubsystem.getArmExtendEncoder() >= ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME
            && DriverStation.isTeleopEnabled()) {

            // Scale each side of the robot to a max of .35
            double scalingFactor = 1.0;

            if (Math.abs(leftSpeed) > .35 || Math.abs(rightSpeed) > .35) {
                scalingFactor = .35 / Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            }

            leftSpeed  *= scalingFactor;
            rightSpeed *= scalingFactor;

            // Watch out for sharp turns
            limitTurning(0.35);
        }
        else {
            // Always limit the turning to 1.0
            limitTurning(1.0);
        }

    }

    private void limitTurning(double turnLimit) {

        if (Math.abs(leftSpeed - rightSpeed) > turnLimit) {

            double center = (leftSpeed + rightSpeed) / 2;

            if (leftSpeed > center) {
                leftSpeed = center + turnLimit / 2;
            }
            else {
                leftSpeed = center - turnLimit / 2;
            }

            if (rightSpeed > center) {
                rightSpeed = center + turnLimit / 2;
            }
            else {
                rightSpeed = center - turnLimit / 2;
            }
        }

    }

    /**
     * Get the error between the current heading and the requested heading in the
     * range -180 to +180 degrees.
     * <p>
     * A positive result means that the passed in heading is clockwise from the
     * current heading.
     *
     * @param requiredHeading to measure the heading error
     * @return degrees difference between the required heading and the current heading.
     */
    public double getHeadingError(double requiredHeading) {

        double currentHeading = getHeading();

        // Determine the error between the current heading and
        // the desired heading
        double error          = requiredHeading - currentHeading;

        if (error > 180) {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        return error;
    }

}
