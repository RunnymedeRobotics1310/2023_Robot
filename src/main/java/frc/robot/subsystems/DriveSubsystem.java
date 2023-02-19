package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final TalonSRX     leftPrimaryMotor         = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT);
    private final TalonSRX     leftFollowerMotor        = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT + 1);

    // The motors on the right side of the drive.
    private final TalonSRX     rightPrimaryMotor        = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT);
    private final TalonSRX     rightFollowerMotor       = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT + 1);

    private final DigitalInput targetSensor             = new DigitalInput(0);

    // Conversion from volts to distance in cm
    // Volts distance
    // 0.12 30.5 cm
    // 2.245 609.6 cm
    private final AnalogInput  ultrasonicDistanceSensor = new AnalogInput(0);

    private final double       ULTRASONIC_M             = (609.6 - 30.5) / (2.245 - .12);
    private final double       ULTRASONIC_B             = 609.6 - ULTRASONIC_M * 2.245;


    private double             leftSpeed                = 0;
    private double             rightSpeed               = 0;

    private AHRS               navXGyro                 = new AHRS() {
                                                            // Override the "Value" in the gyro
                                                            // sendable to use the angle instead of
                                                            // the yaw.
                                                            // Using the angle makes the gyro appear
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

    private double             zeroX                    = 0;
    private double             zeroY                    = 0;

    private double             gyroHeadingOffset        = 0;
    private double             gyroPitchOffset          = 0;
    private double             lastPitch                = 0;
    private double             pitchRate                = 0;

    private enum GyroAxis {
        YAW, PITCH, ROLL
    };

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        leftFollowerMotor.setNeutralMode(NeutralMode.Brake);

        leftFollowerMotor.follow(leftPrimaryMotor);


        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        rightFollowerMotor.setNeutralMode(NeutralMode.Brake);

        rightFollowerMotor.follow(rightPrimaryMotor);

        // Setting both encoders to 0
        resetEncoders();

    }

    /**
     * Calibrate Gyro
     * <p>
     * This routine calibrates the gyro. The robot must not be moved during the
     * calibrate routine which lasts about 10 seconds
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

        return heading;
    }

    private double getRawGyroAngle(GyroAxis gyroAxis) {

        switch (gyroAxis) {
        case YAW:
            return navXGyro.getYaw();
        case PITCH:
            return navXGyro.getPitch();
        case ROLL:
            return navXGyro.getRoll();
        default:
            return 0;
        }
    }

    public double getPitch() {

        double gyroPitch = getRawGyroAngle(GyroAxis.PITCH);

        // adjust by the offset that was saved when the gyro
        // pitch was last set.
        gyroPitch += gyroPitchOffset;

        return gyroPitch;
    }

    public double getPitchRate() {
        return pitchRate;
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderCounts() {
        return (getLeftEncoder() + getRightEncoder()) / 2;
    }

    public double getEncoderDistanceCm() {

        // FIXME: Use the NavX distance for now, but change this to encoder distance when possible
        double xCm = (navXGyro.getDisplacementX() - zeroX) * 100;
        double yCm = (navXGyro.getDisplacementY() - zeroY) * 100;

        return Math.round(Math.sqrt(xCm * xCm + yCm * yCm));

        // return getAverageEncoderCounts() * DriveConstants.INCHES_PER_ENCODER_COUNT;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return 0; // leftEncoder.getPosition();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return 0; // rightEncoder.getPosition();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {

        // FIXME: This routine captures the current NavX position (x, y)
        // which is used in the getDistanceCm.
        zeroX = navXGyro.getDisplacementX();
        zeroY = navXGyro.getDisplacementY();

        // rightEncoder.setPosition(0);
        // leftEncoder.setPosition(0);

        // FIXME: If using a NavX, pull the distance estimate off the NavX gyro.
        // For the reset routine, just store the current NavX value and
        // then return the delta from that value on all subsequent reads
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

        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        leftPrimaryMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightPrimaryMotor.set(ControlMode.PercentOutput, rightSpeed);

        // NOTE: The follower motors are set to follow the primary
        // motors
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    public boolean isTargetDetected() {
        return !targetSensor.get();
    }

    @Override
    public void periodic() {

        pitchRate = (getPitch() - lastPitch) * 50; // 50Hz
        lastPitch = getPitch();

        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left  Motor", leftSpeed);

        SmartDashboard.putNumber("Right Encoder", getRightEncoder());
        SmartDashboard.putNumber("Left Encoder", getLeftEncoder());

        SmartDashboard.putNumber("Distance (cm)", getEncoderDistanceCm());

        SmartDashboard.putNumber("Ultrasonic Voltage", ultrasonicDistanceSensor.getVoltage());
        SmartDashboard.putNumber("Ultrasonic Distance (cm)", getUltrasonicDistanceCm());

        SmartDashboard.putData("Gyro", navXGyro);

        // Put the displacements on the smartDashboard for testing (round to nearest cm)
        SmartDashboard.putNumber("NavX: X (cm)", Math.round(navXGyro.getDisplacementX() * 100));
        SmartDashboard.putNumber("NavX: Y (cm)", Math.round(navXGyro.getDisplacementY() * 100));
        SmartDashboard.putNumber("NavX: Z (cm)", Math.round(navXGyro.getDisplacementZ() * 100));

        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
    }
}
