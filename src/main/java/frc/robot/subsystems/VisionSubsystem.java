package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraView;

public class VisionSubsystem extends SubsystemBase {

    public enum VisionTargetType {
        CUBE, CONE, TAG, CONE_POST_LOW, CONE_POST_HIGH, NONE
    }

    private static final long LED_MODE_PIPELINE = 0;
    private static final long LED_MODE_OFF      = 1;
    private static final long LED_MODE_BLINK    = 2;
    private static final long LED_MODE_ON       = 3;

    private static final long CAM_MODE_VISION = 0;
    private static final long CAM_MODE_DRIVER = 1;

    // configure more pipelines here
    private static final long PIPELINE_CONE_DETECT      = 0;
    private static final long PIPELINE_CUBE_DETECT      = 1;
    private static final long PIPELINE_APRIL_TAG_DETECT = 3;

    private static final LinearFilter CONE_LOW_PASS_FILTER = LinearFilter.singlePoleIIR(.1, .02);

    // calibration data
    private double[] topLeft     = new double[2];
    private double[] topRight    = new double[2];
    private double[] bottomRight = new double[2];
    private double[] bottomLeft  = new double[2];


    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // inputs/configs
    NetworkTableEntry ledMode  = table.getEntry("ledMode");
    NetworkTableEntry camMode  = table.getEntry("camMode");
    NetworkTableEntry pipeline = table.getEntry("pipeline");

    // output
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tl = table.getEntry("tl");

    private boolean isCameraPositionInitialized   = false;
    private long    cameraInitializationStartTime = 0;

    private VisionTargetType currentVisionTargetType = VisionTargetType.NONE;

    private double filteredConeAngle = 0;

    /*
     * Camera motor and encoder
     */
    private final CANSparkMax cameraMotor = new CANSparkMax(VisionConstants.CAMERA_ANGLE_MOTOR_PORT,
        MotorType.kBrushless);

    private double cameraMotorSpeed = 0; // todo: used?

    // Arm lift encoder
    private final RelativeEncoder cameraEncoder = cameraMotor.getEncoder();

    private double cameraEncoderOffset = 0;

    public VisionSubsystem() {

        // Set the max current on the camera Neo550 to 20A. This will prevent the motor
        // from burning out when stalled.
        // See https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing/
        cameraMotor.setSmartCurrentLimit(VisionConstants.CAMERA_MOTOR_CURRENT_LIMIT);

        // When the robot starts, the camera must be set to the high view (0)
        setCameraEncoderPosition(0);
        isCameraPositionInitialized = false;
    }

    /**
     * Tell the vision subsystem the coordinates that it can see (on the floor).
     *
     * <pre>
     * {0, 0} corresponds to the ground directly at the front bumper in the center of the robot
     * {-10, 0} corresponds to a location against the front bumper 10cm to the left of the robot center
     * {10, 0} corresponds to a location against the front bumper 10cm to the right of the robot center
     * {10, 10} corresponds to a location 10cm away from the front bumper of the robot, 10cm to the right of center
     * </pre>
     *
     * etc. Using these values, set the four corners of the field of view of the limelight
     */
    public void calibrateVision(double[] topLeft, double[] topRight, double[] bottomRight, double[] bottomLeft) {
        this.topLeft     = topLeft;
        this.topRight    = topRight;
        this.bottomRight = bottomRight;
        this.bottomLeft  = bottomLeft;
    }

    public double getTargetAreaPercent() {
        return ta.getDouble(-1.0);
    }

    public VisionTargetType getCurrentVisionTargetType() {
        return currentVisionTargetType;
    }

    public CameraView getCameraView() {

        // NOTE: The camera encoder position will be a negative number because the
        // positive direction is a higher camera angle and the negative direction
        // is a lower camera angle. The max camera angle is 0, so this encoder always
        // has a negative value.

        double cameraEncoderPosition = getCameraEncoder();

        // Camera high position = 0 encoder value
        if (Math.abs(cameraEncoderPosition) < VisionConstants.CAMERA_POSITION_TOLERANCE
            || cameraEncoderPosition > 0) {

            return CameraView.HIGH;
        }

        if (Math.abs(
            cameraEncoderPosition - VisionConstants.CAMERA_DOWN_LIMIT_ENCODER_VALUE) < VisionConstants.CAMERA_POSITION_TOLERANCE
            || cameraEncoderPosition < VisionConstants.CAMERA_DOWN_LIMIT_ENCODER_VALUE) {

            return CameraView.LOW;
        }

        return CameraView.IN_BETWEEN;
    }

    /**
     * Get the camera motor speed
     */
    public double getCameraMotorSpeed() {

        return cameraMotorSpeed;
    }

    /**
     * Get the current camera encoder position
     *
     * @return double encoder position
     */
    public double getCameraEncoder() {
        return cameraEncoder.getPosition() + cameraEncoderOffset;
    }

    /**
     * Get the Target Angle Offset in degrees
     * <p>
     * Check whether a target is acquired using {@link #isVisionTargetFound()}
     *
     * @return degrees in horizontal angle offset from the current crosshairs. or {@code 0} if no target is currently found.
     */
    public double getTargetAngleOffset() {

        if (!isVisionTargetFound()) {
            return 0;
        }

        // FIXME: return the filtered cone value if a cone.

        return tx.getDouble(0);
    }

    public double getTargetOffset() {
        // fixme: do proper trigonometry and compute the offset in
        // degrees between the target and "straight ahead".
        // For now, the code will just return -10 if it's to the left of
        // center, +10 if it's to the right of center, but we should be
        // able to be much more precise than this
        if (isConeTargetAcquired() || isCubeTargetAcquired()) {
            // note... we MIGHT switch between having a valid target and not
            // having a valid target between the is*TargetAcquired() and the
            // subsequent call to getTarget(). Measure to see if this is a
            // problem, and if so, code more defensively.
            double[] tgt = getTarget();
            if (tgt[0] < 0)
                return -3.0;
            if (tgt[0] > 0)
                return 3.0;
            return 0.0;
        }
        else {
            throw new IllegalStateException("Cannot get an offset because no target has been acquired");
        }
    }

    public boolean isConeTargetAcquired() {
        // fixme: finish this
        if (PIPELINE_CONE_DETECT != pipeline.getInteger(-1)) {
            return false;
        }

        // is the target area larger than minPercentForConeAcquisition of the screen?
        long minPercentForConeAcquisition = 15;
        if (getTargetAreaPercent() < minPercentForConeAcquisition) {
            return false;
        }

        double[] tgt = getTarget();
        if (tgt[0] < 0 || tgt[1] < 0)
            return false;

        // fixme: more checks
        return true;
    }

    public boolean isCubeTargetAcquired() {
        // fixme: finish this
        if (PIPELINE_CUBE_DETECT != pipeline.getInteger(-1)) {
            return false;
        }

        // Check that a target it acquired.
        if (!isVisionTargetFound()) {
            return false;
        }

        // is the target area larger than minPercentForConeAcquisition of the screen?
        long minPercentForCubeAcquisition = 6;
        if (getTargetAreaPercent() < minPercentForCubeAcquisition) {
            return false;
        }

        double[] tgt = getTarget();
        if (tgt[0] < 0 || tgt[1] < 0)
            return false;

        // fixme: more checks
        return true;
    }

    /**
     * Determine if a vision target of the current type is found.
     * <p>
     * Use {@link #setVisionTargetType(VisionTargetType)} to set the vision target type
     */
    public boolean isVisionTargetFound() {
        return tv.getDouble(-1) == 1;
    }

    public boolean isVisionTargetClose() {
        // fixme: finish this
        if (PIPELINE_APRIL_TAG_DETECT != pipeline.getInteger(-1)) {
            return false;
        }
        double pct = getTargetAreaPercent();
        if (isVisionTargetFound() && pct > 10) {
            System.out.println("Vision target found and target area is " + pct + " which tells us we are close to the target");
            return true;
        }
        return false;

    }

    /**
     * Set the camera motor speed
     */
    public void setCameraMotorSpeed(double speed) {

        // Initialize the camera motor speed
        if (!isCameraPositionInitialized) {
            initializeCameraPosition();
            return;
        }

        // FIXME: If the initialize works, then remove this return statement.
        return;

        // cameraMotorSpeed = checkCameraMotorLimits(speed);

        // cameraMotor.set(cameraMotorSpeed);
    }

    public void initializeCameraPosition() {

        // Wait until the robot is enabled to initialize the camera
        if (!DriverStation.isAutonomousEnabled() && !DriverStation.isTeleopEnabled()) {
            return;
        }

        if (cameraInitializationStartTime == 0) {
            cameraInitializationStartTime = System.currentTimeMillis();
        }

        cameraMotor.set(.3);

        // End after 3 seconds
        if ((System.currentTimeMillis() - cameraInitializationStartTime) > 3000) {
            cameraMotor.set(0);
            isCameraPositionInitialized = true;
            cameraEncoder.setPosition(2); // Above the top limit.
        }
    }

    /**
     * Set the camera encoder to the supplied position
     */
    public void setCameraEncoderPosition(double cameraEncoderPosition) {
        cameraEncoderOffset = -cameraEncoder.getPosition();
    }

    public void setModeConeAcquisition() {
        this.pipeline.setNumber(PIPELINE_CONE_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
        this.ledMode.setNumber(LED_MODE_PIPELINE);
    }

    public void setModeCubeAcquisition() {
        this.pipeline.setNumber(PIPELINE_CUBE_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
        this.ledMode.setNumber(LED_MODE_PIPELINE);
    }

    public void setModeAprilTags() {
        this.pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
        this.camMode.setNumber(CAM_MODE_VISION);
        this.ledMode.setNumber(LED_MODE_PIPELINE);
    }

    public void setModeDriver() {
        this.camMode.setInteger(CAM_MODE_DRIVER);
        this.ledMode.setInteger(LED_MODE_OFF);
    }

    /**
     * Set the current vision target type
     *
     * @param visionTargetType
     */
    public void setVisionTargetType(VisionTargetType visionTargetType) {

        currentVisionTargetType = visionTargetType;

        switch (visionTargetType) {
        case CONE:
            setModeConeAcquisition();
            break;

        case CUBE:
            setModeCubeAcquisition();
            break;

        case TAG:
            setModeAprilTags();
            break;

        case CONE_POST_LOW:
            // FIXME: Implement low post detection pipe
            break;

        case CONE_POST_HIGH:
            // FIXME: Implement high post detection pipe
            break;

        default:
            System.out.println("Invalid value used for "
                + "VisionSubsystem.setVisionTargetType("
                + visionTargetType + ")");
        }

    }

    @Override
    public void periodic() {

        if (currentVisionTargetType == VisionTargetType.CONE) {
            if (isVisionTargetFound()) {
                filteredConeAngle = CONE_LOW_PASS_FILTER.calculate(getTargetAngleOffset());
            }
            else {
                filteredConeAngle = 0;
                CONE_LOW_PASS_FILTER.reset();
            }
        }

        // Call the safety code on the camera motor movement
        setCameraMotorSpeed(cameraMotorSpeed);

        // read values periodically and post to smart dashboard periodically
        SmartDashboard.putBoolean("Limelight Target Found", isVisionTargetFound());
        SmartDashboard.putBoolean("Cube", currentVisionTargetType == VisionTargetType.CUBE && isVisionTargetFound());
        SmartDashboard.putBoolean("Cone", currentVisionTargetType == VisionTargetType.CONE && isVisionTargetFound());
        SmartDashboard.putBoolean("Post Low", currentVisionTargetType == VisionTargetType.CONE_POST_LOW && isVisionTargetFound());
        SmartDashboard.putBoolean("Post High",
            currentVisionTargetType == VisionTargetType.CONE_POST_HIGH && isVisionTargetFound());
        SmartDashboard.putBoolean("Tag", currentVisionTargetType == VisionTargetType.TAG && isVisionTargetFound());
        SmartDashboard.putNumber("Limelight tx-value", tx.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight ty-value", ty.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight ta-value", ta.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight l-value", tl.getDouble(-1.0));
        SmartDashboard.putNumber("Limelight Cam Mode", camMode.getInteger(-1L));
        SmartDashboard.putNumber("Limelight LED mode", ledMode.getInteger(-1L));
        SmartDashboard.putNumber("Limelight Pipeline", pipeline.getInteger(-1L));
        SmartDashboard.putBoolean("Cone Targed Acquired", isConeTargetAcquired());
        SmartDashboard.putBoolean("Cube Targed Acquired", isCubeTargetAcquired());

        SmartDashboard.putString("Camera view", getCameraView().toString());
        SmartDashboard.putNumber("Camera Motor Speed", getCameraMotorSpeed());
        SmartDashboard.putNumber("Camera Encoder", Math.round(getCameraEncoder() * 100) / 100d);

        SmartDashboard.putNumber("Cone Angle Filtered", filteredConeAngle);
    }

    public void stop() {

        // Safely stop all motors
        cameraMotor.set(0);
    }

    /**
     * Get the limelight coordinates for the target (i.e. with respect to the limelight origin, NOT the robot!!)
     *
     * @return limelight target coordinates
     */
    private double[] getTarget() {
        double[] d = new double[2];
        d[0] = tx.getDouble(-1.0);
        d[1] = ty.getDouble(-1.0);
        return d;
    }

    /**
     * Check the camera motor limits and return the appropriate output speed based on the limits
     *
     * @return output speed for the camera motor based on the current camera position.
     */
    private double checkCameraMotorLimits(double inputSpeed) {

        /*
         * High
         */
        if (inputSpeed > 0) {

            if (getCameraView() == CameraView.HIGH) {

                return 0;
            }

            // If we are getting close to the limit, then slow down
            if (Math.abs(getCameraEncoder()) < VisionConstants.CAMERA_POSITION_SLOW_ZONE) {

                return Math.min(inputSpeed, VisionConstants.MAX_CAMERA_SLOW_ZONE_SPEED);
            }
        }

        /*
         * Low
         */

        if (inputSpeed < 0) {

            if (getCameraView() == CameraView.LOW) {

                return 0;
            }

            // If we are getting close to the limit, then slow down
            if (Math.abs(
                getCameraEncoder()
                    - VisionConstants.CAMERA_DOWN_LIMIT_ENCODER_VALUE) < VisionConstants.CAMERA_POSITION_SLOW_ZONE) {

                return Math.max(inputSpeed, -VisionConstants.MAX_CAMERA_SLOW_ZONE_SPEED);
            }
        }

        return inputSpeed;
    }
}
