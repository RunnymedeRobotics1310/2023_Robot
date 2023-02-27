package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraView;

public class VisionSubsystem extends SubsystemBase {

    public enum VisionTargetType {
        CUBE, CONE, TAG, CONE_POST, NONE
    };

    private static final long LED_MODE_PIPELINE         = 0;
    private static final long LED_MODE_OFF              = 1;
    private static final long LED_MODE_BLINK            = 2;
    private static final long LED_MODE_ON               = 3;

    private static final long CAM_MODE_VISION           = 0;
    private static final long CAM_MODE_DRIVER           = 1;

    // configure more pipelines here
    private static final long PIPELINE_CONE_DETECT      = 0;
    private static final long PIPELINE_CUBE_DETECT      = 1;
    private static final long PIPELINE_APRIL_TAG_DETECT = 3;


    // calibration data
    private double[]          topLeft                   = new double[2];
    private double[]          topRight                  = new double[2];
    private double[]          bottomRight               = new double[2];
    private double[]          bottomLeft                = new double[2];

    NetworkTable              table                     = NetworkTableInstance.getDefault().getTable("limelight");

    // inputs/configs
    NetworkTableEntry         ledMode                   = table.getEntry("ledMode");
    NetworkTableEntry         camMode                   = table.getEntry("camMode");
    NetworkTableEntry         pipeline                  = table.getEntry("pipeline");

    // output
    NetworkTableEntry         tv                        = table.getEntry("tv");
    NetworkTableEntry         tx                        = table.getEntry("tx");
    NetworkTableEntry         ty                        = table.getEntry("ty");
    NetworkTableEntry         ta                        = table.getEntry("ta");
    NetworkTableEntry         tl                        = table.getEntry("tl");

    private VisionTargetType  currentVisionTargetType   = VisionTargetType.NONE;

    /*
     * Camera motor and encoder
     */
    private final CANSparkMax cameraMotor               = new CANSparkMax(VisionConstants.CAMERA_ANGLE_MOTOR_PORT,
        MotorType.kBrushless);

    private double            cameraMotorSpeed          = 0;

    // Arm lift encoder
    private RelativeEncoder   cameraEncoder             = cameraMotor.getEncoder();

    private double            cameraEncoderOffset       = 0;

    private CameraView        cameraView                = CameraView.LOW;

    public VisionSubsystem() {
        setCameraView(CameraView.LOW);
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
     * etc.
     * Using these values, set the four corners of the field of view of the limelight
     *
     * @param topLeft
     * @param topRight
     * @param bottomRight
     * @param bottomLeft
     */
    public void calibrateVision(double[] topLeft, double[] topRight, double[] bottomRight, double[] bottomLeft) {
        this.topLeft     = topLeft;
        this.topRight    = topRight;
        this.bottomRight = bottomRight;
        this.bottomLeft  = bottomLeft;
    }

    @Override
    public void periodic() {

        // read values periodically and post to smart dashboard periodically
        SmartDashboard.putBoolean("Limelight Target Found", isVisionTargetFound());
        SmartDashboard.putBoolean("Cube", currentVisionTargetType == VisionTargetType.CUBE && isVisionTargetFound());
        SmartDashboard.putBoolean("Cone", currentVisionTargetType == VisionTargetType.CONE && isVisionTargetFound());
        SmartDashboard.putBoolean("Post", currentVisionTargetType == VisionTargetType.CONE_POST && isVisionTargetFound());
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

        SmartDashboard.putString("Camera view", cameraView.toString());
        SmartDashboard.putNumber("Camera Motor Speed", getCameraMotorSpeed());
        SmartDashboard.putNumber("Camera Encoder", getCameraEncoder());
    }

    /**
     * Get the limelight coordinates for the target
     * (i.e. with respect to the limelight origin, NOT the robot!!)
     *
     * @return limelight target coordinates
     */
    private double[] getTarget() {
        double[] d = new double[2];
        d[0] = tx.getDouble(-1.0);
        d[1] = ty.getDouble(-1.0);
        return d;
    }

    public double getTargetAreaPercent() {
        return ta.getDouble(-1.0);
    }

    public VisionTargetType getCurrentVisionTargetType() {
        return currentVisionTargetType;
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

        case CONE_POST:
            // FIXME: Implement a post detection pipe
            break;

        default:
            System.out.println("Invalid value used for "
                + "VisionSubsystem.setVisionTargetType("
                + visionTargetType + ")");
        }

    }

    /**
     * Get the Target Angle Offset in degrees
     * <p>
     * Check whether a target is acquired using {@link #isVisionTargetFound()}
     *
     * @return degrees in horizontal angle offset from the current crosshairs.
     * or {@code 0} if no target is currently found.
     */
    public double getTargetAngleOffset() {

        if (!isVisionTargetFound()) {
            return 0;
        }

        return tx.getDouble(0);
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

    public boolean isConeTargetAcquired() {
        // todo: fixme
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

        // todo: fixme: more checks
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

    public boolean isCubeTargetAcquired() {
        // todo: fixme
        if (PIPELINE_CUBE_DETECT != pipeline.getInteger(-1)) {
            return false;
        }

        // Check that a target it acquired.
        if (!isVisionTargetFound()) {
            return false;
        }

        // is the target area larger than minPercentForConeAcquisition of the screen?
        long minPercentForCubeAcquisition = 20;
        if (getTargetAreaPercent() < minPercentForCubeAcquisition) {
            return false;
        }

        double[] tgt = getTarget();
        if (tgt[0] < 0 || tgt[1] < 0)
            return false;

        // todo: fixme: more checks
        return true;
    }

    public boolean isVisionTargetClose() {
        // todo: fixme
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

    public CameraView getCameraView() {
        return cameraView;
    }

    /**
     * Set the camera motor speed
     *
     * @param speed
     */
    public void setCameraMotorSpeed(double speed) {

        cameraMotorSpeed = speed;

        cameraMotor.set(speed);
    }

    /**
     * Get the camera motor speed
     *
     * @param speed
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
     * Set the camera encoder to the supplied position
     *
     * @param cameraEncoderPosition
     */
    public void setCameraEncoderPosition(double cameraEncoderPosition) {
        cameraEncoderOffset = -cameraEncoder.getPosition();
    }

    public void setCameraView(CameraView cameraView) {

        if (this.cameraView == cameraView) {
            return;
        }

        // FIXME: Should this be here?
        // SetCameraView should be a command?

        this.cameraView = cameraView;
    }

    public double getTargetOffset() {
        // todo: fixme: do proper trigonometry and compute the offset in
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

    public void stop() {

        // TODO: Safely stop all motors
    }
}
