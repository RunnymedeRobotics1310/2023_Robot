package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.VisionTarget.APRILTAG_GRID;
import static frc.robot.Constants.VisionConstants.VisionTarget.CONE_GROUND;
import static frc.robot.Constants.VisionConstants.VisionTarget.CONE_SUBSTATION;
import static frc.robot.Constants.VisionConstants.VisionTarget.CUBE_GROUND;
import static frc.robot.Constants.VisionConstants.VisionTarget.CUBE_SUBSTATION;
import static frc.robot.Constants.VisionConstants.VisionTarget.NONE;
import static frc.robot.Constants.VisionConstants.VisionTarget.POST_HIGH;
import static frc.robot.Constants.VisionConstants.VisionTarget.POST_LOW;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.Constants.VisionConstants.VisionTarget;

public class VisionSubsystem extends SubsystemBase {

    private static final long            LED_MODE_PIPELINE           = 0;
    private static final long            LED_MODE_OFF                = 1;
    private static final long            LED_MODE_BLINK              = 2;
    private static final long            LED_MODE_ON                 = 3;

    private static final long            CAM_MODE_VISION             = 0;
    private static final long            CAM_MODE_DRIVER             = 1;

    // configure more pipelines here
    private static final long            PIPELINE_CONE_DETECT        = 0;
    private static final long            PIPELINE_CUBE_DETECT        = 1;
    private static final long            PIPELINE_APRIL_TAG_DETECT   = 3;
    private static final long            PIPELINE_POST_DETECT        = 4;

    private static final LinearFilter    CONE_LOW_PASS_FILTER        = LinearFilter.singlePoleIIR(.1, .02);
    private static final int             CAMERA_UP_LIMIT_SWITCH_PORT = 1;

    NetworkTable                         table                       = NetworkTableInstance.getDefault().getTable("limelight");

    // inputs/configs
    NetworkTableEntry                    ledMode                     = table.getEntry("ledMode");
    NetworkTableEntry                    camMode                     = table.getEntry("camMode");
    NetworkTableEntry                    pipeline                    = table.getEntry("pipeline");

    // output
    NetworkTableEntry                    tv                          = table.getEntry("tv");
    NetworkTableEntry                    tx                          = table.getEntry("tx");
    NetworkTableEntry                    ty                          = table.getEntry("ty");
    NetworkTableEntry                    ta                          = table.getEntry("ta");
    NetworkTableEntry                    tl                          = table.getEntry("tl");

    private boolean                      isCameraPositionInitialized = false;

    private VisionConstants.VisionTarget currentVisionTarget         = NONE;

    private double                       filteredConeAngle           = 0;

    /*
     * Camera motor and encoder
     */
    private final CANSparkMax            cameraMotor                 = new CANSparkMax(VisionConstants.CAMERA_ANGLE_MOTOR_PORT,
        MotorType.kBrushless);

    // Arm lift encoder
    private final RelativeEncoder        cameraEncoder               = cameraMotor.getEncoder();

    private double                       cameraMotorSpeed            = 0;

    private double                       cameraEncoderOffset         = 0;

    private DigitalInput                 cameraUpLimitSwitch         = new DigitalInput(
        VisionSubsystem.CAMERA_UP_LIMIT_SWITCH_PORT);

    public VisionSubsystem() {

        // Set the max current on the camera Neo550 to 20A. This will prevent the motor
        // from burning out when stalled.
        // See https://www.revrobotics.com/neo-550-brushless-motor-locked-rotor-testing/
        cameraMotor.setSmartCurrentLimit(VisionConstants.CAMERA_MOTOR_CURRENT_LIMIT);

        cameraMotor.setIdleMode(IdleMode.kBrake);

        // When the robot starts, the camera must be set to the high view (0)
        setCameraEncoderPosition(0);
        isCameraPositionInitialized = false;
    }

    public double getTargetAreaPercent() {
        return ta.getDouble(-1.0);
    }

    public VisionConstants.VisionTarget getCurrentVisionTarget() {
        return currentVisionTarget;
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

        if (Math
            .abs(cameraEncoderPosition - VisionConstants.CAMERA_MID_ENCODER_VALUE) < VisionConstants.CAMERA_POSITION_TOLERANCE) {
            return CameraView.MID;
        }

        if (Math.abs(
            cameraEncoderPosition - VisionConstants.CAMERA_DOWN_LIMIT_ENCODER_VALUE) < VisionConstants.CAMERA_POSITION_TOLERANCE
            || cameraEncoderPosition < VisionConstants.CAMERA_DOWN_LIMIT_ENCODER_VALUE) {

            return CameraView.LOW;
        }

        return CameraView.IN_BETWEEN;
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
     * Return true if the camera is in position for its current vision target.
     *
     * @return true if the camera is in the right position for the specified target.
     */
    public boolean isCameraInPositionForTarget() {
        return getCameraView() == currentVisionTarget.getCameraView();
    }

    /**
     * Get the Target Angle Offset in degrees
     * <p>
     * Check whether a target is acquired using {@link #isVisionTargetFound()}
     *
     * @return degrees in horizontal angle offset from the current crosshairs. or {@code 0} if no
     * target is currently found.
     */
    public double getTargetAngleOffset() {

        if (!isVisionTargetFound()) {
            return 0;
        }

        // Return the angle offset based on a the target
        if (getCurrentVisionTarget() == VisionTarget.CUBE_GROUND) {

            // CUBE offset measurements.

            // At a y value of +17, the x offset is +7
            // At a y value of -14, the x offset is +10

            return getTargetX() - 10 + (14 + getTargetY()) / 10;
        }
        else if (getCurrentVisionTarget() == VisionTarget.APRILTAG_GRID) {
            return getTargetX() - 10;
        }

        // FIXME: return the filtered cone value if a cone.

        return getTarget()[0];
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
     * Use {@link #setVisionTarget(VisionConstants.VisionTarget)} to set the vision target type
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

        cameraMotorSpeed = checkCameraMotorLimits(speed);
        cameraMotor.set(cameraMotorSpeed);
    }

    public void setVisionTarget(VisionConstants.VisionTarget visionTarget) {

        this.currentVisionTarget = visionTarget;

        setCameraView(visionTarget.getCameraView());

        switch (visionTarget) {
        case CONE_GROUND:
        case CONE_SUBSTATION:
            this.pipeline.setNumber(PIPELINE_CONE_DETECT);
            this.camMode.setNumber(CAM_MODE_VISION);
            this.ledMode.setNumber(LED_MODE_PIPELINE);
            break;
        case CUBE_GROUND:
        case CUBE_SUBSTATION:
            this.pipeline.setNumber(PIPELINE_CUBE_DETECT);
            this.camMode.setNumber(CAM_MODE_VISION);
            this.ledMode.setNumber(LED_MODE_PIPELINE);
            break;
        case APRILTAG_GRID:
            this.pipeline.setNumber(PIPELINE_APRIL_TAG_DETECT);
            this.camMode.setNumber(CAM_MODE_VISION);
            this.ledMode.setNumber(LED_MODE_PIPELINE);
            break;
        case POST_LOW:
        case POST_HIGH:
            this.pipeline.setNumber(PIPELINE_POST_DETECT);
            this.camMode.setNumber(CAM_MODE_VISION);
            this.ledMode.setNumber(LED_MODE_PIPELINE);
            break;
        case FIELD:
        case NONE:
        default:
            this.camMode.setInteger(CAM_MODE_DRIVER);
            this.ledMode.setInteger(LED_MODE_OFF);
            break;
        }
    }

    @Override
    public void periodic() {

        if (currentVisionTarget == CONE_GROUND || currentVisionTarget == CONE_SUBSTATION) {
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
        SmartDashboard.putBoolean("Cube",
            (currentVisionTarget == CUBE_GROUND || currentVisionTarget == CUBE_SUBSTATION) && isVisionTargetFound());
        SmartDashboard.putBoolean("Cone",
            (currentVisionTarget == CONE_GROUND || currentVisionTarget == CONE_SUBSTATION) && isVisionTargetFound());
        SmartDashboard.putBoolean("Post Low", currentVisionTarget == POST_LOW && isVisionTargetFound());
        SmartDashboard.putBoolean("Post High", currentVisionTarget == POST_HIGH && isVisionTargetFound());
        SmartDashboard.putBoolean("Tag", currentVisionTarget == APRILTAG_GRID && isVisionTargetFound());
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
        SmartDashboard.putNumber("Camera Motor Speed", cameraMotorSpeed);
        SmartDashboard.putNumber("Camera Encoder", Math.round(getCameraEncoder() * 100) / 100d);
        SmartDashboard.putBoolean("Camera Upper Limit", getCameraUpperLimitSwitch());

        SmartDashboard.putNumber("Cone Angle Filtered", filteredConeAngle);
    }

    public void stop() {

        // Safely stop all motors
        cameraMotor.set(0);
    }

    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append("Vision Target ").append(getCurrentVisionTarget())
            .append(", Camera View ").append(getCameraView())
            .append(", Camera in position ").append(isCameraInPositionForTarget())
            .append(", Target Detected ").append(isVisionTargetFound())
            .append(", target angle ").append(Math.round(getTargetAngleOffset() * 10) / 10d);

        return sb.toString();
    }

    /**
     * Get the limelight coordinates for the target (i.e. with respect to the limelight origin, NOT
     * the robot!!)
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
     * Get the limelight X angle measurement to the target.
     *
     * @return limelight X target coordinates
     */
    private double getTargetX() {
        return tx.getDouble(-1.0);
    }

    /**
     * Get the limelight Y angle measurement to the target.
     *
     * @return limelight Y target coordinates
     */
    private double getTargetY() {
        return ty.getDouble(-1.0);
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

    private void initializeCameraPosition() {

        // Wait until the robot is enabled to initialize the camera
        if (!DriverStation.isAutonomousEnabled() && !DriverStation.isTeleopEnabled()) {
            return;
        }

        // Drive up until the camera touches the limit
        cameraMotor.set(.3);

        if (getCameraUpperLimitSwitch()) {

            // Turn off the motor
            cameraMotor.set(0);

            // Initialize the encoder so that zero is slightly down from the hard stop.
            setCameraEncoderPosition(10);

            isCameraPositionInitialized = true;
        }
    }

    private boolean getCameraUpperLimitSwitch() {
        return !cameraUpLimitSwitch.get();
    }

    private void setCameraView(CameraView desiredView) {

        CameraView currentCameraView = getCameraView();

        if (currentCameraView == desiredView) {

            // Camera is already in position.
            setCameraMotorSpeed(0);

        }
        else if (desiredView == CameraView.HIGH) {

            // Run the motor forward to raise the camera view
            setCameraMotorSpeed(VisionConstants.MAX_CAMERA_MOTOR_SPEED);

        }
        else if (desiredView == CameraView.LOW){

            // Run the motor in reverse to lower the camera view
            setCameraMotorSpeed(-VisionConstants.MAX_CAMERA_MOTOR_SPEED);

        } else if (desiredView == CameraView.MID) {
            double cameraEncoderPosition = getCameraEncoder();

            if (cameraEncoderPosition < VisionConstants.CAMERA_MID_ENCODER_VALUE) {
                setCameraMotorSpeed(VisionConstants.MAX_CAMERA_MOTOR_SPEED);
            } else {
                setCameraMotorSpeed(-VisionConstants.MAX_CAMERA_MOTOR_SPEED);
            }
        }
    }

    /**
     * Set the camera encoder to the supplied position
     */
    private void setCameraEncoderPosition(double cameraEncoderPosition) {
        cameraEncoderOffset = cameraEncoderPosition - cameraEncoder.getPosition();
    }
}
