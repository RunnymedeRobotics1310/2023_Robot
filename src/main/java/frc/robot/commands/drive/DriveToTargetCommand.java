

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class DriveToTargetCommand extends CommandBase {

    final double                   factor                 = 0.01;

    private final double           speed, timeoutSeconds;

    private final DriveSubsystem   driveSubsystem;
    private final VisionSubsystem  visionSubsystem;

    private final VisionTargetType targetType;

    private long                   initializeTime         = 0;

    private double                 targetDelaySec         = 0;

    private boolean                targetFound            = false;

    private double                 lastKnownTargetHeading = 0;

    /**
     * Drive to a cube vision target. If this command does not find a cube vision target,
     * then the command ends after 1 second.
     * <p>
     * This constructor uses the {@link Constants#DEFAULT_COMMAND_TIMEOUT_SECONDS} for the command
     * timeout
     *
     * @param speed in the range 0-1.0
     * @param driveSubsystem
     * @param visionSubsystem
     */
    public DriveToTargetCommand(VisionTargetType targetType, double speed,
        DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(targetType, speed, Constants.DEFAULT_COMMAND_TIMEOUT_SECONDS, driveSubsystem, visionSubsystem);
    }

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in cm.
     *
     * @param speed in the range 0-1.0
     * @param timeoutSeconds to stop this command if the target has not been reached
     * @param driveSubsystem
     * @param visionSubsystem
     */
    public DriveToTargetCommand(VisionTargetType targetType, double speed, double timeoutSeconds,
        DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

        this.targetType      = targetType;
        this.speed           = speed;
        this.timeoutSeconds  = timeoutSeconds;
        this.driveSubsystem  = driveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(driveSubsystem, visionSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("DriveToTargetCommand started."
            + " Target " + targetType
            + " Speed " + speed
            + ", timeout " + timeoutSeconds);

        initializeTime = System.currentTimeMillis();

        if (visionSubsystem.getCurrentVisionTargetType() != targetType) {
            targetDelaySec = VisionConstants.VISION_SWITCH_TIME_SEC;
            visionSubsystem.setVisionTargetType(targetType);
        }
        else {
            targetDelaySec = 0;
        }

        // Start by driving straight towards the target.
        // Assume the robot is lined up with the target.
        targetFound = false;
        driveSubsystem.setMotorSpeeds(speed, speed);
    }

    @Override
    public void execute() {

        double executeTimeSec = (System.currentTimeMillis() - initializeTime) / 1000.0d;

        // If the target was switched, then wait before trying to track a target
        if (executeTimeSec < targetDelaySec) {
            return;
        }

        if (visionSubsystem.isVisionTargetFound()) {

            // FIXME: Is this correct - how do we get the angle to the target?
            lastKnownTargetHeading  = driveSubsystem.getHeading() + visionSubsystem.getTargetAngleOffset();

            lastKnownTargetHeading %= 360.0d;

            if (lastKnownTargetHeading < 0) {
                lastKnownTargetHeading += 360;
            }

            // The first time a target is found, print out the heading
            if (!targetFound) {
                System.out.println(this.getClass().getSimpleName()
                    + ": First target sighting at heading " + lastKnownTargetHeading);
                targetFound = true;
            }
        }

        // If a target has never been found, then keep driving straight.
        // Hopefully a target will be found on the next lap
        if (!targetFound) {
            driveSubsystem.setMotorSpeeds(speed, speed);
            return;
        }

        // Track the last known target heading.
        // Determine the error between the current heading and the last known target heading

        double currentHeading = driveSubsystem.getHeading();

        double error          = lastKnownTargetHeading - currentHeading;

        if (error > 180) {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        // FIXME: Should the robot slow down if it knows how far to the target
        // Note: cubes are not as sensitive as cones which could be tipped over.

        double leftSpeed  = speed + error * factor;
        double rightSpeed = speed - error * factor;

        // In the end, set the speeds on the motors
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {

        // This command can either reach the distance or time out

        // Check the timeout
        if ((System.currentTimeMillis() - initializeTime) / 1000d > timeoutSeconds) {
            return true;
        }

        // Stop when the claw detects the cube.
        // FIXME: The detector should be in the claw subsystem.
        // if (driveSubsystem.isTargetDetected() || visionSubsystem.isVisionTargetClose()) {
        // return true;
        // }

        double targetArea = visionSubsystem.getTargetAreaPercent();
        switch (targetType) {
        case CONE:
            if (targetArea >= 20) {
                return true;
            }
            break;
        case CUBE:
            if (targetArea >= 60) {
                return true;
            }
            break;
        case TAG:
            if (targetArea >= 5) {
                return true;
            }
            break;
        default:
            // ???
            break;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        // Print an end of command message in the logs
        double runTime = (System.currentTimeMillis() - initializeTime) / 1000d;

        if (interrupted) {
            System.out.print(this.getClass().getSimpleName() + " interrupted");
        }
        else {
            System.out.print(this.getClass().getSimpleName() + " ended");
        }

        System.out.println(": vision target detected " + targetFound
            + ": current heading " + driveSubsystem.getHeading()
            + ": in " + runTime + "s"
            + ": area " + visionSubsystem.getTargetAreaPercent());

        // Stop the robot
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}
