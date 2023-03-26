

package frc.robot.commands.drive;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.VisionTarget;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.commands.vision.SetVisionTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToTargetCommand extends RunnymedeCommandBase {

    private static final List<VisionTarget>    SUPPORTED_DRIVE_TARGETS = Arrays.asList(
        VisionTarget.CONE_GROUND,
        VisionTarget.CUBE_GROUND,
        VisionTarget.APRILTAG_GRID,
        VisionTarget.POST_HIGH,
        VisionTarget.POST_LOW);

    final double                               factor                  = 0.01;

    private final double                       speed, timeoutSeconds;

    private final DriveSubsystem               driveSubsystem;
    private final VisionSubsystem              visionSubsystem;
    private final ArmSubsystem                 armSubsystem;

    private final VisionConstants.VisionTarget target;

    private long                               initializeTime          = 0;

    private boolean                            targetFound             = false;

    private double                             lastKnownTargetHeading  = 0;

    private double                             startingToDriveAimlesslyEncoderCount = -1.0;

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
    public DriveToTargetCommand(VisionConstants.VisionTarget target, double speed,
        DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem) {
        this(target, speed, Constants.DEFAULT_COMMAND_TIMEOUT_SECONDS, driveSubsystem, visionSubsystem, armSubsystem);
    }

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in cm.
     *
     * @param speed in the range 0-1.0
     * @param timeoutSeconds to stop this command if the target has not been reached
     * @param driveSubsystem
     * @param visionSubsystem
     */
    public DriveToTargetCommand(VisionTarget target, double speed, double timeoutSeconds,
        DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem) {

        this.target          = target;
        this.speed           = speed;
        this.timeoutSeconds  = timeoutSeconds;
        this.driveSubsystem  = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.armSubsystem    = armSubsystem;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        StringBuilder parms = new StringBuilder();

        parms.append("Target ").append(target)
            .append(", Speed ").append(speed)
            .append(", timeout ").append(timeoutSeconds);

        logCommandStart(parms.toString(), driveSubsystem, visionSubsystem, armSubsystem);

        initializeTime = System.currentTimeMillis();

        if (!SUPPORTED_DRIVE_TARGETS.contains(target)) {
            log("Cannot drive to target " + target + ". Cancelling.");
            // FIXME: does this cancel the entire auto?
            this.cancel();
            return;
        }

        setVisionTarget(target);


        // Start by driving straight towards the target.
        // Assume the robot is lined up with the target.
        targetFound = false;
        driveSubsystem.setMotorSpeeds(speed, speed);
        log("end of init", true);
    }

    @Override
    public void execute() {

        // If the target was switched, then wait before trying to track a target
        if (!visionSubsystem.isCameraInPositionForTarget()) {
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
                log("First target sighting at heading " + lastKnownTargetHeading);
                targetFound = true;
            }
        }

        // If a target has never been found, then keep driving straight.
        // Hopefully a target will be found on the next lap
        if (!targetFound) {
            if (startingToDriveAimlesslyEncoderCount < 0) {
                startingToDriveAimlesslyEncoderCount = driveSubsystem.getEncoderDistanceCm();
                log("Target not found - driving straight ahead hoping to find it.");
            } else {
                double dist = driveSubsystem.getEncoderDistanceCm() - startingToDriveAimlesslyEncoderCount;
                log("Target not found - driving straight ahead hoping to find it. Distance driven: "+dist+"cm.");
            }
            // fixme: this seems very dangerous... should we maybe cancel here? What if the target is a wall!?!?
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
            setFinishReason("Timed out");
            return true;
        }

        double targetArea = visionSubsystem.getTargetAreaPercent();
        switch (target) {
        case CONE_GROUND:
        case CUBE_GROUND:
            boolean detected = armSubsystem.isGamePieceDetected();
            if (detected) {
                log("Drive to target finished - game piece detected.");
            }
            return detected;
        case APRILTAG_GRID:
            final double TARGET_AREA_THRESHOLD = 3.5;
            if (targetArea >= 3.5) {
                log("April tag detected. Actual target area: "+targetArea+", required threshold: "+TARGET_AREA_THRESHOLD);
                return true;
            }
            break;
        case POST_HIGH:
        case POST_LOW:
            log("Not yet supported");
            return true;
        default:
            log("Don't know how to drive to target " + target);
            break;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);

        // Stop the robot
        driveSubsystem.setMotorSpeeds(0, 0);
    }

    private void setVisionTarget(VisionTarget target) {

        if (DriverStation.isTeleopEnabled()) {

            CommandScheduler.getInstance().schedule(new SetVisionTargetCommand(target, visionSubsystem));
        }
    }

}
