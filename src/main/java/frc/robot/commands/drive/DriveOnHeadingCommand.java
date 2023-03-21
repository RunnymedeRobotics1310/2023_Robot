

package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveOnHeadingCommand extends RunnymedeCommandBase {

    final double                 factor         = 0.01;

    private final double         heading, speed, distanceCm, timeoutSeconds;
    private final boolean        brakeAtEnd;
    private final DriveSubsystem driveSubsystem;

    private long                 initializeTime = 0;

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in inches.
     * <p>
     * This constructor uses the {@link Constants#DEFAULT_COMMAND_TIMEOUT_SECONDS} for the command
     * timeout and brakeAtEnd {@code true}
     *
     * @param heading 0-360 degrees
     * @param speed in the range 0-1.0 for forward travel, 0 - -1.0 for reverse travel
     * @param distanceCm for the robot to travel before this command ends.
     * Use a positive number even if traveling backwards
     * @param driveSubsystem
     */
    public DriveOnHeadingCommand(double heading, double speed, double distanceCm,
        DriveSubsystem driveSubsystem) {
        this(heading, speed, distanceCm, true, Constants.DEFAULT_COMMAND_TIMEOUT_SECONDS, driveSubsystem);
    }

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in inches.
     * <p>
     * This constructor uses the {@link Constants#DEFAULT_COMMAND_TIMEOUT_SECONDS} for the command
     * timeout
     *
     * @param heading 0-360 degrees
     * @param speed in the range 0-1.0 for forward travel, 0 - -1.0 for reverse travel
     * @param distanceCm for the robot to travel before this command ends.
     * Use a positive number even if traveling backwards
     * @param driveSubsystem
     */
    public DriveOnHeadingCommand(double heading, double speed, double distanceCm, boolean brakeAtEnd,
        DriveSubsystem driveSubsystem) {
        this(heading, speed, distanceCm, brakeAtEnd, Constants.DEFAULT_COMMAND_TIMEOUT_SECONDS, driveSubsystem);
    }

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in inches.
     * <p>
     * This constructor uses the brakeAtEnd {@code true}
     *
     * @param heading 0-360 degrees
     * @param speed in the range 0-1.0 for forward travel, 0 - -1.0 for reverse travel
     * @param distanceCm for the robot to travel before this command ends.
     * Use a positive number even if traveling backwards
     * @param timeoutSeconds to stop this command if the distance has not been reached
     * @param driveSubsystem
     */
    public DriveOnHeadingCommand(double heading, double speed, double distanceCm, double timeoutSeconds,
        DriveSubsystem driveSubsystem) {
        this(heading, speed, distanceCm, true, timeoutSeconds, driveSubsystem);
    }

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in cm.
     *
     * @param heading 0-360 degrees
     * @param speed in the range 0-1.0 for forward travel, 0 - -1.0 for reverse travel
     * @param distanceCm for the robot to travel before this command ends.
     * Use a positive number even if traveling backwards
     * @param brakeAtEnd {@code true} if braking, {@code false} to continue driving
     * @param timeoutSeconds to stop this command if the distance has not been reached
     * @param driveSubsystem
     */
    public DriveOnHeadingCommand(double heading, double speed, double distanceCm, boolean brakeAtEnd,
        double timeoutSeconds, DriveSubsystem driveSubsystem) {

        this.heading        = heading;
        this.speed          = speed;
        this.distanceCm     = distanceCm;
        this.brakeAtEnd     = brakeAtEnd;
        this.timeoutSeconds = timeoutSeconds;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        StringBuilder parms = new StringBuilder();
        parms.append("Heading ").append(heading)
            .append(", Speed ").append(speed)
            .append(", Distance ").append(distanceCm)
            .append(", Brake at end ").append(brakeAtEnd)
            .append(", Timeout ").append(timeoutSeconds);

        logCommandStart(parms.toString());

        // Reset the distance to zero.
        // Note: this must be done in the initialize instead of in the constructor
        // because the command could get constructed long before it is run
        driveSubsystem.resetEncoders();

        initializeTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {

        // Track the gyro heading.

        double currentHeading = driveSubsystem.getHeading();

        // Determine the error between the current heading and
        // the desired heading

        double error          = heading - currentHeading;

        if (error > 180) {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        double leftSpeed  = speed + error * factor;
        double rightSpeed = speed - error * factor;

        // In the end, set the speeds on the motors
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {

        // This command can either reach the distance or time out

        // Check the distance
        // use the absolute value to account for driving backwards
        if (Math.abs(driveSubsystem.getEncoderDistanceCm()) > Math.abs(distanceCm)) {
            setFinishReason("At required distance");
            return true;
        }

        // Check the timeout
        if ((System.currentTimeMillis() - initializeTime) / 1000d > timeoutSeconds) {
            setFinishReason("Timed out");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);

        // Stop the robot if required
        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }
    }
}
