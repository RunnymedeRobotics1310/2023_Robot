package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command will drive on a heading as fast as it safely can.
 * It accelerates smoothly and decelerates smoothly (though at
 * different rates - because this is smoother). If the distance
 * to drive is too short, acceleration, cruise, and deceleration
 * will not occur and the command will drive at a more constant
 * rate.
 * <p>
 * Tested successfully in the field March 26, 2023
 */
public class DriveFastOnHeadingCommand extends RunnymedeCommandBase {

    public enum Direction { forward, backward }

    final double                 factor         = 0.01;

    private final double         heading, distanceCm, timeoutSeconds;

    private final Direction      direction;
    private final boolean        brakeAtEnd;
    private final DriveSubsystem driveSubsystem;

    private long                 initializeTime = 0;


    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in inches.
     * <p>
     * This constructor uses the {@link Constants#DEFAULT_COMMAND_TIMEOUT_SECONDS} for the command
     * timeout
     * <p>
     * This command will drive as fast as it thinks is safe for the distance requested.
     *
     * @param heading 0-360 degrees
     * @param direction forward or backward
     * @param distanceCm for the robot to travel before this command ends.
     * Use a positive number even if traveling backwards
     * @param brakeAtEnd {@code true} if braking, {@code false} to continue driving
     * @param driveSubsystem
     */
    public DriveFastOnHeadingCommand(double heading, Direction direction, double distanceCm, boolean brakeAtEnd,
        DriveSubsystem driveSubsystem) {
        this(heading, direction, distanceCm, brakeAtEnd, Constants.DEFAULT_COMMAND_TIMEOUT_SECONDS, driveSubsystem);
    }

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in cm.
     * <p>
     * This command will drive as fast as it thinks is safe for the distance requested.
     *
     * @param heading 0-360 degrees
     * @param direction forward or backward
     * @param distanceCm for the robot to travel before this command ends.
     * Use a positive number even if traveling backwards
     * @param brakeAtEnd {@code true} if braking, {@code false} to continue driving
     * @param timeoutSeconds to stop this command if the distance has not been reached
     * @param driveSubsystem
     */
    public DriveFastOnHeadingCommand(double heading, Direction direction, double distanceCm, boolean brakeAtEnd,
        double timeoutSeconds, DriveSubsystem driveSubsystem) {

        this.heading        = heading;
        this.direction      = direction;
        this.distanceCm     = distanceCm;
        this.brakeAtEnd     = brakeAtEnd;
        this.timeoutSeconds = timeoutSeconds;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        StringBuilder parms = new StringBuilder();
        parms.append("DriveFastOnHeading: Heading ").append(heading)
            .append(", Direction ").append(direction)
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

        // Determine the error between the current heading and
        // the desired heading

        double error      = driveSubsystem.getHeadingError(heading);

        double speed = direction == Direction.forward ? getSpeed() : -getSpeed();

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

    /** Get the preferred speed needed to drive the specified distance. */
    public double getSpeed() {

        final double MIN_SPEED = 0.3;
        final double MAX_SPEED = 1.0;
        final double SHORT_DISTANCE_SPEED = 0.5;

        final double ACCLERATING_DISTANCE = 40;
        final double DECELERATE_DISTANCE= 70;

        double driven = Math.abs(driveSubsystem.getEncoderDistanceCm());
        double target = Math.abs(distanceCm);

        if (driven > target) {
            return 0;
        }

        if (target < (ACCLERATING_DISTANCE + DECELERATE_DISTANCE)) {
            return SHORT_DISTANCE_SPEED;
        }

        double remaining = target - driven;
        final double speed;

        if (driven < ACCLERATING_DISTANCE) {
            // speeding up
            double pctDriven = (driven/ACCLERATING_DISTANCE);
            speed = Math.max(MIN_SPEED, pctDriven*MAX_SPEED);
            // verbose but handy if troubleshooting is required. comment out when not needed
            log("Accelerating ("+Math.round(pctDriven*100)+"% done). Speed: "+(Math.round(speed*100)/100d)+" dist/total: "+(Math.round(driven))+"/"+target);
        } else if (remaining < DECELERATE_DISTANCE) {
            // slowing down
            double pctToGo = (remaining/DECELERATE_DISTANCE);
            speed = Math.max(MIN_SPEED, pctToGo*MAX_SPEED);
            // verbose but handy if troubleshooting is required. comment out when not needed
            log("Decelerating ("+Math.round(pctToGo*100)+"% done). Speed: "+(Math.round(speed*100)/100d)+" dist/total: "+(Math.round(driven))+"/"+target);
        } else {
            // cruising
            speed = MAX_SPEED;
        }

        return speed;
    }
}
