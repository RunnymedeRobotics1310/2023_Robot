

package frc.robot.commands.drive;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToHeadingCommand extends RunnymedeCommandBase {

    public enum DirectionOfRotation {
        CLOCKWISE, COUNTER_CLOCKWISE, SHORTEST_PATH
    }

    private final double              heading;
    private final DirectionOfRotation requestedDirectionOfRotation;
    private final DriveSubsystem      driveSubsystem;

    private DirectionOfRotation       directionOfRotation = null;

    /**
     * Rotate to the specified compass heading.
     * <p>
     * The robot will spin on the spot to within 10 degrees of the specified compass heading.
     * <p>
     * This constructor uses the {@link DirectionOfRotation#SHORTEST_PATH} for the rotational
     * direction.
     *
     * @param heading 0-360 degrees
     * @param driveSubsystem
     */
    public RotateToHeadingCommand(double heading, DriveSubsystem driveSubsystem) {
        this(heading, DirectionOfRotation.SHORTEST_PATH, driveSubsystem);
    }

    /**
     * Rotate to the specified compass heading.
     * <p>
     * The robot will spin on the spot to within 10 degrees of the specified compass heading.
     * <p>
     * This constructor uses brakeAtEnd {@code true}
     *
     * @param heading 0-360 degrees
     * @param driveSubsystem
     */
    public RotateToHeadingCommand(double heading, DirectionOfRotation directionOfRotation,
        DriveSubsystem driveSubsystem) {

        this.heading                      = heading;
        this.requestedDirectionOfRotation = directionOfRotation;
        this.driveSubsystem               = driveSubsystem;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        if (requestedDirectionOfRotation == DirectionOfRotation.SHORTEST_PATH) {

            double error = driveSubsystem.getHeadingError(heading);

            if (error > 0) {
                directionOfRotation = DirectionOfRotation.CLOCKWISE;
            }
            else {
                directionOfRotation = DirectionOfRotation.COUNTER_CLOCKWISE;
            }
        }
        else {
            directionOfRotation = requestedDirectionOfRotation;
        }


        StringBuilder parms = new StringBuilder();
        parms.append("Target heading ").append(heading)
            .append(", Requested direction of rotation ").append(requestedDirectionOfRotation);

        if (requestedDirectionOfRotation == DirectionOfRotation.SHORTEST_PATH) {
            parms.append(", Calculated direction of rotation ").append(directionOfRotation);
        }

        logCommandStart(parms.toString());
    }

    @Override
    public void execute() {

        double error = driveSubsystem.getHeadingError(heading);

        double speed = DriveConstants.ROTATION_MAX_SPEED;

        // When rotating clockwise, slow down when near the target.
        // A negative error indicates more than 180 degrees left to go.
        if (directionOfRotation == DirectionOfRotation.CLOCKWISE) {
            if (error > 0 && error < DriveConstants.ROTATION_SLOW_ZONE) {
                speed = DriveConstants.ROTATION_SLOW_ZONE_SPEED;
            }
        }
        else {
            if (error < 0 && error > -DriveConstants.ROTATION_SLOW_ZONE) {
                speed = DriveConstants.ROTATION_SLOW_ZONE_SPEED;
            }
        }

        double leftSpeed  = speed;
        double rightSpeed = speed;

        // Spin one of the motors in reverse to rotate on the spot
        if (directionOfRotation == DirectionOfRotation.CLOCKWISE) {
            rightSpeed *= -1;
        }
        else {
            leftSpeed *= -1;
        }

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {

        // Check if at the heading
        double error = driveSubsystem.getHeadingError(heading);

        // When rotating clockwise, stop at a positive error less than the tolerance.
        // A negative error indicates more than 180 degrees left to go.
        if (directionOfRotation == DirectionOfRotation.CLOCKWISE) {
            if (error > 0 && error < DriveConstants.ROTATION_STOPPING_TOLERANCE) {
                setFinishReason("At requested heading");
                return true;
            }
        }
        else {
            if (error < 0 && error > -DriveConstants.ROTATION_STOPPING_TOLERANCE) {
                setFinishReason("At requested heading");
                return true;
            }
        }

        // Check the timeout
        if ((System.currentTimeMillis() - initializeTime) / 1000d > Constants.DEFAULT_COMMAND_TIMEOUT_SECONDS) {
            setFinishReason("Timed out");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);

        // Stop the robot
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}
