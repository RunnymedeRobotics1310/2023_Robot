

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    long                         startTime                  = 0;
    long                         prevLevelTime              = -1;

    double                       startHeading               = 0;
    double                       centeringSpeed             = 0;
    long                         centeringDelayStartTime    = 0;

    // Track the max absolute value of the gyro pitch.
    double                       maxClimbPitch              = 0;

    static final double          ADJUST_SPEED               = .015;
    static final double          CENTERING_SPEED            = .5;
    static final double          CENTER_OF_GRAVITY_MOVEMENT = 40;   // cm
    static final double          CENTERING_DELAY            = 1000; // ms

    static final double          LEVEL_THRESHOLD            = 2;
    static final long            LEVEL_TIMEOUT              = 350;  // ms



    private enum State {
        CLIMB, CENTER, WAIT, BALANCE
    }

    private State currentState;

    /**
     * Drive on a specified compass heading (0-360 degrees) for the specified distance in cm.
     *
     * @param driveSubsystem
     */
    public BalanceCommand(DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("BalanceCommand started at " + driveSubsystem.getPitch());
        startTime    = System.currentTimeMillis();

        startHeading = driveSubsystem.getHeading();

        currentState = State.CLIMB;
    }

    @Override
    public void execute() {

        // Track the max absolute value of the gyro pitch.
        double pitch = driveSubsystem.getPitch();

        switch (currentState) {

        case CLIMB:
            // When climbing, track the max pitch angle;
            maxClimbPitch = Math.max(maxClimbPitch, Math.abs(pitch));

            // When the robot is falling, the pitch will be 3 degrees off of the max
            if (maxClimbPitch - Math.abs(pitch) > 3) {

                currentState = State.CENTER;
                driveSubsystem.resetEncoders();

                // Move opposite to the climb direction
                centeringSpeed = -CENTERING_SPEED * Math.signum(pitch);
            }

            setPitchSpeed(pitch);

            break;

        case CENTER:

            // Drive some number of cm down the ramp in order to move
            // the center of gravity quickly over the center of the ramp
            // once the ramp is moving.
            trackInitialGyroHeading(centeringSpeed);

            // Start trying to balance again after centering.
            if (Math.abs(driveSubsystem.getEncoderDistanceCm()) > CENTER_OF_GRAVITY_MOVEMENT) {
                currentState            = State.WAIT;
                centeringDelayStartTime = System.currentTimeMillis();
            }

            break;

        case WAIT:
            trackInitialGyroHeading(0);

            if (System.currentTimeMillis() - centeringDelayStartTime > CENTERING_DELAY) {
                currentState = State.BALANCE;
            }
            break;


        case BALANCE:
            // FIXME: try to balance after re-centering
            setPitchSpeed(pitch);
            break;
        }

    }

    @Override
    public boolean isFinished() {

        // Timeout
        if (System.currentTimeMillis() - startTime > 10000) {
            return true;
        }

        // Track the gyro pitch.
        double pitch = driveSubsystem.getPitch();

        // FIXME: Only finish when it has been still for a couple of seconds. Instantaneous 0 is too
        // soon, because the charger may be in the middle of passing through 0 while rocking.

        // Not level
        if (Math.abs(pitch) > LEVEL_THRESHOLD) {
            prevLevelTime = -1;
            return false;
        }

        // it's level!

        // first time?
        if (prevLevelTime == -1) {
            // yes - track time
            prevLevelTime = System.currentTimeMillis();
        }
        else {
            // it's already level - nothing to do.
        }

        if (System.currentTimeMillis() - prevLevelTime > LEVEL_TIMEOUT) {
            // It's been level long enough!
            return true;
        }

        // Keep trying
        return false;
    }

    private void setPitchSpeed(double pitch) {

        double speed = 0;

        if (pitch > 1) {
            speed = ADJUST_SPEED;
        }
        else if (pitch < -1) {
            speed = -ADJUST_SPEED;
        }

        // Feed forward balance \/
        speed = speed + pitch / 130;

        trackInitialGyroHeading(speed);
    }

    private void trackInitialGyroHeading(double speed) {

        /*
         * Adjust the speed by the heading correction required to
         * keep the robot straight.
         */
        double currentHeading = driveSubsystem.getHeading();

        // Determine the error between the current heading and
        // the desired heading

        double error          = startHeading - currentHeading;

        if (error > 180) {
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }

        double leftSpeed  = speed + error * .01;
        double rightSpeed = speed - error * .01;

        // In the end, set the speeds on the motors
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {

        if (interrupted) {
            System.out.print("BalanceCommand interrupted");
        }
        else {
            System.out.print("BalanceCommand ended at " + driveSubsystem.getPitch());
        }

        // Stop the robot
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}
