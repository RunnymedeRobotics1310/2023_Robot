

package frc.robot.commands.auto.oldHumberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class HA_BalanceCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    long                         startTime                  = 0;
    long                         prevLevelTime              = -1;

    double                       startHeading               = 0;
    double                       centeringSpeed             = 0;
    long                         centeringDelayStartTime    = 0;

    // Track the max absolute value of the gyro pitch.
    double                       maxClimbPitch              = 0;

    static final double          ADJUST_SPEED               = .01;
    static final double          CENTERING_SPEED            = .25;
    static final double          CENTER_OF_GRAVITY_MOVEMENT = 1;    // cm
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
    public HA_BalanceCommand(DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {

        System.out.println("BalanceCommand started at pitch " + driveSubsystem.getPitch() + ". CLIMB");

        startTime               = System.currentTimeMillis();
        prevLevelTime           = -1;

        startHeading            = driveSubsystem.getHeading();

        centeringSpeed          = 0;
        centeringDelayStartTime = 0;

        maxClimbPitch           = 0;

        /*
         * The current state should either be climb or balance
         * based on whether the current angle is near to the balance point.
         * If close to the balance state, then just go to the balancing
         * routine. This will allow the balance to be re-started when
         * previously balanced and just a bit off.
         */
        double pitch = driveSubsystem.getPitch();

        // Balance if within 4 degrees. FIXME: 4 deg may not be the right number? not tested
        if (Math.abs(pitch) > 4) {
            currentState = State.CLIMB;
        }
        else {
            currentState = State.BALANCE;
        }
    }

    @Override
    public void execute() {

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

                System.out.println("Balance Command: CENTERING from pitch " + pitch);
            }

            setPitchSpeed(pitch);

            break;

        case CENTER:

            // Drive some number of cm down the ramp in order to move
            // the center of gravity quickly over the center of the ramp
            // once the ramp is moving.
            trackInitialGyroHeading(centeringSpeed);

            // Start trying to balance again after centering.
            // Stop after getting to the centering distance.
            if (Math.abs(driveSubsystem.getEncoderDistanceCm()) > CENTER_OF_GRAVITY_MOVEMENT) {

                currentState            = State.WAIT;
                centeringDelayStartTime = System.currentTimeMillis();

                System.out.println("Balance Command: WAITING at pitch " + pitch);
            }

            break;

        case WAIT:

            trackInitialGyroHeading(0);

            if (System.currentTimeMillis() - centeringDelayStartTime > CENTERING_DELAY) {
                currentState = State.BALANCE;
                System.out.println("Balance Command: BALANCING from pitch " + pitch);
            }
            break;


        case BALANCE:
            setPitchSpeed(pitch);

            // if angle is too big, we
            // can't balance with BALANCE
            if (Math.abs(pitch) > 8) {
                currentState = State.CLIMB;
            }
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
        double feedForward = pitch / 180;

        feedForward = Math.min(.15, Math.abs(feedForward)) * Math.signum(feedForward);

        // Feed forward balance
        speed       = speed + feedForward;

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
