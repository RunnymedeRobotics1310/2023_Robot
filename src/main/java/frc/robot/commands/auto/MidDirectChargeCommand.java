package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MidDirectChargeCommand  extends CommandBase {

    public MidDirectChargeCommand(/*
        driveSubsystem,
        chargeStation,
        positioningSubsystem,
        fieldOfPLay */) {

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // ask positioning subsystem where we are
        // record initial position
    }

    @Override
    public void execute() {
        // record start time
        // drive at top speed to the on-ramp
        // drive at half speed onto the middle of chargeStation and stop
        // ask positioning subystem if we are balanced & drive forward/backward to balance
        // set balanced=true
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if balanced return true
        // if now is 15s after start time  (i.e. auto is over) return true
        return false;
    }

}
