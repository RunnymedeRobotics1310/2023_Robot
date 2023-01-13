package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.position.*;

public class MidDirectChargeCommand  extends CommandBase {

    private long startTime = 0L;
    private final PositionAwareDriveSubsystem positionAwareDriveSubsystem;
    private final PositioningSubsystem positioningSubsystem;
    private final FieldOfPlay fieldOfPlay;

    public MidDirectChargeCommand(/*PositionAwareDriveSubsystem positionAwareDriveSubsystem, PositioningSubsystem positioningSubsystem, FieldOfPlay fieldOfPlay */) {
        this.positionAwareDriveSubsystem = null; // todo: positionAwareDriveSubsystem;
        this.positioningSubsystem =  null; // todo: positioningSubsystem;
        this.fieldOfPlay =  null; // todo: fieldOfPlay;
        addRequirements(this.positionAwareDriveSubsystem, this.positioningSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        startTime = System.nanoTime();

        Position pos = positioningSubsystem.getCurrentPosition();

        if (FieldOfPlay.CENTER_OF_MY_CHARGE_STATION.equals(pos)) {
            positionAwareDriveSubsystem.balanceOnChargeStation();
        } else {
            if (fieldOfPlay.isInMyChargeStation(pos)) {
                positionAwareDriveSubsystem.driveTo(FieldOfPlay.CENTER_OF_MY_CHARGE_STATION, Speed.SLOW);
            } else {
                positionAwareDriveSubsystem.driveTo(FieldOfPlay.IN_FRONT_OF_CHARGE_STATION, Speed.FAST);
                positionAwareDriveSubsystem.driveTo(FieldOfPlay.CENTER_OF_MY_CHARGE_STATION, Speed.SLOW);
            }
            positionAwareDriveSubsystem.balanceOnChargeStation();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (System.nanoTime() - startTime > 15L * 1000 * 1000 * 1000) return true;
        Position pos = positioningSubsystem.getCurrentPosition();
        return FieldOfPlay.CENTER_OF_MY_CHARGE_STATION.equals(pos) && positioningSubsystem.isLevel();
    }

}
