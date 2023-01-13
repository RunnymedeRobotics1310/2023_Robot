package frc.robot.subsystems.position;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionAwareDriveSubsystem  extends SubsystemBase {

    private final PositioningSubsystem positioningSubsystem;
    private final FieldOfPlay fieldOfPlay;

    PositionAwareDriveSubsystem(/*DriveSubsystem driveSubsystem,*/ PositioningSubsystem positioningSubsystem, FieldOfPlay fieldOfPlay) {
        this.positioningSubsystem = positioningSubsystem;
        this.fieldOfPlay = fieldOfPlay;
    }

    public void driveTo(Position to, Speed speed) {
        Position from = positioningSubsystem.getCurrentPosition();
//        driveSubsystem.drive(from, to, speed);
    }

    public void balanceOnChargeStation() {
        if (!FieldOfPlay.CENTER_OF_MY_CHARGE_STATION.equals(positioningSubsystem.getCurrentPosition())) throw new IllegalStateException("Not on in position");
        // ask positioning system which way we're leaning and move forward or backward to center on the charge station
    }

}
