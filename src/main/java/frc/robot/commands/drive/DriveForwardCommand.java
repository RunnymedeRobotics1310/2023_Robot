package frc.robot.commands.drive;

import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCommand extends RunnymedeCommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double speed, distance;
    private final boolean brakeAtEnd;
    private double startEncoderCount;
    private double driven = 0;
    public DriveForwardCommand(double speed, double distance, boolean brakeAtEnd, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.speed = speed;
        this.distance = distance;
        this.brakeAtEnd = brakeAtEnd;
    }

    @Override
    public void initialize() {
        super.initialize();
        driveSubsystem.setMotorSpeeds(speed, speed);
        startEncoderCount = driveSubsystem.getEncoderDistanceCm();
        driven = 0;
        logCommandStart("speed: "+speed+" distance: "+distance+" brake: "+brakeAtEnd);
    }

    @Override
    public void execute() {
        driven = driveSubsystem.getEncoderDistanceCm() - startEncoderCount;
    }

    @Override
    public boolean isFinished() {

        return driven >= distance;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
        if (brakeAtEnd) {
            driveSubsystem.stop();
        }
    }
}
