package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.commands.arm.CompactCommand;
import frc.robot.commands.arm.ReleaseCommand;
import frc.robot.commands.arm.ScoreAutoCommand;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class HumberAutoAutoCommand extends SequentialCommandGroup {

    private AutoLane startingLane = null;
    private Alliance alliance     = null;

    public HumberAutoAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startinglanecChooser) {

        startingLane = startinglanecChooser.getSelected();
        alliance     = DriverStation.getAlliance();

        StringBuilder sb = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Starting Position :").append(startingLane);
        sb.append("\nAlliance             :").append(alliance);

        System.out.println(sb.toString());

        // If any of these are null, then there was some kind of error.
        if (startingLane == null) {
            System.out.println("*** ERROR *** null starting lane");
        }

        // Print an error if the alliance is not set
        if (alliance == null) {
            System.out.println("*** ERROR **** null Alliance ");
        }
        else if (alliance == Alliance.Invalid) {
            System.out.println("*** ERROR *** Invalid alliance");
        }

        addCommands(new SetGyroHeadingCommand(180, driveSubsystem));

        addCommands(new InstantCommand(() -> {
            armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
        }));


        addCommands(new ScoreAutoCommand(ScoringRow.TOP, armSubsystem));

        addCommands(new WaitCommand(.1));
        addCommands(new ReleaseCommand(armSubsystem));


        double exitZoneDistance = 330;
        if (startingLane == AutoLane.BOTTOM) {
            exitZoneDistance = 340;
        }

        addCommands(new DriveOnHeadingCommand(180, .65, exitZoneDistance, driveSubsystem)
            .deadlineWith(new CompactCommand(armSubsystem)));

        if ((alliance == Alliance.Red && startingLane == startingLane.BOTTOM)
            || (alliance == Alliance.Blue && startingLane == startingLane.TOP)) {
            addCommands(new DriveOnHeadingCommand(90, .65, 180, driveSubsystem));
        }
        else {
            addCommands(new DriveOnHeadingCommand(270, .65, 180, driveSubsystem));
        }


        addCommands(new DriveOnHeadingCommand(0, -.5, 120, false, driveSubsystem)
            .andThen(new DriveOnHeadingCommand(0, -.3, 50, driveSubsystem)));

        addCommands(new WaitCommand(1));
        addCommands(new BalanceCommand(driveSubsystem));


    }
}
