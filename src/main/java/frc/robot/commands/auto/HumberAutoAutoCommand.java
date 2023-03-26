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
import frc.robot.commands.drive.DriveFastOnHeadingCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.commands.drive.DriveFastOnHeadingCommand.Direction.*;


public class HumberAutoAutoCommand extends SequentialCommandGroup {

    public HumberAutoAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startinglanecChooser) {

        final AutoLane startingLane = startinglanecChooser.getSelected();
        final Alliance alliance     = DriverStation.getAlliance();

        StringBuilder sb = new StringBuilder("Auto Selections: ");
        sb.append("Starting Position:").append(startingLane).append(' ');
        sb.append("Alliance:").append(alliance);

        System.out.println(sb.toString());

        // If any of these are null, then there was some kind of error.
        if (startingLane == null || startingLane == AutoLane.MIDDLE) {
            System.out.println("*** ERROR *** invalid starting lane");
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
        addCommands(new ReleaseCommand(armSubsystem));


        if (startingLane == AutoLane.BOTTOM) {
            addCommands(new DriveOnHeadingCommand(180, -.65, 340, driveSubsystem)
                .deadlineWith(new CompactCommand(armSubsystem)));
        }
        else if (startingLane == AutoLane.TOP) {
            addCommands(new DriveFastOnHeadingCommand(180, backward, 330, true, driveSubsystem)
                .deadlineWith(new CompactCommand(armSubsystem)));
        }

        // Invalid starting lane (unsafe)
        else {
            return;
        }


        if ((alliance == Alliance.Red && startingLane == AutoLane.BOTTOM)
            || (alliance == Alliance.Blue && startingLane == AutoLane.TOP)) {
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
