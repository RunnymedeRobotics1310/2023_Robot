package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

public class MiddleMayhem1AutoCommand extends SequentialCommandGroup {

    public MiddleMayhem1AutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        StringBuilder sb = new StringBuilder("Auto Selections: ");
        sb.append("Pattern: Middle Mayhem ");
        sb.append("Alliance: ").append(DriverStation.getAlliance());
        System.out.println(sb.toString());

        /*
         * Set the to be facing the grid
         */
        addCommands(new SetGyroHeadingCommand(180, driveSubsystem));

        /*
         * Robot is holding a cone
         */
        addCommands(new InstantCommand(() -> {
            armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
        }));

        /*
         * Score the cone
         */
        addCommands(new ScoreAutoCommand(ScoringRow.TOP, GamePiece.CONE, armSubsystem));
        addCommands(new ReleaseCommand(armSubsystem));

        // Drive over charger & compact
        addCommands(new DriveOnHeadingCommand(180, -.4, 230, false, driveSubsystem)
            .andThen(new DriveOnHeadingCommand(180, -.3, 70, false, driveSubsystem)
                .andThen(new DriveOnHeadingCommand(180, -.5, 80, driveSubsystem)))
            .alongWith(new CompactCommand(armSubsystem)));

        // Mount charger again
        addCommands(new WaitCommand(.5));
        addCommands(new DriveOnHeadingCommand(180, .3, 190, driveSubsystem));

        /*
         * Balance on the platform
         */
        addCommands(new WaitCommand(1));
        addCommands(new BalanceCommand(driveSubsystem));
    }
}

