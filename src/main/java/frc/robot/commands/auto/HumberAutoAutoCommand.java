package frc.robot.commands.auto;

import static frc.robot.commands.drive.DriveFastOnHeadingCommand.Direction.backward;
import static frc.robot.commands.drive.RotateToHeadingCommand.DirectionOfRotation.CLOCKWISE;
import static frc.robot.commands.drive.RotateToHeadingCommand.DirectionOfRotation.COUNTER_CLOCKWISE;

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
import frc.robot.commands.drive.ResetGyroPitchCommand;
import frc.robot.commands.drive.RotateToHeadingCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class HumberAutoAutoCommand extends SequentialCommandGroup {

    public HumberAutoAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startinglanecChooser) {

        final AutoLane startingLane = startinglanecChooser.getSelected();
        final Alliance alliance     = DriverStation.getAlliance();

        StringBuilder  sb           = new StringBuilder("Auto Selections: ");
        sb.append("Pattern: The Humber Auto");
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
        addCommands(new ResetGyroPitchCommand(driveSubsystem));

        addCommands(new InstantCommand(() -> {
            armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
        }));


        addCommands(new ScoreAutoCommand(ScoringRow.TOP, GamePiece.CONE, armSubsystem));
        addCommands(new ReleaseCommand(armSubsystem));


        if (startingLane == AutoLane.BOTTOM) {
            addCommands(new DriveOnHeadingCommand(180, -.65, 340, driveSubsystem)
                .alongWith(new CompactCommand(armSubsystem)));
        }
        else if (startingLane == AutoLane.TOP) {
            addCommands(new DriveFastOnHeadingCommand(180, backward, 360, true, driveSubsystem)
                .alongWith(new CompactCommand(armSubsystem)));
        }

        // Invalid starting lane (unsafe)
        else {
            return;
        }


        if ((alliance == Alliance.Red && startingLane == AutoLane.BOTTOM)
            || (alliance == Alliance.Blue && startingLane == AutoLane.TOP)) {
            addCommands(new RotateToHeadingCommand(90, COUNTER_CLOCKWISE, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(90, .65, 210, driveSubsystem));
            addCommands(new RotateToHeadingCommand(180, CLOCKWISE, driveSubsystem));
        }
        else {
            addCommands(new RotateToHeadingCommand(270, CLOCKWISE, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(270, .65, 210, driveSubsystem));
            addCommands(new RotateToHeadingCommand(180, COUNTER_CLOCKWISE, driveSubsystem));
        }

        addCommands(new DriveOnHeadingCommand(180, .3, 230, driveSubsystem));

        addCommands(new WaitCommand(1));
        addCommands(new BalanceCommand(driveSubsystem));


    }
}
