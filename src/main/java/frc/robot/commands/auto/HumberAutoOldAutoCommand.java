package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.GameConstants.*;
import frc.robot.commands.auto.oldHumberCommands.*;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.subsystems.*;

public class HumberAutoOldAutoCommand extends SequentialCommandGroup {

    public HumberAutoOldAutoCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startingLaneChooser) {

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        AutoLane      startingLane = startingLaneChooser.getSelected();

        Alliance      alliance     = DriverStation.getAlliance();

        StringBuilder sb           = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Starting Postion          :").append(startingLane);
        sb.append("\nAlliance                     :").append(alliance);

        System.out.println(sb.toString());

        // If any of these are null, then there was some kind of error.
        // FIXME: Is there anything we can do here?
        if (startingLane == null) {

            System.out.println("*** ERROR - null found in auto pattern builder ***");
            return;
        }

        // Print an error if the alliance is not set
        if (alliance == null) {
            System.out.println("*** ERROR **** null Alliance ");
            return;
        }
        else if (alliance == Alliance.Invalid) {
            System.out.println("*** ERROR *** Invalid alliance");
            return;
        }

        // The robot always starts next to the grid
        Zone currentZone = Zone.COMMUNITY;

        /*
         * Set the gyro heading if required
         */
        addCommands(new SetGyroHeadingCommand(180, driveSubsystem));

        /**
         * Step 1 - Score first game piece
         */
        /*
         * Set up for cone scoring
         */
        addCommands(new InstantCommand(() -> {
            armSubsystem.setPincherEncoder(GamePiece.CONE.pincherEncoderCount);
        }));

        ScoringRow scoringRow = ScoringRow.TOP;

        addCommands(new HA_ScoreAutoCommand(scoringRow, armSubsystem));
        addCommands(new WaitCommand(.1));
        addCommands(new HA_ReleaseCommand(armSubsystem));

        /**
         * Step 2 - Exit the zone
         */
        double exitZoneDistance = 330;
        if (startingLane == AutoLane.BOTTOM) {
            exitZoneDistance = 340;
        }
        // Drive out of the zone
        addCommands(new HA_DriveOnHeadingCommand(180, -0.6, exitZoneDistance, driveSubsystem)
            .deadlineWith(new HA_CompactCommand(armSubsystem)));

        /**
         * Step 4 - Balance
         */
        if ((alliance == Alliance.Red && startingLane == AutoLane.BOTTOM)
            || (alliance == Alliance.Blue && startingLane == AutoLane.TOP)) {

            System.out.println("Balance Red/Bot or Blue/Top");
            addCommands(new HA_DriveOnHeadingCommand(135, .1, 1, driveSubsystem));
            addCommands(new HA_DriveOnHeadingCommand(90, .3, 180, driveSubsystem));
            addCommands(new HA_DriveOnHeadingCommand(180, .3, 215, driveSubsystem));
        }
        else if ((alliance == Alliance.Red && startingLane == AutoLane.TOP)
            || (alliance == Alliance.Blue && startingLane == AutoLane.BOTTOM)) {

            System.out.println("Balance Red/Top or Blue/Bottom");
            addCommands(new HA_DriveOnHeadingCommand(225, .1, 1, driveSubsystem));
            addCommands(new HA_DriveOnHeadingCommand(270, .3, 180, driveSubsystem));
            addCommands(new HA_DriveOnHeadingCommand(180, .3, 215, driveSubsystem));
        }

        // Balance on the platform
        addCommands(new WaitCommand(1));
        addCommands(new HA_BalanceCommand(driveSubsystem));
    }
}
