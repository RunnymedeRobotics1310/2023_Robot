package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.AutoAction;
import frc.robot.Constants.AutoConstants.AutoLane;
import frc.robot.Constants.AutoConstants.Orientation;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.Zone;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.SetGyroHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class AutonomousCommand extends SequentialCommandGroup {

    private AutoLane        startingLane           = null;
    private GamePiece       currentGamePiece       = null;
    private Orientation     currentOrientation     = null;
    private Zone            currentZone            = null;

    private Alliance        alliance               = null;

    private AutoAction      firstGamePieceScoring  = null;
    private AutoAction      exitZoneAction         = null;
    private AutoAction      secondGamePieceScoring = null;
    private AutoAction      balanceAction          = null;
    private DriveSubsystem  driveSubsystem         = null;
    private VisionSubsystem visionSubsystem        = null;


    public AutonomousCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startingLaneChooser,
        SendableChooser<GamePiece> loadedGamePieceChooser,
        SendableChooser<Orientation> startingOrientationChooser,
        SendableChooser<AutoAction> firstGamePieceScoringChooser,
        SendableChooser<AutoAction> exitZoneActionChooser,
        SendableChooser<AutoAction> secondGamePieceScoringChooser,
        SendableChooser<AutoAction> balanceChooser) {

        this.driveSubsystem  = driveSubsystem;
        this.visionSubsystem = visionSubsystem;

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        startingLane           = startingLaneChooser.getSelected();
        currentGamePiece       = loadedGamePieceChooser.getSelected();
        currentOrientation     = startingOrientationChooser.getSelected();

        firstGamePieceScoring  = firstGamePieceScoringChooser.getSelected();
        exitZoneAction         = exitZoneActionChooser.getSelected();
        secondGamePieceScoring = secondGamePieceScoringChooser.getSelected();
        balanceAction          = balanceChooser.getSelected();

        alliance               = DriverStation.getAlliance();

        StringBuilder sb = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Starting Postion          :").append(startingLane);
        sb.append("\n   Starting Orientation      :").append(currentOrientation);
        sb.append("\n   Loaded Game Piece         :").append(currentGamePiece);
        sb.append("\n   First Game Piece Scoring  :").append(firstGamePieceScoring);
        sb.append("\n   Exit Zone Action          :").append(exitZoneAction);
        sb.append("\n   Second Game Piece Scoring :").append(secondGamePieceScoring);
        sb.append("\n   Balance Action            :").append(balanceAction);
        sb.append("\nAlliance                     :").append(alliance);

        System.out.println(sb.toString());

        // If any of these are null, then there was some kind of error.
        // FIXME: Is there anything we can do here?
        if (startingLane == null
            || currentOrientation == null
            || currentGamePiece == null
            || firstGamePieceScoring == null
            || exitZoneAction == null
            || secondGamePieceScoring == null
            || balanceAction == null) {

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
        currentZone = Zone.COMMUNITY;

        /*
         * Set the gyro heading if required
         */
        if (currentOrientation == Orientation.FACE_GRID) {
            addCommands(new SetGyroHeadingCommand(180, driveSubsystem));
        }
        else {
            addCommands(new SetGyroHeadingCommand(0, driveSubsystem));
        }

        /*
         * Compose the required auto commands for each of the steps in the auto
         * based on the selector values
         */
        addStep1Commands_ScoreFirstGamePiece();
        addStep2Commands_ExitZone();
        addStep3Commands_ScoreSecondGamePiece();
        addStep4Commands_Balance();
    }

    /**
     * Step 1 - Score first game piece
     */
    private void addStep1Commands_ScoreFirstGamePiece() {

        // The selector must be set to score low or score mid, otherwise
        // there is nothing to do

        if (!(firstGamePieceScoring == AutoAction.SCORE_BOTTOM
            || firstGamePieceScoring == AutoAction.SCORE_MIDDLE
            || firstGamePieceScoring == AutoAction.SCORE_TOP)) {
            return;
        }



        if (currentOrientation == Orientation.FACE_GRID) {

            switch (currentGamePiece) {
            case CUBE:
                break;
            case CONE:
                break;
            }
            // FIXME:
            // Set the arm position (different positions required for cone or cube
            // Drive forward?
            // Drop the piece

        }
        else {
            // Currently facing the field

            // When facing the field, only a low delivery is allowed because a piece would be
            // balanced on the bumper
            if (firstGamePieceScoring == AutoAction.SCORE_MIDDLE
                || firstGamePieceScoring == AutoAction.SCORE_TOP) {
                System.out.println("Cannot score mid unless facing grid, overriding to score low");
            }

            // Drive Backward
            // NOTE: deposit the piece placed on the bumper in the low scoring position using
            // gravity and inertia (or maybe a piston?)

            // reverse and deposit the starting piece

            double speed = -0.3;
            addCommands(new DriveOnHeadingCommand(0, speed, 50, 0.25, driveSubsystem));
        }

        // Now that the game piece is scored, we do not have a game piece
        currentGamePiece = GamePiece.NONE;
    }

    /**
     * Step 2 - Exit the zone, and optionally pick up a second game piece
     */
    private void addStep2Commands_ExitZone() {

        // An exit zone action must be selected, otherwise do nothing

        if (!(exitZoneAction == AutoAction.EXIT_ZONE
            || exitZoneAction == AutoAction.PICK_UP_CUBE
            || exitZoneAction == AutoAction.PICK_UP_CONE)) {
            return;
        }

        // Drive out of the zone
        // This command may cause a rotation to heading 0.
        addCommands(new DriveOnHeadingCommand(0, 0.5, 400, 2, driveSubsystem));

        currentZone        = Zone.FIELD;
        currentOrientation = Orientation.FACE_FIELD;

        /*
         * If a piece is not required, this portion is complete
         */
        if (exitZoneAction == AutoAction.PICK_UP_CUBE) {
            // FIXME: ensure cube is close enough to the robot that the arm can reach it
            addCommands(new DriveToTargetCommand(VisionTargetType.CUBE, .2, driveSubsystem, visionSubsystem));
        }
        if (exitZoneAction == AutoAction.PICK_UP_CONE) {
            // FIXME: add commands to drive right up to the cone
        }

        /*
         * Grab a piece
         */

        // FIXME:
        // Game piece should be directly in front of robot now but orientation of robot w.r.t. field
        // may not be 0/180
        // Add new command to tell arm to pick up the piece it sees in front of itself (arm will
        // need to be able to see)
        // look for piece
        // align arm with piece
        // pick up piece
        // reposition arm to transport position
    }

    /**
     * Step 3 - Deliver a second game piece if required
     */
    private void addStep3Commands_ScoreSecondGamePiece() {

        // If a scoring location is not set, there is nothing to do.

        if (!(secondGamePieceScoring == AutoAction.SCORE_BOTTOM
            || secondGamePieceScoring == AutoAction.SCORE_MIDDLE
            || secondGamePieceScoring == AutoAction.SCORE_TOP)) {
            return;
        }

        // Check that grabbing a piece was scheduled

        if (!(exitZoneAction == AutoAction.PICK_UP_CONE
            || exitZoneAction == AutoAction.PICK_UP_CUBE)) {
            System.out.println("*** ERROR *** Cannot deliver a second piece if it was not picked up");
            // Do nothing
            return;
        }

        // Turn around and go back to the grid
        addCommands(new DriveOnHeadingCommand(180.0, 0.5, 400, 3, driveSubsystem));
        addCommands(new DriveToTargetCommand(VisionTargetType.TAG, 0.3, driveSubsystem, visionSubsystem));

        // Vision subsystem to acquire the nearest scoring position marker (vision subsystem
        // operation to find scoring position +
        // command to switch to "locate AprilTag" or "locate ConePostRetroReflector", etc)
        // Drive towards target
        // todo: fixme: the bot cannot see the apriltag, so it doesn't know where to stop!
        // addCommands(new DriveToTargetCommand(VisionSubsystem.VisionTargetType.TAG, 0.5,
        // driveSubsystem, visionSubsystem));
        // Position the arm to the appropriate height
        // Drop object

        currentOrientation = Orientation.FACE_GRID;
        currentZone        = Zone.COMMUNITY;
    }

    /**
     * Step 4 - Balance if required
     */
    private void addStep4Commands_Balance() {

        // If the balance command was not selected, there is nothing to do

        if (balanceAction != AutoAction.BALANCE) {
            return;
        }

        // FIXME:

        // Determine the path to the platform

        // Determine the path to the charging station based on the starting position, current
        // orientation and current zone.

        // NOTE: This is some complex logic, but essentially the charging station is
        // in one of 6 directions from the robot.
        // If N is toward the opposing alliance, the the charging station is NW, N, NE of
        // the robot when the robot is at the Grid, SW, S, or SE when the robot is in the field.

        // An Example:
        // If the robot is in the Grid area, and the robot is on the Blue Alliance, and
        // the robot has started in the low position, then the charging station is
        // North-West of the robot.

        // In the case of the Red alliance, the charging station will be to the North-East
        // in the same scenario.

        // NOTE: A small additional complexity is that the robot could be currently facing
        // either toward the field or toward the grid.

        // Drive to the platform
        addCommands(new DriveOnHeadingCommand(180, -.3, 50, 0.25, driveSubsystem));
        addCommands(new DriveOnHeadingCommand(270, -.3, 450, 1.25, driveSubsystem));
        addCommands(new DriveOnHeadingCommand(180, -.5, 400, 1.25, driveSubsystem));

        // Balance on the platform

        // TODO: Integrate the code from the auto-balance branch.
        addCommands(new BalanceCommand(driveSubsystem));
    }
}
