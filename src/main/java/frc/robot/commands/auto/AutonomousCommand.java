package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants.*;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.Zone;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.commands.arm.DriveWithPieceCommand;
import frc.robot.commands.arm.PickUpGroundCommand;
import frc.robot.commands.drive.*;
import frc.robot.commands.vision.SetCameraViewCommand;
import frc.robot.commands.vision.SwitchVisionTargetCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class AutonomousCommand extends SequentialCommandGroup {

    private AutoLane              startingLane       = null;
    private GamePiece             currentGamePiece   = null;
    private Orientation           currentOrientation = null;
    private Zone                  currentZone        = null;

    private Alliance              alliance           = null;

    private AutoAction      firstGamePieceScoring  = null;
    private AutoAction      exitZoneAction         = null;
    private AutoAction      secondGamePieceScoring = null;
    private AutoAction      balanceAction          = null;

    private final ArmSubsystem    armSubsystem;
    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;


    public AutonomousCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem,
        SendableChooser<AutoLane> startingLaneChooser,
        SendableChooser<GamePiece> loadedGamePieceChooser,
        SendableChooser<Orientation> startingOrientationChooser,
        SendableChooser<AutoAction> firstGamePieceScoringChooser,
        SendableChooser<AutoAction> exitZoneActionChooser,
        SendableChooser<AutoAction> secondGamePieceScoringChooser,
        SendableChooser<AutoAction> balanceChooser) {

        this.driveSubsystem  = driveSubsystem;
        this.armSubsystem    = armSubsystem;
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

        // Start by switching the camera view if picking up a game piece.
        // this will set the camera angle while driving
        if (exitZoneAction == AutoAction.PICK_UP_CUBE) {
            addCommands(new SetCameraViewCommand(CameraView.LOW, visionSubsystem));
            addCommands(new SwitchVisionTargetCommand(VisionTargetType.CUBE, visionSubsystem));
        }
        if (exitZoneAction == AutoAction.PICK_UP_CONE) {
            addCommands(new SetCameraViewCommand(CameraView.LOW, visionSubsystem));
            addCommands(new SwitchVisionTargetCommand(VisionTargetType.CONE, visionSubsystem));
        }

        // Drive out of the zone
        // This command may cause a rotation to heading 0.
        addCommands(new DriveOnHeadingCommand(0, 0.6, 400, 2, driveSubsystem));

        currentZone        = Zone.FIELD;
        currentOrientation = Orientation.FACE_FIELD;

        /*
         * If a piece is not required, this portion is complete
         */
        if (exitZoneAction == AutoAction.PICK_UP_CUBE) {
            // FIXME: ensure cube is close enough to the robot that the arm can reach it
            addCommands(new PickUpGroundCommand(GamePiece.CUBE, armSubsystem));
            addCommands(new DriveToTargetCommand(VisionTargetType.CUBE, .2, driveSubsystem, visionSubsystem));
            addCommands(new DriveWithPieceCommand(armSubsystem));
        }
        if (exitZoneAction == AutoAction.PICK_UP_CONE) {
            addCommands(new PickUpGroundCommand(GamePiece.CONE, armSubsystem));
            addCommands(new DriveToTargetCommand(VisionTargetType.CONE, .2, driveSubsystem, visionSubsystem));
            addCommands(new DriveWithPieceCommand(armSubsystem));
        }
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
        addCommands(new DriveOnHeadingCommand(270.0, 0.5, 5, 3, driveSubsystem));
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

        // Drive to the platform
        if ((alliance == Alliance.Red && startingLane == AutoLane.BOTTOM)
            || (alliance == Alliance.Blue && startingLane == AutoLane.TOP)) {

            System.out.println("Balance Red/Bot or Blue/Top");
            addCommands(new DriveOnHeadingCommand(180, -.3, 50, 0.25, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(270, -.3, 450, 1.25, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(180, -.5, 400, 1.25, driveSubsystem));
        }
        else if ((alliance == Alliance.Red && startingLane == AutoLane.TOP)
            || (alliance == Alliance.Blue && startingLane == AutoLane.BOTTOM)) {

            System.out.println("Balance Red/Top or Blue/Bottom");
            addCommands(new DriveOnHeadingCommand(180, -.3, 50, 0.25, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(90, -.3, 450, 1.25, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(180, -.3, 400, 1.25, driveSubsystem));
        }
        else {
            System.out.println("Balance 'else'");
            addCommands(new DriveOnHeadingCommand(180, -.3, 400, 1.75, driveSubsystem));
        }

        // Balance on the platform
        addCommands(new BalanceCommand(driveSubsystem));
    }
}
