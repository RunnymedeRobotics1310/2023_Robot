package frc.robot.commands.arm;

import static frc.robot.Constants.GameConstants.GamePiece.NONE;

import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.GameConstants.ScoringRow;
import frc.robot.commands.RunnymedeCommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoTuneScore extends RunnymedeCommandBase {

    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private ScoringRow            scoringRow                        = ScoringRow.TOP;
    private GamePiece             heldGamePiece                     = NONE;

    private double                visionTargetHeadingAngleError     = 0;
    private double                visionTargetDistanceError         = 0;

    private static final double   SCORE_CONE_HIGH_MAX_HEADING_ERROR = 0.5;


    public AutoTuneScore(ScoringRow scoringRow, VisionSubsystem visionSubsystem,
        DriveSubsystem driveSubsystem) {

        this.driveSubsystem  = driveSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(driveSubsystem);

        this.scoringRow = scoringRow;
    }

    @Override
    public void initialize() {
    	
        super.initialize();
    }

    @Override
    public void execute() {
        align();
    }

    @Override
    public boolean isFinished() {

        boolean tuned = isAligned();
        setFinishReason("Robot in position to score");
        return tuned;
    }

    @Override
    public void end(boolean isInterrupted) {
        driveSubsystem.stop();
    }

    private void align() {

        visionTargetHeadingAngleError = visionSubsystem.getTargetAngleOffset();
        visionTargetDistanceError     = visionSubsystem.getTargetVerticalAngleOffset() * 70.0 / 3.0;


        double driveSpeed = 0;
        if (visionTargetDistanceError > 0) {
            driveSpeed = .18;
        }

        // fix if out of alignment
        double turn       = visionTargetHeadingAngleError * .015;

        double leftSpeed  = driveSpeed + turn;
        double rightSpeed = driveSpeed - turn;

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    private boolean isAligned() {
        return Math.abs(visionTargetHeadingAngleError) < SCORE_CONE_HIGH_MAX_HEADING_ERROR
            && visionSubsystem.getTargetVerticalAngleOffset() < .02;
    }

}
