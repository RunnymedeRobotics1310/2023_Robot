package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.commands.vision.ConfigureCameraCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PickUpSubstationCommand extends BaseArmCommand {

    private final VisionSubsystem visionSubsystem;

    private GamePiece             gamePiece = GamePiece.CONE;

    public PickUpSubstationCommand(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem) {

        super(armSubsystem);
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {

        System.out.println("PickUp Substation started.  GamePiece " + gamePiece);

        visionSubsystem.setVisionTarget(Constants.VisionConstants.VisionTarget.CONE_SUBSTATION);
        moveCameraToHighPosition();

        printArmState();
        stopArmMotors();
    }

    @Override
    public void execute() {

        double armAngle          = armSubsystem.getArmLiftAngle();
        double armExtendPosition = armSubsystem.getArmExtendEncoder();

        /*
         * Special logic to make sure the arm comes up over the frame when extending.
         */
        if (armAngle < (ArmConstants.CLEAR_FRAME_ARM_ANGLE - ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES)
            && armExtendPosition < ArmConstants.MAX_ARM_EXTEND_INSIDE_FRAME) {

            // Retract the arm before lifting.
            if (!retractArm()) {
                return; // Wait for the retraction before lifting the arm
            }

            moveArmLiftToAngle(ArmConstants.CLEAR_FRAME_ARM_ANGLE + ArmConstants.ARM_LIFT_ANGLE_TOLERANCE_DEGREES + 2);
            return;
        }

        /*
         * Move the arm to set up for an intake from the substation.
         */

        // Always open the pincher, there is no point in waiting
        openPincher();

        if (moveArmLiftToAngle(ArmConstants.SUBSTATION_PICKUP_POSITION.angle)) {
            if (moveArmExtendToEncoderCount(ArmConstants.SUBSTATION_PICKUP_POSITION.extension, ArmConstants.MAX_EXTEND_SPEED)) {
                if (armSubsystem.isGamePieceDetected()) {
                    movePincherToEncoderCount(gamePiece.pincherEncoderCount);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {

        // If at the target position, and a game piece is detected.
        if (armSubsystem.isInPosition(ArmConstants.SUBSTATION_PICKUP_POSITION)) {
            return armSubsystem.getHeldGamePiece() == GamePiece.CONE;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        stopArmMotors();

        if (interrupted) {
            System.out.print("Pick Up Substation interrupted");
        }
        else {
            System.out.print("Pick Up Substation ended");
        }
        printArmState();


        // In Teleop, pick the next command
        if (DriverStation.isTeleopEnabled()) {

            if (armSubsystem.isGamePieceDetected()) {
                CommandScheduler.getInstance().schedule(new PickupGamePieceCommand(gamePiece, armSubsystem));
            }
            else {
                CommandScheduler.getInstance().schedule(new CompactCommand(armSubsystem));
            }

        }
    }

    private void moveCameraToHighPosition() {

        Constants.VisionConstants.VisionTarget visionTarget = Constants.VisionConstants.VisionTarget.NONE;
        if (gamePiece == GamePiece.CONE) {
            visionTarget = Constants.VisionConstants.VisionTarget.CONE_SUBSTATION;
        }
        else if (gamePiece == GamePiece.CUBE) {
            visionTarget = Constants.VisionConstants.VisionTarget.CUBE_SUBSTATION;
        }
        else {
            System.out.println("UNSUPPORTED GAME PIECE: " + gamePiece);
        }

        if (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) {

            CommandScheduler.getInstance().schedule(new ConfigureCameraCommand(visionTarget, visionSubsystem));
        }
    }
}
