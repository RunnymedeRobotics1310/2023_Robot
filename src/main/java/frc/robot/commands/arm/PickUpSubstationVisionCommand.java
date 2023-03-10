package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GameConstants.GamePiece;
import frc.robot.Constants.VisionConstants.CameraView;
import frc.robot.commands.vision.SetCameraViewCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.VisionSubsystem.VisionTargetType;

public class PickUpSubstationVisionCommand extends BaseArmCommand {

    private static final double   MIN_PICKUP_DISTANCE  = 95;
    private static final double   MAX_PICKUP_DISTANCE  = 115;
    private static final double   TARGET_CAMERA_OFFSET = 5;

    private final VisionSubsystem visionSubsystem;
    private final DriveSubsystem  driveSubsystem;

    private GamePiece             gamePiece            = GamePiece.CONE;

    private double                visionTargetOffset   = 0;

    private enum State {
        MOVE_CAMERA, ALIGN, PICKUP
    };

    private State currentState = State.MOVE_CAMERA;

    public PickUpSubstationVisionCommand(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem) {

        super(armSubsystem);

        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        System.out.println("PickUp Substation started.  GamePiece " + gamePiece);

        visionSubsystem.setVisionTargetType(VisionTargetType.CONE);
        moveCameraToHighPosition();

        printArmState();
        stopArmMotors();
    }

    @Override
    public void execute() {

        double armAngle             = armSubsystem.getArmLiftAngle();
        double armExtendPosition    = armSubsystem.getArmExtendEncoder();
        double ultrasonicDistanceCm = driveSubsystem.getUltrasonicDistanceCm();

        switch (currentState) {

        case MOVE_CAMERA:

            if (visionSubsystem.getCameraView() == CameraView.HIGH
                && visionSubsystem.isVisionTargetFound()) {
                currentState       = State.ALIGN;
                visionTargetOffset = visionSubsystem.getTargetAngleOffset();
            }
            // Wait for the camera to get into position;
            return;

        case ALIGN:

            // Move forward or backwards until the distance is in range
            // Scoring can only happen between 75 and 115

            double driveSpeed = 0;

            if (driveSubsystem.getUltrasonicDistanceCm() < MIN_PICKUP_DISTANCE) {
                driveSpeed = -.1;
            }

            if (driveSubsystem.getUltrasonicDistanceCm() > MAX_PICKUP_DISTANCE) {
                driveSpeed = .1;
            }

            if (visionSubsystem.isVisionTargetFound()) {
                visionTargetOffset = visionSubsystem.getTargetAngleOffset() - TARGET_CAMERA_OFFSET;
            }

            double turn = visionTargetOffset * .01;

            driveSubsystem.setMotorSpeeds(driveSpeed + turn, driveSpeed - turn);

            if (driveSpeed == 0
                && Math.abs(visionTargetOffset) < 2) {
                currentState = State.PICKUP;
            }
            // Wait for target alignment
            return;

        case PICKUP:

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

                // The extension is based on the distance following the formula

                double requiredExtensionEncoderPosition = ultrasonicDistanceCm * .88 - 53.1;

                if (moveArmExtendToEncoderCount(requiredExtensionEncoderPosition, ArmConstants.MAX_EXTEND_SPEED)) {

                    if (armSubsystem.isGamePieceDetected()) {
                        movePincherToEncoderCount(gamePiece.pincherEncoderCount);
                    }
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
                CommandScheduler.getInstance().schedule(new DriveWithPieceCommand(armSubsystem));
            }
            else {
                CommandScheduler.getInstance().schedule(new CompactCommand2(armSubsystem));
            }

        }
    }

    private void moveCameraToHighPosition() {

        if (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) {

            CommandScheduler.getInstance().schedule(new SetCameraViewCommand(CameraView.HIGH, visionSubsystem));
        }
    }
}
