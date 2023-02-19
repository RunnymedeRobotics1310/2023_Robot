package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private static final MotorType motorType      = MotorType.kBrushless;

    // The motors for the arm lifting
    private final CANSparkMax      topArmMotor    = new CANSparkMax(ArmConstants.TOP_MOTOR_PORT, motorType);
    private final CANSparkMax      bottomArmMotor = new CANSparkMax(ArmConstants.BOTTOM_MOTOR_PORT, motorType);

    // motors for extension and pincher
    private final CANSparkMax      extendArmMotor = new CANSparkMax(ArmConstants.EXTEND_MOTOR_PORT, motorType);
    // private final CANSparkMax

    private RelativeEncoder        liftEncoder    = topArmMotor.getEncoder();
    private RelativeEncoder        extendEncoder  = extendArmMotor.getEncoder();

    private double                 liftSpeed      = 0;
    private double                 extendSpeed    = 0;

    /** Creates a new ArmSubsystem */
    public ArmSubsystem() {

        // TODO Do we need to invert any motors?
        // reset encoders?

        bottomArmMotor.follow(topArmMotor);

        resetEncoders();
    }

    /**
     * Gets the lift motor encoder. (top lift motor)
     *
     * @return the lift motor encoder position
     */
    public double getLiftEncoder() {
        return liftEncoder.getPosition();
    }

    /**
     * Gets the extend motor encoder.
     *
     * @return the extend motor encoder position
     */
    public double getExtendEncoder() {
        return liftEncoder.getPosition();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        liftEncoder.setPosition(0);
        extendEncoder.setPosition(0);
    }

    /**
     * Set the speed of the lift motors
     * 
     * @param speed
     */
    public void setLiftSpeed(double speed) {
        this.liftSpeed = speed;

        topArmMotor.set(speed);
    }

    /** Safely stop the arm from moving */
    public void stopLift() {
        setLiftSpeed(0);
    }

    /**
     * Set the speed of the extension motor
     * 
     * @param speed
     */
    public void setExtendSpeed(double speed) {
        this.extendSpeed = speed;

        extendArmMotor.set(speed);
    }

    /** Safely stop the arm from extending */
    public void stopExtend() {
        setExtendSpeed(0);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Lift Motor", liftSpeed);
        SmartDashboard.putNumber("Extend  Motor", extendSpeed);

        SmartDashboard.putNumber("Lift Encoder", getLiftEncoder());
        SmartDashboard.putNumber("Extend Encoder", getExtendEncoder());

    }
}
