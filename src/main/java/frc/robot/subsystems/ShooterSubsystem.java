package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    //Motors on both sides of shooter
    private final CANSparkMax shooterMotor   = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_ADDRESS, MotorType.kBrushless);
    private final CANSparkMax kickerMotor    = new CANSparkMax(ShooterConstants.KICKER_MOTOR_ADDRESS, MotorType.kBrushless);

    private double shooterMotorSetpoint = 0;
    private double kickerMotorSetpoint = 0;

    private final DigitalInput ballSensor = new DigitalInput(ShooterConstants.BALL_SENSOR_ADDRESS);

    public ShooterSubsystem(){
        //Initialize rotation direction
        shooterMotor .setInverted(ShooterConstants.SHOOTER_MOTOR_REVERSED);
        shooterMotor .setIdleMode(IdleMode.kBrake);
        kickerMotor  .setInverted(ShooterConstants.KICKER_MOTOR_REVERSED);
        kickerMotor  .setIdleMode(IdleMode.kBrake);

    }

    public void setShooterMotorSpeed(double speed) {

        shooterMotorSetpoint = speed;
        shooterMotor.set(speed);

    }
    public boolean getBallSensor(){
        return !ballSensor.get();
    }

    public double getShooterEncoderVelocity() {
        return shooterMotor.getEncoder().getVelocity();
    }

    public void setKickerMotorSpeed(double speed) {
        kickerMotorSetpoint = speed;
        kickerMotor.set(speed);

    }

    public double getKickerEncoderVelocity() {
        return kickerMotor.getEncoder().getVelocity();
    }

    public void stop() {
        setShooterMotorSpeed(0);
        setKickerMotorSpeed(0);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter Motor Setpoint", shooterMotorSetpoint);
        SmartDashboard.putBoolean("Ball Sensor", getBallSensor());
        SmartDashboard.putNumber("Shooter Encoder (rpm)", getShooterEncoderVelocity());
        SmartDashboard.putNumber("Kicker Motor Setpoint", kickerMotorSetpoint);
        SmartDashboard.putNumber("Kicker Encoder (rpm)", getKickerEncoderVelocity());

    }

}
