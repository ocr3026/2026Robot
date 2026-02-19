package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterIOSpark implements ShooterIO {
    private final SparkFlex shooterMotor;
    private final SparkFlex shooterIntakeMotor;
    private final SparkClosedLoopController shooterPID;
    private final SparkClosedLoopController shooterIntakePID;

    public ShooterIOSpark() {
        shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);
        shooterIntakeMotor = new SparkFlex(ShooterConstants.shooterMotorIntakeID, MotorType.kBrushless);
        shooterPID = shooterMotor.getClosedLoopController();
        shooterIntakePID = shooterIntakeMotor.getClosedLoopController();
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kBrake);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = shooterMotor.getAppliedOutput();
        inputs.shooterAppliedVolts = shooterIntakeMotor.getAppliedOutput();
        inputs.shooterConnected = true;
        inputs.shooterCurrentAmps = shooterMotor.getOutputCurrent();
        inputs.shooterCurrentAmps = shooterIntakeMotor.getOutputCurrent();
        inputs.shooterPosition = Degrees.of(shooterMotor.getEncoder().getPosition());
        inputs.shooterPosition = Degrees.of(shooterIntakeMotor.getEncoder().getPosition());
        inputs.shooterVelocity = DegreesPerSecond.of(shooterMotor.getEncoder().getVelocity());
        inputs.shooterVelocity = DegreesPerSecond.of(shooterIntakeMotor.getEncoder().getVelocity());
    }

    @Override
    public void setAngularSpeed(double speed) {
        shooterPID.setSetpoint(speed, ControlType.kVoltage);
        shooterIntakePID.setSetpoint(speed, ControlType.kVoltage);
    }
}
