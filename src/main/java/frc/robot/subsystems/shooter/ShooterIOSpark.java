package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ShooterIOSpark implements ShooterIO {
    private final SparkFlex shooterMotor;
    private final SparkFlex shooterMotor2;
    private final SparkFlex shooterKickupMotor;
    private final SparkClosedLoopController shooterPID;
    private final SparkClosedLoopController shooter2PID;
    private final SparkClosedLoopController shooterKickupPID;

    public ShooterIOSpark() {
        shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, MotorType.kBrushless);
        shooterMotor2 = new SparkFlex(ShooterConstants.shooterMotorID2, MotorType.kBrushless);
        shooterKickupMotor = new SparkFlex(ShooterConstants.shooterMotorKickupID, MotorType.kBrushless);
        shooterPID = shooterMotor.getClosedLoopController();
        shooter2PID = shooterMotor2.getClosedLoopController();
        shooterKickupPID = shooterKickupMotor.getClosedLoopController();

        FeedForwardConfig shooterConf = new FeedForwardConfig();
        shooterConf.kV(0.00245);

        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kBrake);
        shooterConfig.closedLoop.p(0.0006).i(0).d(0.00192);
        shooterConfig.closedLoop.apply(shooterConf);

        FeedForwardConfig shooter2Conf = new FeedForwardConfig();
        shooter2Conf.kV(0.00245);

        SparkFlexConfig shooter2Config = new SparkFlexConfig();
        shooter2Config.idleMode(IdleMode.kBrake);
        shooter2Config.closedLoop.p(0.0006).i(0).d(0.00192);
        shooter2Config.closedLoop.apply(shooter2Conf);

        FeedForwardConfig shooterKickupConf = new FeedForwardConfig();
        shooterKickupConf.kV(0.00245);

        SparkFlexConfig shooterKickupConfig = new SparkFlexConfig();
        shooterKickupConfig.idleMode(IdleMode.kBrake);
        shooterKickupConfig.closedLoop.p(0.0006).i(0).d(0.00192);
        shooterKickupConfig.closedLoop.apply(shooterKickupConf);

        shooterConfig.smartCurrentLimit(70);
        shooter2Config.smartCurrentLimit(70);
        shooterKickupConfig.smartCurrentLimit(70);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterKickupMotor.configure(shooterKickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = shooterMotor.getAppliedOutput();
        inputs.shooterConnected = true;

        inputs.shooter2AppliedVolts = shooterMotor2.getAppliedOutput();
        inputs.shooter2Connected = true;
        
        inputs.shooterKickupAppliedVolts = shooterKickupMotor.getAppliedOutput();
        inputs.shooterKickupConnected = true;

        inputs.shooterCurrentAmps = shooterMotor.getOutputCurrent();
        inputs.shooterPosition = Degrees.of(shooterMotor.getEncoder().getPosition());
        inputs.shooterVelocity = DegreesPerSecond.of(shooterMotor.getEncoder().getVelocity());

        inputs.shooter2CurrentAmps = shooterMotor2.getOutputCurrent();
        inputs.shooter2Position = Degrees.of(shooterMotor2.getEncoder().getPosition());
        inputs.shooter2Velocity = DegreesPerSecond.of(shooterMotor2.getEncoder().getVelocity());

        inputs.shooterKickupPosition = Degrees.of(shooterKickupMotor.getEncoder().getPosition());
        inputs.shooterKickupCurrentAmps = shooterKickupMotor.getOutputCurrent();
        inputs.shooterKickupVelocity = DegreesPerSecond.of(shooterKickupMotor.getEncoder().getVelocity());
    }

    @Override
    public void setShooterSpeed(double speed) {
        shooterPID.setSetpoint(speed, ControlType.kVelocity);
    }
    
    @Override
    public void setShooter2Speed(double speed) {
        shooter2PID.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void setKickupSpeed(double speed) {
        shooterKickupPID.setSetpoint(speed, ControlType.kVelocity);
    }
}
