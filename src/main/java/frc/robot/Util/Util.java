// package frc.robot.Util;

// import static edu.wpi.first.units.Units.*;

// import java.util.function.Supplier;

// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.motorsims.SimulatedBattery;
// import org.ironmaple.simulation.motorsims.SimulatedMotorController;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
// import com.ctre.phoenix6.sim.CANcoderSimState;
// import com.ctre.phoenix6.sim.TalonFXSimState;
// import com.ctre.phoenix6.swerve.SwerveModuleConstants;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.generated.TunerConstants;

// public final class Util {
//     public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
//         for (int i = 0; i < maxAttempts; i++) {
//             var error = command.get();
//             if (error.isOK()) break;
//         }
//     }

//     public static class TalonFXMotorControllerSim implements SimulatedMotorController {
//         private static int instances = 0;
//         public final int id;

//         private final TalonFXSimState talonFXSimState;

//         public TalonFXMotorControllerSim(TalonFX talonFX) {
//             this.id = instances++;

//             this.talonFXSimState = talonFX.getSimState();
//         }

//         @Override
//         public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle, AngularVelocity encoderVelocity) {
//             talonFXSimState.setRawRotorPosition(encoderAngle);
//             talonFXSimState.setRotorVelocity(encoderVelocity);
//             talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
//             return talonFXSimState.getMotorVoltageMeasure();
//         }
//     }

//     public static class TalonFXMotorControllerWithRemoteCancoderSim extends TalonFXMotorControllerSim {
//         private final CANcoderSimState remoteCancoderSimState;

//         public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
//             super(talonFX);
//             this.remoteCancoderSimState = cancoder.getSimState();
//         }

//         @Override
//         public Voltage updateControlSignal(
//                 Angle mechanismAngle,
//                 AngularVelocity mechanismVelocity,
//                 Angle encoderAngle,
//                 AngularVelocity encoderVelocity) {
//             remoteCancoderSimState.setRawPosition(mechanismAngle);
//             remoteCancoderSimState.setVelocity(mechanismVelocity);

//             return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
//         }
//     }

//     public static double[] getSimulationOdometryTimeStamps() {
//         final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
//         for (int i = 0; i < odometryTimeStamps.length; i++) {
//             odometryTimeStamps[i] = Timer.getFPGATimestamp()
//                     - 0.02
//                     + i * SimulatedArena.getSimulationDt().in(Seconds);
//         }

//         return odometryTimeStamps;
//     }

//     public static SwerveModuleConstants regulateModuleConstantForSimulation(
//             SwerveModuleConstants<?, ?, ?> moduleConstants) {
//         // Skip regulation if running on a real robot
//         if (RobotBase.isReal()) return moduleConstants;

//         // Apply simulation-specific adjustments to module constants
//         return moduleConstants
//                 // Disable encoder offsets
//                 .withEncoderOffset(0)
//                 // Disable motor inversions for drive and steer motors
//                 .withDriveMotorInverted(false)
//                 .withSteerMotorInverted(false)
//                 // Disable CanCoder inversion
//                 .withEncoderInverted(false)
//                 // Adjust steer motor PID gains for simulation
//                 .withSteerMotorGains(TunerConstants.steerGains)
//                 .withSteerMotorGearRatio(moduleConstants.SteerMotorGearRatio)
//                 // Adjust friction voltages
//                 .withDriveFrictionVoltage(Volts.of(0.2))
//                 .withSteerFrictionVoltage(Volts.of(0.2))
//                 // Adjust steer inertia
//                 .withSteerInertia(KilogramSquareMeters.of(0.05));
//     }
// }
