fpackage frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import frc.robot.subsystems.vision.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Util.LocalADStarAK;
import frc.robot.generated.TunerConstants;

public class DriveSubsystem extends SubsystemBase implements Vision.VisionConsumer {

	static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	public GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final Module[] modules = new Module[4]; // Front Left (FL), Front Right (fr), Back Left (BL), Back Right (BR)

	private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert = new Alert("Gyro disconnected, using kinematics instead", AlertType.kError);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
	// For delta tracking
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] { new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()};

	private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
	private final Consumer<Pose2d> resetSimulationPoseCallback;

	private final Field2d field = new Field2d();

	public DriveSubsystem(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO rlModuleIO, ModuleIO rrModuleIO, Consumer<Pose2d> resetSimulationPoseCallback) {
		this.gyroIO = gyroIO;
		this.resetSimulationPoseCallback = resetSimulationPoseCallback;
		modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
		modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
		modules[2] = new Module(rlModuleIO, 2, TunerConstants.BackLeft);
		modules[3] = new Module(rrModuleIO, 3, TunerConstants.BackRight);
        var status = DriveConstants.m_orchestra.loadMusic("test.chrp");
        if (!status.isOK()) {
   // log error
   Logger.recordOutput("Song Error","Failed To Load Song");
}


		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

		PhoenixOdometryThread.getInstance().start();

		 AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                DriveConstants.PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

		Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

		sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));

	}

	@Override
	public void periodic() {
		odometryLock.lock();
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

		//stops all modules if driver station is disabled
		if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
			//logs empty setpoints while disabled
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;

		for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

			if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }
			  poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
	}

	public void runVelocity(ChassisSpeeds speeds) {
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

		// Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

		 for (int i = 0; i < 4; i++) {
			// runSetpoint in Module.Java optomizes the setpoints
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

	/** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

	public void stop() {
        runVelocity(new ChassisSpeeds());
    }

	/**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

	  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

	 /** Returns the module states (turn angles and drive velocities) for all of the modules */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

	/** Returns the module positions (turn angles and drive positions) for all of the modules */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

	 /** Returns the measured chassis speeds of the robot */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

	 public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

	public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

	/** Returns the current odometry pose */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

	public Rotation2d getRotation() {
        return getPose().getRotation();
    }

	/** Sets the current odometry pose, use to reset the odometry */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallback.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

	  /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

	public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

	 public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DriveConstants.DRIVE_BASE_RADIUS.in(Meters);
    }

	/** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };


    }

}
