/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.climber;

import com.orangefrc.annotation.GenerateJson;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberConstants.*;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.util.Arrays;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  ClimberJson json = new ClimberJson();
  int timesupdated = 0;
  private boolean startUpBool = false;
  private final String filePath = "/home/lvuser/pid/ClimberRotation.json";
  public static boolean hasZeroed = false;
  private static boolean updateDir = false;
  private static boolean lastLimit = false;

  @GenerateJson
  public class Climber {
    double p = 0;
    double i = 0;
    double d = 0;
    double v = 0;
  }

  public class Json {
    private boolean isClockwise = false;

    public boolean getIsClockwise() {
      return isClockwise;
    }

    public Json(boolean isClockwise) {
      this.isClockwise = isClockwise;
    }
  }

  Json climberDirection = new Json(ClimberConstants.climberClockwise);

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
    try (Reader reader = new FileReader(filePath)) {
      climberDirection = TunerConstants.gson.fromJson(reader, Json.class);
      ClimberConstants.climberClockwise = climberDirection.getIsClockwise();
      startUpBool = climberDirection.getIsClockwise();
    } catch (FileNotFoundException e) {
      Logger.recordOutput(
          "ClimberDirection/Error",
          Arrays.stream(e.getStackTrace())
              .map(StackTraceElement::toString)
              .collect(Collectors.joining(System.lineSeparator() + "\tat")));
    } catch (IOException e) {
      System.err.println(Arrays.stream(e.getStackTrace())
          .map(StackTraceElement::toString)
          .collect(Collectors.joining(System.lineSeparator() + "\tat")));
    }
  }

  public void climberDown(double speed) {
    io.setClimberSpeed(speed);
  }

  public void climberUp(double speed) {
    if (Math.abs(io.getClimberPosition()) <= ClimberConstants.maxHeight - 50) {
      io.setClimberSpeed(speed);
    } else {
      io.stopMotor();
    }
  }

  public void setClimberPos(double pos) {
    io.setClimberPos(pos);
  }

  public void zeroClimber() {
    io.zeroClimber();
  }

  public void stopMotor() {
    io.stopMotor();
  }

  @Override
  public void periodic() {
    // hasZeroed = io.hasZeroed();
    json.updateVals();
    // if (json.hasUpdated()) {
    //   timesupdated++;
    //   // io.updatePID(json.getp(), json.geti(), json.getd(), json.getv());
    //   NetworkTableInstance.getDefault()
    //       .getTable("Tuning")
    //       .getStringTopic("ClimberJson/Hasupdated")
    //       .publish()
    //       .set("Has updated the pid" + timesupdated);
    // }
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    Logger.recordOutput("Climber/ClimberDirection", ClimberConstants.climberClockwise);

    // if (!io.getLimitSwitch() && !updateDir) {
    //   updateDir = true;
    // } else {
    //   updateDir = true;
    // }

    Logger.recordOutput("Climber/PrevLimit", lastLimit);
    Logger.recordOutput("Climber/LimitSwitchHasChanged", io.getLimitSwitch() != lastLimit);
    if (!io.getLimitSwitch() && io.getLimitSwitch() != lastLimit) {
      updateDir = true;
    } else {
      updateDir = false;
    }
    lastLimit = io.getLimitSwitch();

    if (updateDir) {
      System.out.println("Updating the direction: " + updateDir);
      if (io.getVelocity() < 0) {
        ClimberConstants.climberClockwise = true;
      } else if (io.getVelocity() > 0) {
        ClimberConstants.climberClockwise = false;
      } else {
      }
      climberDirection = new Json(ClimberConstants.climberClockwise);

      try (Writer writer = new FileWriter(filePath)) {
        TunerConstants.gson.toJson(climberDirection, writer);
        writer.close();
      } catch (IOException e) {
        System.err.println(Arrays.stream(e.getStackTrace())
            .map(StackTraceElement::toString)
            .collect(Collectors.joining(System.lineSeparator() + "\tat")));
      }
      updateDir = false;
    }
    Logger.recordOutput("Climber/UpdateDir", updateDir);

    // SmartDashboard.putNumber("ClimberSetpoint", io.getSetpoint());
    // SmartDashboard.putNumber(
    //     "ClimberError", (io.getSetpoint() - (60 *
    // inputs.climberVelocity.in(RotationsPerSecond))));
  }

  @Override
  public void simulationPeriodic() {}
}
