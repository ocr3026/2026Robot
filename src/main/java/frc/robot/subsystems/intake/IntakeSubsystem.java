 package frc.robot.subsystems.intake;

 import com.revrobotics.spark.SparkClosedLoopController;
 import com.revrobotics.spark.SparkFlex;
 import com.revrobotics.spark.SparkMax;
 import com.revrobotics.spark.SparkBase.ControlType;
 import com.revrobotics.spark.SparkLowLevel.MotorType;

 import com.ctre.phoenix6.hardware.TalonFX;

 import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
 import edu.wpi.first.wpilibj.motorcontrol.Spark;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

 public class IntakeSubsystem extends SubsystemBase {

  
 public static SparkFlex motor23 = new SparkFlex(23, MotorType.kBrushless);
 public static SparkClosedLoopController mController23 = motor23.getClosedLoopController();
 public static SparkMax motor24 = new SparkMax(24, MotorType.kBrushless);
 public static SparkClosedLoopController mController24 = motor24.getClosedLoopController();

   public void setvelocity(double speed){
     motor23.set(speed);
 }

   public void setvelocitylift(double speed){
     motor24.set(speed);
   }
   /** Creates a new ExampleSubsystem. */
   public IntakeSubsystem() {}

   /**
    * Example command factory method.
    *
    * @return a command
    */
  public Command exampleMethodCommand() {
     // Inline construction of command goes here.
     // Subsystem::RunOnce implicitly requires `this` subsystem.
     return runOnce(
         () -> {
           /* one-time action goes here */
         });
   }

  /**
    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
    *
    * @return value of some boolean subsystem state, such as a digital sensor.
    */
   public boolean exampleCondition() {
     // Query some boolean state, such as a digital sensor.
     return false;
   }

  @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }

   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
   }
 }
