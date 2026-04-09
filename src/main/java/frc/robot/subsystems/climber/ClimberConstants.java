/* Generated and Formatted by yours truly <3*/
package frc.robot.subsystems.climber;

import com.google.gson.Gson;

public class ClimberConstants {
  public static final int climberMotorID = 30;

  public static boolean climberClockwise = false;

  public static final double midPos = 3000;

  public static boolean downNegative = false;
  public static boolean upNegative = false;

  public static Gson climberGson = new Gson();

  public static final int maxHeight = 600;

  public class Animal {
    public String sound;

    public Animal(String sound) {
      this.sound = sound;
    }
  }

  Animal horse = new Animal("Neigh");
  Animal cow = new Animal("Moo");

  public void callAnimals() {
    System.out.println(horse.sound);
    System.out.println(cow.sound);
  }
}
