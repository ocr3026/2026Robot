package frc.robot.subsystems.drive;

public class PIDJson {
    private double p;
    private double i;
    private double d;

    public double getP() {return p;}
    public double getI() {return i;}
    public double getD() {return d;}

    public PIDJson(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}
