package frc.robot.util;

public class Gains {
    public Gains(double p, double i, double d, double ff, double iz) {
      kP = p;
      kI = i;
      kD = d;
      kFF = ff;
      kIz = iz;
    }

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kFF;
    public final double kIz;
}
