package frc.robot.util;

import com.revrobotics.SparkPIDController;

public class Gains {

  public final double kP;
  public final double kI;
  public final double kD;
  public final double kFF;
  public final double kIz;

  public Gains(double p, double i, double d, double ff, double iz) {
    kP = p;
    kI = i;
    kD = d;
    kFF = ff;
    kIz = iz;
  }

  public void configureController(SparkPIDController controller, double maxSpeed) {
    controller.setP(this.kP);
    controller.setI(this.kI);
    controller.setD(this.kD);
    controller.setFF(this.kFF);
    controller.setIZone(this.kIz);
    controller.setOutputRange(-maxSpeed, maxSpeed);
  }
}