package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DrivetrainIO {
    @AutoLog
    public static class DriveIOInputs {public static class Inputs implements LoggableInputs {
        private class Wheel {
          public double positionMeters;
          public double velocityMetersPerSec;
          public double AppliedVolts;
          public double CurrentAmps;
    
          public Wheel() {}
        }
    
        public Wheel frontLeft = new Wheel();
        public Wheel frontRight = new Wheel();
        public Wheel backLeft = new Wheel();
        public Wheel backRight = new Wheel();
        public Rotation2d gyroYaw = new Rotation2d();
    
        @Override
        public void toLog(LogTable table) {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'toLog'");
        }
    
        @Override
        public void fromLog(LogTable table) {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
        }
    
      }
    }
}