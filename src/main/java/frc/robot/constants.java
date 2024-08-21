package frc.robot;

import edu.wpi.first.math.util.Units;

public class constants {
    public static final class ModuleConstants {
    
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //(MODIFY)
        public static final double kDriveMotorGearRatio = 1 / 8.14; //(MODIFY)
        public static final double kTurningMotorGearRatio = 1 / 21.4286; //(MODIFY)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kITurning = 0;
        public static final double KDTurning = 0;
        
    }

    public static final class DriveConstants {
        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(12.5);
    }
}
