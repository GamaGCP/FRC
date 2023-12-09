package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class ModuleConstants {
        
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);//
        public static final double kDriveMotorGeatRatio = 1 / 5.8642;//
        public static final double kTurningMotorGearRatio = 1 / 18.0;//

        public static final double kDriveEncoderRot2Meter =  kDriveMotorGeatRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRpm2MeterPerSecond = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kTurnEncoderRpm2RadperSecond = kTurnEncoderRot2Rad / 60;

        //Turning PID values

        public static final double kPTurning = 0.1;//
        public static final double kITurning = 0.0;//
        public static final double kDturning = 0.0;//

        //Turning FeedForward Values

        public static final double kSTurning = 0.0; //
        public static final double kVTurning = 0.0; //
        public static final double kATurning = 0.0;

        //Drive Pid Values

        public static final double kPDrive = 0.1;//
        public static final double kIDrive = 0.0;//
        public static final double kDDive = 0.0;//

        //Drive FeedForward Values

        public static final double kSDrive = 0.0; //
        public static final double kVDrive = 0.0; //
        public static final double kADrive = 0.0;

    }
    
    public static final class DriveConstants {

        public static final int kTeleDriveSpeedReduction = 4;    // 1/int
        public static final int kTeleAngularSpeedReduction = 4;  // 1/int

        // Distance between right and left wheels

        public static final double kTrackWidth = Units.inchesToMeters(21);

        // Distance between front and back wheels

        public static final double kWheelBase = Units.inchesToMeters(25.5);
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        //drive motor ports

        public static final int kFrontLeftDriveMotorPort = 0;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackLeftMotorPort = 4;
        public static final int kBackRightMotorPort = 6;

        //turn motor ports

        public static final int kFrontLeftTurnMotorPort = 1;
        public static final int kFrontRightTurnMotorPort = 3;
        public static final int kBackLeftTurnMotorPort = 5;
        public static final int kBackRightTurnMotorPort = 7;

        //drive motor directions

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //turn motor directions

        public static final boolean kFrontLeftTurnEncoderReversed = false;
        public static final boolean kFrontRightTurnEncoderReversed = false;
        public static final boolean kBackLeftTurnEncoderReversed = false;
        public static final boolean kBackRightTurnEncoderReversed = false;

        //CANCoder ports

        public static final int kFrontLeftCANCoderPort = 0;
        public static final int kFrontRightCANCoderPort = 1;
        public static final int kBackLeftCANCoderPort = 2;
        public static final int kBackRightCANCoderPort = 3;

        //CANCoder offset

        public static final double kFrontLeftCANCoderOffsetRad = 0.0;
        public static final double kFrontRightCANCoderOffsetRad = 0.0;
        public static final double kBackLeftCANCoderoderOffsetRad = 0.0;
        public static final double kBackRightCANCoderOffsetRad = 0.0;

        //CANCoder direction 
        public static final boolean kFrontLeftCANCoderReversed = false;
        public static final boolean kFrontRightCANCoderReversed = false;
        public static final boolean kBackLeftCANCoderReversed = false;
        public static final boolean kBackRightCANCoderReversed = false;

        //physical max speeds

        public static final double kPhysicalMaxSpeedMeterPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kPhysicalMaxAngularAccelerationRadiansPerSecond = 2 * Math.PI;

        //Max teleop speeds
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMeterPerSecond / kTeleDriveSpeedReduction;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / kTeleAngularSpeedReduction;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        



    }
}
