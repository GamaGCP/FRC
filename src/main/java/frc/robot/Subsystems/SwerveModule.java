package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    //define the differnt components 
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    //feedback
    private final ProfiledPIDController  turningPIDControler = new ProfiledPIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDturning, new TrapezoidProfile.Constraints(DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, DriveConstants.kPhysicalMaxAngularAccelerationRadiansPerSecond));
    private final PIDController drivePIDControler = new PIDController(ModuleConstants.kPDrive, ModuleConstants.kIDrive, ModuleConstants.kDDive);

    //feed Forward
    private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(ModuleConstants.kSTurning, ModuleConstants.kVTurning, ModuleConstants.kATurning);
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(ModuleConstants.kSDrive, ModuleConstants.kVDrive, ModuleConstants.kADrive);
    private final AnalogInput absoluteEncoder;
    private final boolean CANCoderReversed;
    private final double CANCoderOffsetRad;
    private final CANcoder CANCoder ;

   

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int CANCoderId, boolean CANCoderReversed, double CANCoderOffset ) {
        this.CANCoderOffsetRad = CANCoderOffset;
        this.CANCoderReversed = CANCoderReversed;
        absoluteEncoder = new AnalogInput(CANCoderId);
        CANCoder = new CANcoder(CANCoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRpm2MeterPerSecond);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRpm2RadperSecond);

       
        turningPIDControler.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= CANCoderOffsetRad;
        return angle * (CANCoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void SetDesiredState(SwerveModuleState state) {
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        final double drivePID = drivePIDControler.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        final double turnPID = turningPIDControler.calculate(getTurnVelocity(), state.angle.getRadians());
        final double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);
        final double turnFF = turnFeedForward.calculate(turningPIDControler.getSetpoint().velocity);
        final double driveOutput = drivePID + driveFF;
        final double turnOutput = turnPID + turnFF;
        driveMotor.setVoltage(driveOutput);
        turnMotor.setVoltage(turnOutput);
        

    }

    public void stop() {
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }




}