package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.Lib.config.*;

public class SwerveModule extends SubsystemBase {

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
    
    
    private final CANCoder CANCoder ;

    public int moduleNumber;

    ShuffleboardTab PIDtab = Shuffleboard.getTab("PID Tuning");

   

    public SwerveModule(int moduleNumber, SwerveModuleConstants  moduleConstraints, ShuffleboardLayout container  ) {
        this.moduleNumber = moduleNumber;
        this.CANCoder = new CANCoder(moduleConstraints.cancoderID);
        this.CANCoder.configMagnetOffset(-1 * moduleConstraints.angleOffset); //Possible reversed //FIXME
        this.CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        driveMotor = new CANSparkMax(moduleConstraints.driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(moduleConstraints.angleMotorID, MotorType.kBrushless);

        //driveMotor.setInverted(driveMotorReversed);  //no use
        //turnMotor.setInverted(turnMotorReversed);    // no use

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRpm2MeterPerSecond);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurnEncoderRpm2RadperSecond);

       
        turningPIDControler.enableContinuousInput(-Math.PI, Math.PI);

        resetDriveEncoders();
        resetTurnEncoders();
    }

    public SwerveModulePosition  getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getTurnPosition()));
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(CANCoder.getAbsolutePosition());
      }

    public double getTurnPosition() { //Module Heading
        return ((2 * Math.PI)/360) *  CANCoder.getAbsolutePosition();
    }

    public void resetToAbsolute() {
        double absolutePosition =  -getTurnPosition();//work on the ideal version of this so it boots up correctly
        turnEncoder.setPosition(absolutePosition);
        driveEncoder.setPosition(0.0);
        SmartDashboard.putNumber("Reset Cancoder absolute position", absolutePosition);
        SmartDashboard.putNumber("Reset Cancoder value", getCanCoder().getDegrees());
      }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    

    public void resetDriveEncoders() {
        driveEncoder.setPosition(0.0);
        
    }
    public void resetTurnEncoders() {
        
        turnEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void SetDesiredState(SwerveModuleState desieredState) {
        if(Math.abs(desieredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        SwerveModuleState state = SwerveModuleState.optimize(desieredState, getState().angle);
        SmartDashboard.putNumber("SpeedMetersPerSecond", getDriveVelocity());
        SmartDashboard.putNumber("AbsoluteEncoderRad", getTurnPosition());
        SmartDashboard.putNumber("AbsoluteEncoderAngle", CANCoder.getAbsolutePosition());
        SmartDashboard.putNumber("desiredState", state.speedMetersPerSecond);



        final double drivePID = drivePIDControler.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        final var turnPID = turningPIDControler.calculate(getTurnPosition(), state.angle.getRadians());
        final double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);
        final var turnFF = turnFeedForward.calculate(turningPIDControler.getSetpoint().velocity);
        final double driveOutput = drivePID + driveFF;
        final var turnOutput = turnPID + turnFF;
        driveMotor.setVoltage(driveOutput);
        turnMotor.setVoltage(turnOutput);

        SmartDashboard.putNumber("turnPID Setpoint Velocity", turningPIDControler.getSetpoint().velocity);
    SmartDashboard.putNumber("PID driveOutput", driveOutput);
    SmartDashboard.putNumber("PID turnOutput", turnOutput);
    SmartDashboard.putNumber("Feedforward", driveFeedForward.calculate(desieredState.speedMetersPerSecond));
    SmartDashboard.putNumber("PID Output", drivePIDControler.calculate(getDriveVelocity(), state.speedMetersPerSecond));

        

    }

    public void stop() {
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }




}