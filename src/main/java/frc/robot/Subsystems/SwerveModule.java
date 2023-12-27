package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.config.SwerveModuleConstants;
import frc.robot.Constants;
//import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

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
    //private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(ModuleConstants.kSTurning, ModuleConstants.kVTurning, ModuleConstants.kATurning);
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(ModuleConstants.kSDrive, ModuleConstants.kVDrive, ModuleConstants.kADrive);
    
    
    private final CANCoder CANCoder ;

    private final CANCoderSimCollection CANCoderSim; 

    public int moduleNumber;

    private double drivePID = 0;
    private double turnOutput = 0;
    private double driveFF = 0;
    private double driveOutput = 0; 
    private double m_simTurnEncoderDistance = 0;
    private double m_simTurnEncoderVelocity = 0;
    private int TurnEncodeInt = 0;
    private int TurnEncodeVelocityInt = 0;
    

      // Using FlywheelSim as a stand-in for a simple motor

      FlywheelSim driveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Constants.ModuleConstants.kVDrive, Constants.ModuleConstants.kADrive), DCMotor.getNEO(1), 1 / Constants.ModuleConstants.kDriveMotorGeatRatio);
      FlywheelSim turnSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Constants.ModuleConstants.kVTurning, Constants.ModuleConstants.kATurning), DCMotor.getNEO(1), 1 / Constants.ModuleConstants.kTurningMotorGearRatio);

      
  

    ShuffleboardTab PIDtab = Shuffleboard.getTab("PID Tuning");

   
   

    public SwerveModule(int moduleNumber, SwerveModuleConstants  moduleConstraints, ShuffleboardLayout container  ) {
        this.moduleNumber = moduleNumber;
        this.CANCoder = new CANCoder(moduleConstraints.cancoderID);
        this.CANCoder.configMagnetOffset(-1 * moduleConstraints.angleOffset); //Possible reversed //FIXME
        this.CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        this.CANCoderSim = CANCoder.getSimCollection();

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

        simulationInit(moduleConstraints);

        resetDriveEncoders();
        resetTurnEncoders();
    }

    public SwerveModulePosition  getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getTurnPosition()));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(CANCoder.getAbsolutePosition());
      }

    public double getTurnPosition() { //Module Heading
        return ((2 * Math.PI)/360) *  CANCoder.getAbsolutePosition();
    }

    public void resetToAbsolute() {
        double absolutePosition =  -getTurnPosition();
        turnEncoder.setPosition(absolutePosition);
        driveEncoder.setPosition(0.0);
        SmartDashboard.putNumber("Reset Cancoder absolute position", absolutePosition);
        SmartDashboard.putNumber("Reset Cancoder value", getCanCoder().getDegrees());
        
      }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
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



        drivePID = drivePIDControler.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        turnOutput = turningPIDControler.calculate(getTurnPosition(), state.angle.getRadians());
        driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);
        driveOutput = drivePID + driveFF;
        double driveFinalOutput = MathUtil.clamp(driveOutput, -12.0, 12.0);
        driveMotor.setVoltage(driveFinalOutput);
        turnMotor.set(turnOutput);

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
    public void simulationInit(SwerveModuleConstants  moduleConstraints) {
       REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
       REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNEO(1));
        

       



    }
    public void simulationPeriodic(double dt) {
        REVPhysicsSim.getInstance().run();

        turnSim.setInputVoltage(turnOutput * RobotController.getBatteryVoltage());
    driveSim.setInputVoltage(driveOutput);

    turnSim.update(dt);
    driveSim.update(dt);

    m_simTurnEncoderDistance = turnSim.getAngularVelocityRadPerSec() * dt;
    m_simTurnEncoderDistance = Math.toDegrees(m_simTurnEncoderDistance);
    TurnEncodeInt = (int)Math.round(m_simTurnEncoderDistance);
    CANCoderSim.addPosition(TurnEncodeInt);
    m_simTurnEncoderVelocity = turnSim.getAngularVelocityRadPerSec();
    m_simTurnEncoderVelocity =Math.toDegrees(m_simTurnEncoderVelocity);
    TurnEncodeVelocityInt = (int)Math.round(m_simTurnEncoderVelocity);
    CANCoderSim.setVelocity(TurnEncodeVelocityInt);


  }
        
    
        
      




}