// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Comands.TeleopSwerve;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem s_Swerve = new SwerveSubsystem();

  
    XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);  // USE XI INPUT
    private final JoystickButton zeroGyro = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
    private final JoystickButton robotCentric = new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value);

    


  public RobotContainer() {

    s_Swerve.setDefaultCommand(
            new TeleopSwerve(
              s_Swerve,
              () -> driverJoystick.getLeftX(),
              () -> driverJoystick.getLeftY(),
              () -> driverJoystick.getRightX(),
              () -> robotCentric.getAsBoolean()
              )
    );

    configureBindings();
  }

  private void configureBindings() {
     zeroGyro.onTrue( new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  public boolean  fieldCentric() {
    return robotCentric.getAsBoolean();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
