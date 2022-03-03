// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.SimSwerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.InsidePath;
import frc.robot.commands.OutsidePath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public SwerveSubsystem swerve = new SimSwerve();

  public RobotContainer() {
    SmartDashboard.putNumber("Start Position", 0);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int position = (int)SmartDashboard.getNumber("Start Position", 0);

    Command autoCommand = new PrintCommand("Do Nothing");

    switch(position){
      case 0: startPosition = new Pose2d(6.5, 5, Rotation2d.fromDegrees(-30));
              autoCommand = new InsidePath(swerve);
              break;
      case 1: startPosition = new Pose2d(6.5, 5, Rotation2d.fromDegrees(-30));
              autoCommand = new OutsidePath(swerve);
              break;
      default: ;
    }


    swerve.setPose(startPosition);
    return autoCommand;
  }

  private Pose2d startPosition = new Pose2d(0,0,Rotation2d.fromDegrees(0));
}
