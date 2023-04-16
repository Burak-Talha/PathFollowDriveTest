// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Drive drive = new Drive(new Joystick(0));

  PathPlannerTrajectory straightTrajectory = PathPlanner.loadPath("straight", new PathConstraints(4, 3));
  PathPlannerTrajectory curvedTrajectory = PathPlanner.loadPath("curved", new PathConstraints(4, 3));

  SendableChooser<PPRamseteCommand> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive.setDefaultCommand(new ArcadeDriveCmd(drive));

    // Configure the button bindings
    autoChooser.setDefaultOption("Do nothing", null);
    autoChooser.addOption("Straight Path :", new PPRamseteCommand(
       straightTrajectory,
       drive::robotPosition,
       new RamseteController(),
       Constants.DriveConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
       drive::tankDrive, drive));
    autoChooser.addOption("Curved Path :", new PPRamseteCommand(
       curvedTrajectory,
       drive::robotPosition,
       new RamseteController(),
       Constants.DriveConstants.DIFFERENTIAL_DRIVE_KINEMATICS,
       drive::tankDrive, drive));
    
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
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
