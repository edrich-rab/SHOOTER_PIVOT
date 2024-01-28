// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualPivotCmd;
import frc.robot.commands.PivotPidCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final PivotSubsystem pivotSubs = new PivotSubsystem();
  private final Joystick joystick = new Joystick(PivotConstants.JOYSTICK_PORT);
  private final PivotPidCmd pivotAmpShoot = new PivotPidCmd(pivotSubs, 20);
  private final PivotPidCmd pivotSubShoot = new PivotPidCmd(pivotSubs, 40);
  private final PivotPidCmd  pivotWingShoot = new PivotPidCmd(pivotSubs, 50);


  public RobotContainer() {
    pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, joystick.getY()));
    configureBindings();
  }

 
  private void configureBindings(){
    new JoystickButton(joystick, 0).onTrue(pivotAmpShoot);
    new JoystickButton(joystick, 1).onTrue(pivotSubShoot);
    new JoystickButton(joystick, 2).onTrue(pivotWingShoot);
  }

 
  public Command getAutonomousCommand() {
   return null;
  }
}
