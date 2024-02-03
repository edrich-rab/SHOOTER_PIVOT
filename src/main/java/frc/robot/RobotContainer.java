// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ManualPivotCmd;
import frc.robot.commands.PivotPidCmd;
import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final PivotSubsystem pivotSubs = new PivotSubsystem();
  private final XboxController joystick = new XboxController(PivotConstants.JOYSTICK_PORT);
  //private final PivotPidCmd pivotAmpShoot = new PivotPidCmd(pivotSubs, 40);
  //private final PivotPidCmd pivotSubShoot = new PivotPidCmd(pivotSubs, 80);
  //private final PivotPidCmd  pivotWingShoot = new PivotPidCmd(pivotSubs, 60);


  public RobotContainer() {
    pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getLeftY()));
    configureBindings();
  }

 
  private void configureBindings(){
    new JoystickButton(joystick, XboxController.Button.kX.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.ampEnc));
    new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.subWooferEnc));
    new JoystickButton(joystick, XboxController.Button.kB.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.wingEnc));
  }

 
  public Command getAutonomousCommand() {
   return null;
  }
}
