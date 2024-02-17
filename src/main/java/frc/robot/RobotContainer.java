// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ManualPivotCmd;
import frc.robot.commands.PivotPidCmd;
import frc.robot.commands.LimelightDriveAlignCmd;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final PivotSubsystem pivotSubs = new PivotSubsystem();
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  //private final XboxController joystick = new XboxController(PivotConstants.JOYSTICK_PORT);
  private final Joystick joystick = new Joystick(PivotConstants.JOYSTICK_PORT);

  //private final PivotPidCmd pivotAmpShoot = new PivotPidCmd(pivotSubs, 40);
  //private final PivotPidCmd pivotSubShoot = new PivotPidCmd(pivotSubs, 80);
  //private final PivotPidCmd  pivotWingShoot = new PivotPidCmd(pivotSubs, 60);
  //private final PivotPidCmd autoSubwoofShoot = new PivotPidCmd(pivotSubs, pivotSubs.angleSubwooferShot());


  public RobotContainer() {
    //pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getLeftY()));
    //pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getRawAxis(1)));
    configureBindings();
  }

 
  private void configureBindings(){
    //new JoystickButton(joystick, 2).onTrue(new PivotPidCmd(pivotSubs, 8));

    /* COMMANDS THAT TURN THE PIVOT TO SPECIFIC ANGLES
    new JoystickButton(joystick, XboxController.Button.kX.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.ampEnc));
    new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.subWooferEnc));
    new JoystickButton(joystick, XboxController.Button.kB.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.wingEnc));
    */

    /* ALIGN COMMANDS FOR REV
    new JoystickButton(joystick, 10).whileTrue(new S_QuickTurnCommand(swerveSubs, () -> joystick.getRawAxis(0), () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(2), 0));
    new JoystickButton(joystick,10).whileTrue(new S_QuickTurnCommand(swerveSubs, () -> joystick.getRawAxis(0), () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(2), 1));
    new JoystickButton(joystick, 12).whileTrue(new S_DriveCommand(swerveSubs, () -> joystick.getRawAxis(0), () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(2) , 1));
    */

    new JoystickButton(joystick,12 ).whileTrue(new LimelightDriveAlignCmd(swerveSubs, 1));
    //new JoystickButton(joystick, 10).whileTrue(new PivotPidCmd(pivotSubs, 30));
  }

  public Command getAutonomousCommand() {
   return null;
  }
}