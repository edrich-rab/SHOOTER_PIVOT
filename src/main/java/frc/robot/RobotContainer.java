// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ManualPivotCmd;
import frc.robot.commands.PivotPidCmd;
import frc.robot.commands.Violet;
import frc.robot.subsystems.Lights;
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
  public static Lights lights = new Lights();

  private final XboxController joystick = new XboxController(PivotConstants.JOYSTICK_PORT);
  //private final Joystick joystick = new Joystick(PivotConstants.JOYSTICK_PORT);


  public RobotContainer() {
    pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getLeftY()));
    //pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getRawAxis(1)));
    configureBindings();
  }

  //0.083 encoder counts in 1 degree
  //
  private void configureBindings(){
    new JoystickButton(joystick, XboxController.Button.kX.value).onTrue(new PivotPidCmd(pivotSubs, pivotSubs.angleSubwooferShot()));
    new JoystickButton(joystick, XboxController.Button.kY.value).onTrue(new PivotPidCmd(pivotSubs, 0)); //runs to starting position
    new JoystickButton(joystick, XboxController.Button.kB.value).onTrue(new PivotPidCmd(pivotSubs, -4));
    new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new PivotPidCmd(pivotSubs, -10));

    //new JoystickButton(joystick, XboxController.Button.kX.value).onTrue(new Violet(lights));
      

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

    //new JoystickButton(joystick,12 ).whileTrue(new LimelightDriveAlignCmd(swerveSubs, 1));
    //new JoystickButton(joystick, 10).whileTrue(new PivotPidCmd(pivotSubs, 30));
   // new JoystickButton(joystick, 11).whileTrue(new setHeading(swerveSubs, 90));
  }

  public Command getAutonomousCommand() {
   return null;
  }
}