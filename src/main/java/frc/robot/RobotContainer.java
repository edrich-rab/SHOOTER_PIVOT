// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.LimelightTurnAlignCmd;
import frc.robot.commands.ManualPivotCmd;
import frc.robot.commands.PivotPidCmd;
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
  //public static Lights lights = new Lights();

  //private final XboxController joystick = new XboxController(PivotConstants.JOYSTICK_PORT);
  private final Joystick joystick = new Joystick(PivotConstants.JOYSTICK_PORT);


  public RobotContainer() {
    //pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getLeftY()));
    pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getRawAxis(1)));
    configureBindings();
  }

  private void configureBindings(){
    // new JoystickButton(joystick, XboxController.Button.kX.value).onTrue(new PivotPidCmd(pivotSubs, pivotSubs.angleSubwooferShot()));
    // new JoystickButton(joystick, XboxController.Button.kY.value).onTrue(new PivotPidCmd(pivotSubs, 0)); //runs to starting position

    // //TEST
    // new JoystickButton(joystick, XboxController.Button.kB.value).onTrue(new PivotPidCmd(pivotSubs, -4));
    // new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new PivotPidCmd(pivotSubs, -10));

    new JoystickButton(joystick, 3).onTrue(new PivotPidCmd(pivotSubs, pivotSubs.angleSubwooferShot()));
    new JoystickButton(joystick, 4).onTrue(new PivotPidCmd(pivotSubs, 0)); //runs to starting position

    //new JoystickButton(joystick, 5).whileTrue(new LimelightTurnAlignCmd(swerveSubs, () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(2), () -> joystick.getRawAxis(3), false, 0));

    //TEST
    //new JoystickButton(joystick, 5).onTrue(new PivotPidCmd(pivotSubs, -4));
    //new JoystickButton(joystick, 6).onTrue(new PivotPidCmd(pivotSubs, -10));

    /* COMMANDS THAT TURN THE PIVOT TO SPECIFIC ANGLES
    new JoystickButton(joystick, XboxController.Button.kX.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.ampEnc));
    new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.subWooferEnc));
    new JoystickButton(joystick, XboxController.Button.kB.value).onTrue(new PivotPidCmd(pivotSubs, PivotConstants.wingEnc));
    */

  }

  public Command getAutonomousCommand() {
   return null;
  }
}