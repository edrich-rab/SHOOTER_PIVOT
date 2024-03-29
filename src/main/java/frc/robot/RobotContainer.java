// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.LimelightFlash;
import frc.robot.commands.LimelightTurnAlignCmd;
import frc.robot.commands.ManualPivotCmd;
import frc.robot.commands.PivotPidAlignCmd;
import frc.robot.commands.PivotPidCmd;
import frc.robot.commands.RunToTopLim;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final PivotSubsystem pivotSubs = new PivotSubsystem();
  private final SwerveSubsystem swerveSubs = new SwerveSubsystem(); 

  private final XboxController joystick = new XboxController(PivotConstants.JOYSTICK_PORT);
  //private final Joystick joystick = new Joystick(PivotConstants.JOYSTICK_PORT);


  public RobotContainer() {
    pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getLeftY()));
    // pivotSubs.setDefaultCommand(new ManualPivotCmd(pivotSubs, () -> joystick.getRawAxis(1)));
    configureBindings();
  }

  private void configureBindings(){

    // new JoystickButton(joystick, 8).onTrue(new RunToTopLim(pivotSubs));
    //new JoystickButton(joystick, 10).onTrue(new PivotPidCmd(pivotSubs, 45));
    // new JoystickButton(joystick, 12).onTrue(new PivotPidCmd(pivotSubs, 30));

    // new JoystickButton(joystick, 7).onTrue(new PivotPidAlignCmd(pivotSubs));

    // new JoystickButton(joystick, 9).whileTrue(new LimelightTurnAlignCmd(swerveSubs, () -> joystick.getX(), () -> joystick.getY(), () -> joystick.getZ(), false, 0));

    new JoystickButton(joystick, XboxController.Button.kB.value).whileTrue(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink("limelight")));
    new JoystickButton(joystick, XboxController.Button.kB.value).whileFalse(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")));
  
     new JoystickButton(joystick, XboxController.Button.kA.value).onTrue(new PivotPidCmd(pivotSubs, 45));
     new JoystickButton(joystick, XboxController.Button.kB.value).onTrue(new PivotPidCmd(pivotSubs, 35));
    new JoystickButton(joystick, XboxController.Button.kY.value).onTrue(new RunToTopLim(pivotSubs));
  }

  public Command getAutonomousCommand() {
   return null;
  }
}