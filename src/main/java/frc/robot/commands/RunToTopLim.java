// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class RunToTopLim extends Command {
  PivotSubsystem pivotSubs;
  double offset = 5;

  
  public RunToTopLim(PivotSubsystem pivotSubsystem){
    pivotSubs = pivotSubsystem;
    addRequirements(pivotSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    pivotSubs.init();
    pivotSubs.enablePid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    pivotSubs.changeSetpoint(60 + offset);

  }

  @Override
  public void end(boolean interrupted){
    pivotSubs.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if(pivotSubs.topLimitSwitchPressed() || pivotSubs.atSetpoint()){
      return true;
    }
    return false;
  }
}
