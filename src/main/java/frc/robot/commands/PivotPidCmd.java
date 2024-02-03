// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotPidCmd extends Command {
 
  PivotSubsystem pivotSub;
  double setpoint; 

  public PivotPidCmd(PivotSubsystem pivotSubs, double setpoint){
    pivotSub = pivotSubs;
    this.setpoint = setpoint;

    addRequirements(pivotSub);
  }
   
  @Override
  public void initialize(){
    pivotSub.init();
    pivotSub.enablePid();
  }

  @Override
  public void execute(){
    pivotSub.changeSetpoint(setpoint);
  }

  @Override
  public void end(boolean interrupted){
    pivotSub.disablePid();
  }

  @Override
  public boolean isFinished() {
    if(pivotSub.atSetpoint()){
      return true;
    }
    return false;
  }
}
