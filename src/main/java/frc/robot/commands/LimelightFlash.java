// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;

public class LimelightFlash extends Command {

  public LimelightFlash() {
    
  }

  
  @Override
  public void initialize(){

  }

 
  @Override
  public void execute(){
   LimelightHelpers.setLEDMode_ForceBlink("limelight");
  }

  @Override
  public void end(boolean interrupted){

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
