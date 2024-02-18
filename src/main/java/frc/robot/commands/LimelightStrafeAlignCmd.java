// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightStrafeAlignCmd extends Command {
  private SwerveSubsystem swerveSubs;
  private PIDController strafepid;


  public LimelightStrafeAlignCmd(SwerveSubsystem swerveSubsystem) {
    swerveSubs = swerveSubsystem;

    strafepid = new PIDController(0.05, 0, 0);
    addRequirements(swerveSubs);
  }

  @Override
  public void initialize(){

  }


  @Override
  public void execute(){
    double strafeSpeed = strafepid.calculate(LimelightHelpers.getTX("limelight"), 0);

    if(LimelightHelpers.getTV("limelight")){
      swerveSubs.drive(strafeSpeed, 0, 0, false);
    }
    else{
      swerveSubs.stopModules();
    }

  }

  @Override
  public void end(boolean interrupted){
    swerveSubs.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
