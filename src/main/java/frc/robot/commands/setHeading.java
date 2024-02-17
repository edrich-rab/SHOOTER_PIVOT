// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class setHeading extends Command {
  private SwerveSubsystem swerveSubs;
  private double heading;
  private PIDController pid;

  public setHeading(SwerveSubsystem swerveSubsystem, double heading) {
    swerveSubs = swerveSubsystem;
    this.heading = heading;

    pid = new PIDController(0.05, 0, 0);
    addRequirements(swerveSubs);
  }
  @Override
  public void initialize(){

  }

  @Override
  public void execute(){
    SwerveModuleState[] states; 
    double pidSpeed = pid.calculate(swerveSubs.getRotation2d().getDegrees(), heading);

    states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, pidSpeed, swerveSubs.getRotation2d())
    );

    if(LimelightHelpers.getTV("limelight")){
      swerveSubs.setModuleStates(states);
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
