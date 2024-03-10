package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;

public class ManualPivotCmd extends Command {

  PivotSubsystem pivotSub;
  DoubleSupplier moveSpeed;

  public ManualPivotCmd(PivotSubsystem pivotSubsystem, DoubleSupplier speed){
    pivotSub = pivotSubsystem;
    moveSpeed = speed;

    addRequirements(pivotSub);
  }

  @Override
  public void initialize(){
    pivotSub.init();
    pivotSub.disablePid();
  }

  @Override
  public void execute(){
    if (Math.abs(moveSpeed.getAsDouble()) > Constants.PivotConstants.MAX_SPEED){
      pivotSub.setManualSpeed(Math.copySign(Constants.PivotConstants.MAX_SPEED, moveSpeed.getAsDouble()));
    }
    else{
      pivotSub.setManualSpeed(moveSpeed.getAsDouble());
    } 
  }

  @Override 
  public void end(boolean interrupted){
    
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}