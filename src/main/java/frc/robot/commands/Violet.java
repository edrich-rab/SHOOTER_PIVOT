package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Lights;

public class Violet extends Command {
  private Lights lights;

  public Violet(Lights subs) {
    lights = subs;
    addRequirements(subs);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    lights.lavendar();
  }

  @Override
  public void end(boolean interrupted) {
    lights.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}