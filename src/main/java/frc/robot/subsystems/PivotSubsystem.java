// hardstop at the start position
// subwoofer position, amp position, feed position?

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;

public class PivotSubsystem extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private RelativeEncoder encoder;
  private DigitalInput limitSwitch;

  private PIDController pid;
  private double setpoint;
  private boolean pidOn;
  private double setpointTolerance;

  private double manualSpeed;
  private double maxPidSpeed;

 // private double robotHeight; // in feet
  //private NetworkTable table;
  //private NetworkTableEntry robotPos;

  public PivotSubsystem(){
    pivotMotor = new CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    limitSwitch = new DigitalInput(PivotConstants.PIVOT_LIMIT);
    encoder = pivotMotor.getEncoder();
    
    pid = new PIDController(0.01, 0, 0);
    setpoint = 0;
    setpointTolerance = 2.5;

    manualSpeed = 0;
    maxPidSpeed = 0.2;

   // robotHeight = 1;
    //table = NetworkTableInstance.getDefault().getTable("limelight");
    //robotPos = table.getEntry("");
  }

  public void init(){
    pivotMotor.setIdleMode(IdleMode.kBrake);
  }

  //////////////////////
  // ENCODER METHODS  //
  //////////////////////

  public double returnEncoder(){
    return encoder.getPosition();
  }

  public void resetEnc(){
    encoder.setPosition(0);
  }

  ///////////////////
  //  PID METHODS //
  //////////////////

  public void enablePid(){
    pidOn = true;
    changeSetpoint(returnEncoder()); //???
  }

  public void disablePid(){
    pidOn = false;
  }

  public void changeSetpoint(double newSetpoint){
    setpoint = newSetpoint;
  }

  /////////////////////
  //  CHECK METHODS  //
  ////////////////////

  public boolean topLimitSwitchPressed(){
    return limitSwitch.get();
  }


  public boolean atSetpoint(){ 
    double error1 = setpoint - returnEncoder();
    return Math.abs(error1) < setpointTolerance;
  }

  /////////////////////////
  //  MOVEMENT METHODS   //
  ////////////////////////

  public void stopMotor(){
    pivotMotor.set(0);
  }

  public double deadzone(double speed){
    if(speed < 0.1 && speed > -0.1 ){
      return 0;
    }
    else{
      return speed;
    }
  }

  public void setManualSpeed(double speed){
    manualSpeed = deadzone(speed);
  }

  public void adjustPidShoot(){
   
  }
 
  @Override
  public void periodic() {

    if(topLimitSwitchPressed()){
      resetEnc();
    }
  
    double pidSpeed = 0;

    if(pidOn){
      pidSpeed = -pid.calculate(setpoint, encoder.getPosition());
    }
    else{
      pidSpeed = manualSpeed;
    }

    
    if(topLimitSwitchPressed() && pidSpeed > 0){
      pidSpeed = 0;
    }
    else if(pidSpeed > maxPidSpeed){
      pidSpeed = maxPidSpeed;
    }
    else if(pidSpeed < -maxPidSpeed){
      pidSpeed = - maxPidSpeed;
    }
    
  
    pivotMotor.set(pidSpeed);

    SmartDashboard.putBoolean("Pid On?", pidOn);
    SmartDashboard.putNumber("Speed", pidSpeed);
    SmartDashboard.putBoolean("Limit switch pressed?", topLimitSwitchPressed());
    SmartDashboard.putNumber("Encoder values", returnEncoder());

  }
}
