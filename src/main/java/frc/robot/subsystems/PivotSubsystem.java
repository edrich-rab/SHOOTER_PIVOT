// hardstop at the start position
// subwoofer position, amp position, feed position?

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

public class PivotSubsystem extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private AbsoluteEncoder encoder;
  private DigitalInput limitSwitch;
  private DigitalInput bottomLimitSwitch;

  private PIDController pid;
  private double setpoint;
  private boolean pidOn;
  private double setpointTolerance;

  private double manualSpeed;
  private double maxPidSpeed;
  
  //private double distance;

  private double finalAngle;

  private double horizontalDist;
  private double statsAngleCalc; 

  private double statsCalcAngle;

  public PivotSubsystem(){
    pivotMotor = new CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    pivotMotor.setInverted(false);
    limitSwitch = new DigitalInput(PivotConstants.PIVOT_TOP_LIMIT);
    bottomLimitSwitch = new DigitalInput(PivotConstants.PIVOT_BOTTOM_LIMIT);
    encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setZeroOffset(270);
    encoder.setPositionConversionFactor(360);
    
    pid = new PIDController(0.01, 0, 0);
    setpoint = 0;
    setpointTolerance = 1.5;

    manualSpeed = 0;
    maxPidSpeed = 0.2;

    pid.enableContinuousInput(0, 260); 
    pid.setTolerance(1.5);

  }

  public void init(){
    pivotMotor.setIdleMode(IdleMode.kCoast);
  }

  //////////////////////
  // ENCODER METHODS  //
  //////////////////////

  public double returnEncoder(){
    return encoder.getPosition();
  }

  public void resetEnc(){
    encoder.setZeroOffset(encoder.getPosition());
  }

  ///////////////////
  //  PID METHODS //
  //////////////////

  public void enablePid(){
    pidOn = true;
    changeSetpoint(returnEncoder()); 
  }

  public void disablePid(){
    pidOn = false;
  }

  public void changeSetpoint(double newSetpoint){
    setpoint = 360 - newSetpoint;
  }

  /////////////////////
  //  CHECK METHODS  //
  ////////////////////

  public boolean topLimitSwitchPressed(){
    return !limitSwitch.get();
  }

  public boolean bottomLimitSwitchPressed(){
    return !bottomLimitSwitch.get();
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
    if(Math.abs(speed) < 0.1){
      return 0;
    }
    else{
      return speed;
    }
  }

  public void setManualSpeed(double speed){
    manualSpeed = deadzone(speed);
  }

  /////////////////////////
  //  LIMELIGHT METHODS  //
  ////////////////////////

  // returns the encoder count of the angle shooter should go to
  public double angleSubwooferShot(){
    return finalAngle;
  }

  public double getDistanceFromTarget(){
    double distance = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getTranslation().getNorm();
    return distance;
  }

  public double returnCalcAngle (){
    return statsCalcAngle;
  }
 
  @Override
  public void periodic() {
    horizontalDist = Units.inchesToMeters(43) / (Math.tan(Units.degreesToRadians(LimelightHelpers.getTY("limelight") + 15)));
    statsCalcAngle = 84.3 + (-9.18 * horizontalDist) + (0.369 * Math.pow(horizontalDist, 2));

    finalAngle = Units.radiansToDegrees(Math.atan((Units.inchesToMeters(43) + Units.inchesToMeters(21))/horizontalDist));
    statsAngleCalc = 86 + (-9.5 * horizontalDist) + (0.386 * Math.pow(horizontalDist, 2)); 
    statsAngleCalc /= 2;
  
    double pidSpeed = 0;

    if(pidOn){
      pidSpeed = pid.calculate(encoder.getPosition(), setpoint);
    }
    else{
      pidSpeed = manualSpeed;
    }

    if(topLimitSwitchPressed() && pidSpeed < 0){
      pidSpeed = 0;
    }
    else if(bottomLimitSwitchPressed() && pidSpeed > 0){
      pidSpeed = 0;
    }
    else if(pidSpeed > maxPidSpeed){
      pidSpeed = maxPidSpeed;
    }
    else if(pidSpeed < -maxPidSpeed){
      pidSpeed = maxPidSpeed;
    }

//    pivotMotor.set(pidSpeed);

    SmartDashboard.putBoolean("Pid On?", pidOn);
    SmartDashboard.putNumber("Speed", pidSpeed);
    SmartDashboard.putBoolean("Top limit switch pressed?", topLimitSwitchPressed());
    SmartDashboard.putNumber("Encoder values", returnEncoder());
    SmartDashboard.putNumber("Encoder Conversion Factor", encoder.getPositionConversionFactor() );
    SmartDashboard.putNumber("Zero Offset", encoder.getZeroOffset());
    SmartDashboard.putBoolean("Bottom limit switch pressed?", bottomLimitSwitchPressed());
    SmartDashboard.putNumber("calculated angle", finalAngle);
    SmartDashboard.putNumber("distance from limelight", horizontalDist);
    SmartDashboard.putBoolean("at setpoint?", pid.atSetpoint());
    SmartDashboard.putNumber("pid setpoint", setpoint);

    SmartDashboard.putNumber("TY", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("degree of shooter", encoder.getPosition());
    SmartDashboard.putNumber("stats calc angle", returnCalcAngle());

  }
}
