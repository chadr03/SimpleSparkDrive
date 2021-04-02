// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
  double governer = 1.0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private CANPIDController leftPidController;  
  private CANPIDController rightPidController;


  /** Creates a new DriveTrainSubsystem. */
  CANSparkMax leftLeader = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rightLeader = new CANSparkMax(1, MotorType.kBrushless);

 

  CANEncoder leftEncoder;
  CANEncoder rightEncoder;


  TalonSRX emptyTalon = new TalonSRX(4);
  PigeonIMU gyro = new PigeonIMU(emptyTalon);




  DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

 
 



  

  public DriveTrainSubsystem() {
    leftLeader.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();


    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    leftPidController = leftLeader.getPIDController();
    rightPidController = rightLeader.getPIDController();

    // PID coefficients
    kP = 0.0001; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.00018; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    
        // Smart Motion Coefficients
    maxVel = 5700; // rpm
    maxAcc = 1500;
    
        // set PID coefficients
    leftPidController.setP(kP);
    leftPidController.setI(kI);
    leftPidController.setD(kD);
    leftPidController.setIZone(kIz);
    leftPidController.setFF(kFF);
    leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    rightPidController.setP(kP);
    rightPidController.setI(kI);
    rightPidController.setD(kD);
    rightPidController.setIZone(kIz);
    rightPidController.setFF(kFF);
    rightPidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    leftPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    leftPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    leftPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    leftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    rightPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    rightPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    rightPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    rightPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);



    resetGyro();
    resetEncoders();



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Position", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Position", getRightEncoderPosition());
    SmartDashboard.putNumber("Left Encoder Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Gyro Angle", getHeading());
  }


  public void teleopDrive(double move, double turn){
    drive.arcadeDrive(governer*move, turn);
  }

  public void closedLoopDrive(double move, double turn){
    //double maxForwardPercent = 1.0;
    double maxForwardPercent = governer;
    double maxTurnPercent = 1.0;
    double deadband = 0.1;

    SmartDashboard.putNumber("Move", move);
    SmartDashboard.putNumber("Turn", turn);


    if (Math.abs(move)<deadband){
      move = 0.0;
    }

    if (Math.abs(turn)<deadband){
      turn = 0.0;
    }    

    double leftVelocity;
    double rightVelocity;



    move = move*maxForwardPercent;
    turn = turn*maxTurnPercent;

    SmartDashboard.putNumber("Move1", move);
    SmartDashboard.putNumber("Turn1", turn);

    double kNorm = Math.abs(move) + Math.abs(turn);

    SmartDashboard.putNumber("KNorm", kNorm);

    leftVelocity = maxVel * (move + turn);//kNorm;
    rightVelocity = maxVel * (-move + turn);//kNorm;

    SmartDashboard.putNumber("Left V SetP", leftVelocity);
    SmartDashboard.putNumber("Right V SetP", rightVelocity);


    leftPidController.setReference(leftVelocity, ControlType.kVelocity);
    rightPidController.setReference(rightVelocity, ControlType.kVelocity);


  }

  public void setGovener(double value){
    governer=value;
  }

  //Encoder methods
  public double getLeftEncoderPosition(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoderPosition(){
    return -rightEncoder.getPosition();
  }

  public double getLeftEncoderVelocity(){
    return leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity(){
    return -rightEncoder.getVelocity();
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }


  //Gyro methods
  public double getHeading(){
    double[] ypr_deg = new double[3];
    gyro.getYawPitchRoll(ypr_deg);
    return ypr_deg[0];
  }

  public void resetGyro(){
    gyro.setYaw(0.0);
  }
}
