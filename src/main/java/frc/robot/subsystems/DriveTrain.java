// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

//creates drivetrain class; it inherits atrributes from SubsystemBase class
public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain");
  private GenericEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry();
  private GenericEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry();

  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }
// it set motor power of both talons
  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

// reset to inital value of encoders (everytime code run it can be accurately measured)
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  // gets the amount of ticks traveled by encoder and turns them to meters
  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }

  public double ticksToMeter(){
    return(0.1524 * Math.PI / 4096 ) * getTicks();
  }

  // gets Angle robot is at
  public double getAngle(){
    return navx.getAngle(); 
  }
 
  // rest angle everytime code is ran
  public void resetNavx(){
    navx.reset();
  }

  // allows you to see stats while running robot
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());
    SmartDashboard.putNumber("Tickd to Meter", ticksToMeter());

    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());

  }
}