// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class TankDrive extends CommandBase {
  public DriveTrain dt;
  public Joystick joy;

  /** Creates a new TankDrive. */
  public TankDrive(DriveTrain dt, Joystick j) {
    this.dt = dt;
    this.joy = j;

    // use requirements
    addRequirements(dt);
  }

  // calle dwhen command is shcuedled at first
  @Override
  public void initialize() {
    dt.tankDrive(0.0, 0.0);
  }

  // called every time it runs while command is scheduled
  @Override
  public void execute() {
    double leftPowerRaw = joy.getRawAxis(1);

    double rightPowerRaw = joy.getRawAxis(5);

    dt.tankDrive(leftPowerRaw*-0.7, rightPowerRaw*-0.7);
  }

  // called when it is end/interrupted
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0);
  }

  // returns true when command ends
  @Override
  public boolean isFinished() {
    return false;
  }
}
