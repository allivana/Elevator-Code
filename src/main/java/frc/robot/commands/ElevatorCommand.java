// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.DistanceSensor;

public class ElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final Elevator m_elevator;
  private final DistanceSensor m_distanceSensor;
  private final double targetDistance = 6;
  private final double motorSpeed = 0.1;



  public ElevatorCommand(Elevator subsystem, DistanceSensor subsystem1) {
    m_elevator = subsystem;
    m_distanceSensor = subsystem1;
    addRequirements(subsystem);
    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.moveUp();
    SmartDashboard.putNumber("elevator test", m_distanceSensor.getDistance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_distanceSensor.getDistance() >= targetDistance) {
      m_elevator.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_distanceSensor.getDistance() >= targetDistance;
  }
}
