// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import java.util.HashMap;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;

// archaic communication protocols with modern problems require out of the box thinking
public class packetLossCheckerInator extends Command {
  private final Pigeon2 pigeon;
  private int startTime;
  private final int amountOfCycles = 10_000; // Estemated at 200s of robot time.
  private int lastAngle = 0;
  private int errorCount = 0;
  private int iterations = 0;
  private HashMap<Integer, Integer> statusMap; 
  /** Creates a new packetLossCheckerInator. */
  public packetLossCheckerInator(Pigeon2 pigeon) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pigeon = pigeon;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("### Test started, this will take a bit. ###");
    startTime = (int) System.currentTimeMillis();
    statusMap = new HashMap<Integer, Integer>();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = pigeon.getAngle();
    if ((int) Math.round(angle) != lastAngle) errorCount++;

    angle += 1;
    lastAngle = (int) angle;
    StatusCode e = pigeon.setYaw(lastAngle);
    if (statusMap.containsKey(e.value)) statusMap.put(e.value, statusMap.get(e.value) + 1);
    else statusMap.put(e.value, 1);   //statusMap.get(e.value)
    iterations++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int tElapsed = (int) System.currentTimeMillis() - startTime;
    System.out.println("### Test concluded ###");
    System.out.print("Obvious Falures: ");
    System.out.println(errorCount);
    System.out.print("Cycles");
    System.out.println(amountOfCycles);
    System.out.print("Time it took: ");
    System.out.println(tElapsed);
    System.out.println("## Status map counts ##");
    for (int key : statusMap.keySet()) {
      System.out.print(key);
      System.out.print(": ");
      System.out.println(statusMap.get(key));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return iterations >= amountOfCycles;
  }
}
