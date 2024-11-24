package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.I2C;

public class DistanceSensor extends SubsystemBase {

  // Declare the REV 2M distance sensor
  private Rev2mDistanceSensor distanceSensor;

  public DistanceSensor() {
    // Initialize the sensor on a specified port (e.g., I2C)
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    // Configure the sensor if necessary
    distanceSensor.setAutomaticMode(true); // Automatically updates distance
    distanceSensor.setDistanceUnits(Unit.kMillimeters); // Set to millimeters
    distanceSensor.setRangeProfile(RangeProfile.kDefault); // Set range profile
  }

  // Method to get the current distance reading in millimeters
  public double getRealRange() {
    return distanceSensor.getRange(); // Use getRange() instead of GetRange()
  }

  // Optional: Method to check if the sensor is operating normally
  public boolean isSensorValid() {
    return distanceSensor.isRangeValid();
  }

  @Override
  public void periodic() {
    // Called once per scheduler run, you can use it for debuggin
  }
}
