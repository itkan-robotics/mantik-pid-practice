// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorExpoSubsystem extends SubsystemBase {
  MechanismGearing gearing = new MechanismGearing(new GearBox(new double[]{5}));
  DCMotor motors = DCMotor.getKrakenX60(1);
  Mass weight = Pounds.of(6);
  Distance radius = Millimeters.of(168).div(2 * Math.PI);

  private SmartMotorControllerConfig smcConfigLeft =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Mechanism Circumference is the distance traveled by each mechanism rotation converting
          // rotations to meters.
          .withMechanismCircumference(Millimeters.of(168))
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              new ExponentialProfilePIDController(
                  0,
                  0,
                  0.0,
                  ExponentialProfilePIDController.createElevatorConstraints(
                      Volts.of(12), // Maximum voltage during profile
                      motors, // Motors
                      weight, // Carraige weight
                      radius, // Drum radius
                      gearing))) // Gearing)
          .withSimClosedLoopController(
              new ExponentialProfilePIDController(
                0,
                0,
                0.0,
                  ExponentialProfilePIDController.createElevatorConstraints(
                      Volts.of(12), // Maximum voltage during profile
                      motors, // Motors
                      weight, // Carraige weight
                      radius, // Drum radius
                      gearing))) // Gearing)
          // Feedforward Constants
          .withFeedforward(new ElevatorFeedforward(0, 0.0, 0.0, 0.0))
          .withSimFeedforward(new ElevatorFeedforward(0, 0.0, 0.0, 0.0))
          // Telemetry name and verbosity level
          .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to
          // the gearbox attached to your motor.
          .withGearing(gearing)
          // Motor properties to prevent over currenting.
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.0))
          .withOpenLoopRampRate(Seconds.of(0.0));

  // Vendor motor controller object
  private TalonFX leftMotor =
      new TalonFX(13);

  // Create our SmartMotorController from our Spark and config with the NEO.

  private SmartMotorController elevatorMotorsController =
      new TalonFXWrapper(leftMotor, DCMotor.getKrakenX60(1), smcConfigLeft);

  private ElevatorConfig elevconfig =
      new ElevatorConfig(elevatorMotorsController)
          .withStartingHeight(Meters.of(0.0))
          .withHardLimits(Inches.of(0), Inches.of(84))
          .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
          .withMass(Pounds.of(6));

  // Elevator Mechanism
  private Elevator elevator = new Elevator(elevconfig);

  /**
   * Set the height of the elevator.
   *
   * @param angle Distance to go to.
   */
  public Command setHeight(Distance height) {
    return elevator.setHeight(height);
  }

  /**
   * Move the elevator up and down.
   *
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    return elevator.set(dutycycle);
  }

  /** Run sysId on the {@link Elevator} */
  public Command sysId() {
    return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorExpoSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}