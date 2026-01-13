package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CAN_BUS_NAME;
import static frc.robot.Constants.Hopper.*;

@LoggedObject
public class Hopper extends SubsystemBase implements BaseIntake {
    @Log
    private final TalonFX motor;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    public Hopper() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        if (RobotBase.isReal()) {
            motorConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
            motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        motor = new TalonFX(MOTOR_ID, CAN_BUS_NAME);
        TalonFXConfigurator leftMotorConfigurator = motor.getConfigurator();
        leftMotorConfigurator.apply(motorConfig);
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> motor.setControl(voltageRequest.withOutput(4)),
                () -> motor.setControl(voltageRequest.withOutput(0)))
                .withName("hopper.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> motor.setControl(voltageRequest.withOutput(-4)),
                () -> motor.setControl(voltageRequest.withOutput(0)))
                .withName("hopper.runRollers");
    }

    public Command setRollerVoltageCommand(Supplier<Double> voltage) {
        return Commands.runEnd(
                () -> motor.setControl(voltageRequest.withOutput(voltage.get())),
                () -> motor.setControl(voltageRequest.withOutput(0)))
                .withName("hopper.runRollers");
    }

}