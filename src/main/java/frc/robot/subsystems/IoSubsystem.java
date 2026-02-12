package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IoConstants.*;

/**
 * Handles intaking and outtaking fuel.
 * 
 * @author znotfireman
 */
public class IoSubsystem extends SubsystemBase {
    public final SparkMax ioMotor;
    public final SparkMax loaderMotor;

    public IoSubsystem() {
        ioMotor = new SparkMax(IO_MOTOR_ID, MotorType.kBrushed);
        SparkMaxConfig ioConfig = new SparkMaxConfig();
        ioConfig.smartCurrentLimit(IO_MOTOR_CURRENT_LIMIT);
        ioMotor.configure(ioConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        loaderMotor = new SparkMax(LOADER_MOTOR_ID, MotorType.kBrushed);
        SparkMaxConfig loaderConfig = new SparkMaxConfig();
        loaderConfig.smartCurrentLimit(LOADER_MOTOR_CURRENT_LIMIT);
        loaderMotor.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeeds(double ioSpeed, double loaderSpeed) {
        ioMotor.set(ioSpeed);
        loaderMotor.set(loaderSpeed);
    }

    public void stop() {
        setSpeeds(0, 0);
    }

    public Command commandStop() {
        return runOnce(() -> stop());
    }

    public Command commandSpeeds(double ioSpeed, double loaderSpeed) {
        return startEnd(() -> setSpeeds(ioSpeed, loaderSpeed), () -> stop());
    }

    public Command commandIntake() {
        return commandSpeeds(INTAKING_IO_VOLTAGE, INTAKING_LOADER_VOLTAGE);
    }

    public Command comamndPrepare() {
        return commandSpeeds(PREPARING_IO_VOLTAGE, PREPARING_LOADER_VOLTAGE);
    }

    public Command commandLaunch() {
        return commandSpeeds(LAUNCHING_IO_VOLTAGE, LAUNCHING_LOADER_VOLTAGE);
    }

    // An IQ Too High?
    public Command commandEject() {
        return commandSpeeds(-INTAKING_IO_VOLTAGE, -INTAKING_LOADER_VOLTAGE);
    }
}
