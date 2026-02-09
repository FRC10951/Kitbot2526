## Kitbot 2026 Robot Code

This project contains the WPILib command-based robot code for the 2026 Kitbot.

### Robot Controls

**Driver (USB port 0, Xbox controller)**
- **Left stick Y**: Drive forward/backward (inverted so pushing forward drives forward)
- **Right stick X**: Rotate

**Operator (USB port 1, Xbox controller)**
- **Left bumper (LB)**: Intake fuel (runs intake + feeder motors)
- **X button**: Intake fuel (same behavior as left bumper)
- **Right bumper (RB)**: Spin up, then launch fuel (shoot)
- **Y button**: Spin up, then launch fuel (same behavior as right bumper)
- **A button**: Eject fuel back out the intake

### Fuel Mechanism CAN IDs

- **Feeder motor**: CAN ID 9
- **Intake/launcher motor**: CAN ID 19

These motors are managed by the `CANFuelSubsystem` and used by the `Intake`, `SpinUp`, `Launch`, and `LaunchSequence` commands.

