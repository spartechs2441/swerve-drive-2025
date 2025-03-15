package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

public class ControllerCheckCmd extends Command {

    private final static List<Pair<Joystick, String>> controllerMap = List.of(
            new Pair<>(new Joystick(0), "Controller (Xbox One For Windows)"),
            new Pair<>(new Joystick(1), "Extreme 3D pro")
    );

    private long lastCheckTime = 0;
    @Override
    public void execute() {
        if (lastCheckTime + 3000 > System.currentTimeMillis()) return;

        lastCheckTime = System.currentTimeMillis();
        for (Pair<Joystick, String> pair : controllerMap) {
            Joystick joystick = pair.getFirst();
            String expectedName = pair.getSecond();
            if (!joystick.isConnected()) return;
            if (!joystick.getName().equals(expectedName)) return;

            DriverStation.reportWarning("Driver controller isn't expected type. Expected: '"
                    + expectedName + "' Got: '" + joystick.getName(), false);
        }
    }
}
