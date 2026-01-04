package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Simple Auto", group = "Auto")
public class SimpleAuto extends OpMode {

    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();

    private enum AutoState {
        START,
        MOVE_BACKWARD,
        SPIN_UP_SHOOTER,
        SHOOT,
        WAIT,
        STOP
    }

    private AutoState currentState = AutoState.START;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.initHardware();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        switch (currentState) {
            case START:
                robot.robotMove(0.5, 0.5, 0.5, 0.5); // Move backward
                if (timer.seconds() >= 1.2) {
                    robot.robotMove(0, 0, 0, 0); // Stop
                    timer.reset();
                    currentState = AutoState.SPIN_UP_SHOOTER;
                }
                break;

            case SPIN_UP_SHOOTER:
                robot.shooterOn(); // Start shooter first
                // Wait for shooter to reach speed before shooting
                if (timer.seconds() >= 1.5) {
                    timer.reset();
                    currentState = AutoState.SHOOT;
                }
                break;

            case SHOOT:
                robot.kickerDown();
                robot.intakeShoot();
                if (timer.seconds() >= 4.0) {
                    robot.kickerUp();
                    timer.reset();
                    currentState = AutoState.WAIT;
                }
                break;

            case WAIT:
                if (timer.seconds() >= 5.0) {
                    robot.intakeOff();
                    robot.shooterOff();
                    currentState = AutoState.STOP;
                }
                break;

            case STOP:
                // Robot stops, do nothing
                break;
        }

        telemetry.addData("State", currentState.toString());
        telemetry.addData("Time", timer.seconds());
    }

    @Override
    public void stop() {
        robot.robotMove(0, 0, 0, 0);
        robot.intakeOff();
        robot.shooterOff();
    }
}
