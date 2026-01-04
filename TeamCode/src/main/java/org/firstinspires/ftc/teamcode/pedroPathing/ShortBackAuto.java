package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Short Back Auto", group = "Auto")
public class ShortBackAuto extends OpMode {

    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();

    private enum AutoState {
        START,
        MOVE_BACKWARD,
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
                robot.robotMove(-0.5, -0.5, -0.5, -0.5); // Move backward
                // Adjusted time for 16-17 inches (was 1.5s for ~21 inches)
                // 16.5/21 * 1.5 = ~1.18 seconds
                if (timer.seconds() >= 1.18) {
                    robot.robotMove(0, 0, 0, 0); // Stop
                    timer.reset();
                    currentState = AutoState.SHOOT;
                }
                break;

            case SHOOT:
                robot.kickerDown();
                robot.intakeShoot();
                robot.shooterOn();
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

