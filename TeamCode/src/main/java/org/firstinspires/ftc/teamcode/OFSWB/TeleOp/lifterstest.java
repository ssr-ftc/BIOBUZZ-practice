package org.firstinspires.ftc.teamcode.OFSWB.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OFSWB.Subsystems.lifters;

    @TeleOp(name = "lifters test vihaan", group = "tests")
    public class lifterstest extends OpMode {
        private lifters lift;

        @Override
        public void init() {
            lift = new lifters(hardwareMap);
        }

        @Override
        public void start() {
            lift.allDown();
        }

        @Override
        public void loop() {
            if (gamepad2.squareWasPressed()) {
                if (lift.leftisup()) {
                    lift.leftDown();
                } else {
                    lift.leftUp();
                }
            }

            if (gamepad2.triangleWasPressed()) {
                if (lift.backisup()) {
                    lift.backDown();
                } else {
                    lift.backUp();
                }
            }
            if (gamepad2.circleWasPressed()) {
                if (lift.rightisup()) {
                    lift.rightDown();
                } else {
                    lift.rightUp();
                }
            }


                telemetry.addData("left position", lift.liftLeft.getPosition());
                telemetry.addData("back position", lift.liftBack.getPosition());
                telemetry.addData("right position", lift.liftRight.getPosition());
                telemetry.update();
            }
        }