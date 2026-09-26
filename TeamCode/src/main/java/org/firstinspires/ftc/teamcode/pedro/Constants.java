package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("flwheel");
        c.frontRightName.set("frwheel");
        c.backLeftName.set("blwheel");
        c.backRightName.set("brwheel");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.915994298739696);
        c.yPodOffset.set(5.858882994163694);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.22495088606937844);
                Controller secondaryTranslationalForward = Controller.proportional(0.08311334160425708);
                Controller primaryTranslationalLateral = Controller.proportional(0.3995184729935072);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14761140044089213);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018480570356829106));
                c.brake.set(Controller.proportionalFeedforward(0.01570848480330474));

                c.headingFeedback.set(Controller.proportional(3.333503782408267));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0553612447591359, 0.005237320537581774));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09103691394540052, 0.04654945441249328));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.682979151132949E-4, 0.0020756364378511825));

                c.maxAchievableForwardVelocity.set(57.00309555238967);
                c.maxAchievableStrafeVelocity.set(46.422243296433614);
                c.naturalForwardDeceleration.set(48.92682939746124);
                c.naturalStrafeDeceleration.set(68.63473892838032);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
