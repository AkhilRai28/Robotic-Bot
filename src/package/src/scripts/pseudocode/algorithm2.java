import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import geometry_msgs.Twist;
import org.ros.message.MessageListener;
import java.util.ArrayList;
import java.util.List;

public class UltraInput extends AbstractNodeMain {

    private static List<Double> x = new ArrayList<>(10);
    private Publisher<geometry_msgs.Twist> pub;

    static {
        for (int i = 0; i < 10; i++) {
            x.add(0.0);
        }
    }

    @Override
    public String getDefaultNodeName() {
        return "ultra_input";
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        pub = connectedNode.newPublisher("mod_msg", Twist._TYPE);

        Subscriber<Twist> subscriber = connectedNode.newSubscriber("ultra_msg", Twist._TYPE);
        subscriber.addMessageListener(new MessageListener<Twist>() {
            @Override
            public void onNewMessage(Twist message) {
                modify(message);
            }
        });
    }

    private static double pid(double dis) {
        double kp = 1.0;
        double ki = 0.0;
        double kd = 0.0;

        x.remove(0);
        x.add(dis);

        double p = dis;
        double i = x.stream().mapToDouble(Double::doubleValue).sum();
        double d = x.get(9) - x.get(8);

        double speed = (kp * p) + (ki * i) + (kd * d);
        System.out.println(speed);
        return speed;
    }

    private void modify(Twist data) {
        Twist revData = data;
        Twist calData = pub.newMessage();
        Twist sendData = pub.newMessage();

        if (revData.getLinear().getX() == 0) {
            revData.getLinear().setX(65);
        } else if (revData.getLinear().getY() == 0) {
            revData.getLinear().setY(65);
        } else if (revData.getLinear().getZ() == 0) {
            revData.getLinear().setZ(65);
        }

        double error = (-1 * revData.getLinear().getX()) + (1 * revData.getLinear().getZ());
        double errorPid = pid(error);

        calData.getLinear().setX(errorPid * revData.getLinear().getX());
        calData.getLinear().setZ(errorPid * revData.getLinear().getZ());

        if ((revData.getLinear().getY() < 20 && revData.getLinear().getY() > 10) ||
            (revData.getLinear().getZ() < 20 && revData.getLinear().getZ() > 10) ||
            (revData.getLinear().getX() < 20 && revData.getLinear().getX() > 10)) {

            sendData.getLinear().setX(calData.getLinear().getZ());
            sendData.getLinear().setZ(-calData.getLinear().getX());
        } else if (revData.getLinear().getY() < 10 || revData.getLinear().getZ() < 10 || revData.getLinear().getX() < 10) {
            sendData.getLinear().setX(-calData.getLinear().getZ() * 2);
            sendData.getLinear().setZ(-calData.getLinear().getX() * 2);
        } else {
            sendData.getLinear().setX(calData.getLinear().getZ());
            sendData.getLinear().setZ(calData.getLinear().getX());
        }
        System.out.println(sendData.getLinear().getX() + ", " + sendData.getLinear().getZ());
        pub.publish(sendData);
    }

    public static void main(String[] args) {
        org.ros.RosCore rosCore = org.ros.RosCore.newPublic(11311);
        rosCore.start();
        try {
            rosCore.awaitStart();
        } catch (Exception e) {
            e.printStackTrace();
        }
        org.ros.node.NodeMainExecutor nodeMainExecutor = org.ros.node.DefaultNodeMainExecutor.newDefault();
        UltraInput ultraInputNode = new UltraInput();
        org.ros.node.NodeConfiguration nodeConfiguration = org.ros.node.NodeConfiguration.newPublic("localhost", rosCore.getUri());
        nodeMainExecutor.execute(ultraInputNode, nodeConfiguration);
    }
}
