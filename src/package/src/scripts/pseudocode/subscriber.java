import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageListener;
import geometry_msgs.Twist;

public class ListenerNode extends AbstractNodeMain {

    private Publisher<geometry_msgs.Twist> pub;
    private double x1 = 0, x2 = 0, x3 = 0;

    @Override
    public String getDefaultNodeName() {
        return "listener";
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        pub = connectedNode.newPublisher("mod_msg", Twist._TYPE);

        Subscriber<Twist> subscriber = connectedNode.newSubscriber("ultra_msg", Twist._TYPE);
        subscriber.addMessageListener(new MessageListener<Twist>() {
            @Override
            public void onNewMessage(Twist message) {
                callback(message);
            }
        });
    }

    private void callback(Twist data) {
        x1 = data.getLinear().getX(); // front
        x2 = data.getLinear().getY(); // left
        x3 = data.getLinear().getZ(); // right

        System.out.printf("%f %f %f %f %f%n", data.getLinear().getX(), data.getLinear().getY(),
                data.getLinear().getZ(), data.getAngular().getX(), data.getAngular().getY());

        double x5, x6;
        try {
            x5 = (x2 - 30) / Math.abs(x2 - 30);
            x6 = (x3 - 30) / Math.abs(x3 - 30);
        } catch (Exception e) {
            x5 = 0;
            x6 = 0;
        }

        double s = 104;
        double xa = s;
        double xb = s;

        if (x1 <= 15 || x2 <= 15 || x3 <= 15) {
            xa = -s * 1.25;
            xb = -s * 1.25;
        } else {
            if (x1 < 30 || x2 < 60 || x3 < 60) {
                if (x2 > x3) {
                    xb = (s * Math.abs(x3 - 30) * x5) / (2 * x6);
                } else {
                    xa = (s + Math.abs(x2 - 30) * x6) / (2 * x5);
                }
            }
        }

        Twist z1 = pub.newMessage();
        z1.getLinear().setX(xa);
        z1.getLinear().setZ(xb);
        pub.publish(z1);
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
        ListenerNode listenerNode = new ListenerNode();
        org.ros.node.NodeConfiguration nodeConfiguration = org.ros.node.NodeConfiguration.newPublic("localhost", rosCore.getUri());
        nodeMainExecutor.execute(listenerNode, nodeConfiguration);
    }
}
