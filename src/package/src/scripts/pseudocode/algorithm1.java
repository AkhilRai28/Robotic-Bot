import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import geometry_msgs.Twist;
import org.ros.message.MessageListener;
import java.lang.Math;

public class Obstacle extends AbstractNodeMain {

    private Double leftDistance = null;
    private Double middleDistance = null;
    private Double rightDistance = null;
    private Publisher<geometry_msgs.Twist> velPub;

    @Override
    public String getDefaultNodeName() {
        return "obstacle";
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        velPub = connectedNode.newPublisher("mybot", Twist._TYPE);

        Subscriber<Twist> subscriber = connectedNode.newSubscriber("ultrasound", Twist._TYPE);
        subscriber.addMessageListener(new MessageListener<Twist>() {
            @Override
            public void onNewMessage(Twist message) {
                ultrasoundCallback(message);
            }
        });

        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                try {
                    while (true) {
                        if (leftDistance != null && middleDistance != null && rightDistance != null) {
                            avoidObstacle();
                        }
                        Thread.sleep(100);  // 10Hz
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        };

        Thread thread = new Thread(runnable);
        thread.start();
    }

    private void ultrasoundCallback(Twist data) {
        leftDistance = data.getLinear().getX();
        middleDistance = data.getLinear().getY();
        rightDistance = data.getLinear().getZ();
    }

    private void avoidObstacle() {
        Twist twistCmd = velPub.newMessage();
        if (middleDistance > 10) {
            double distance = Math.sqrt(Math.pow(rightDistance - leftDistance, 2) + Math.pow(middleDistance, 2));
            double angle1 = Math.acos((rightDistance - leftDistance) / distance);
            double angle = Math.toDegrees(angle1);
            twistCmd.getLinear().setX((180 - angle) * distance / 100 * 3 * 0.63 * 1.35);
            twistCmd.getLinear().setY(angle * distance / 100 * 3 * 0.63 * 1.35);
            twistCmd.getLinear().setZ(1);
            System.out.println(angle);
            System.out.println(distance);
        } else if (middleDistance < 10) {
            twistCmd.getLinear().setZ(0);
            twistCmd.getLinear().setY(255);
            twistCmd.getLinear().setX(115);
        }
        velPub.publish(twistCmd);
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
        Obstacle obstacleNode = new Obstacle();
        org.ros.node.NodeConfiguration nodeConfiguration = org.ros.node.NodeConfiguration.newPublic("localhost", rosCore.getUri());
        nodeMainExecutor.execute(obstacleNode, nodeConfiguration);
    }
}
