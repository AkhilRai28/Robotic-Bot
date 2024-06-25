import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageListener;
import std_msgs.Float32;

public class UltraInputNode extends AbstractNodeMain {

    private Publisher<std_msgs.Float32> pub;

    @Override
    public String getDefaultNodeName() {
        return "ultra_input";
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        pub = connectedNode.newPublisher("mod_msg", Float32._TYPE);

        Subscriber<Float32> subscriber = connectedNode.newSubscriber("ultra_msg", Float32._TYPE);
        subscriber.addMessageListener(new MessageListener<Float32>() {
            @Override
            public void onNewMessage(Float32 message) {
                modify(message);
            }
        });
    }

    private void modify(Float32 data) {
        double calData = data.getData();
        System.out.println("I heard " + calData);
        calData *= Math.pow(10, 3);
        System.out.println("I send " + calData);

        std_msgs.Float32 msg = pub.newMessage();
        msg.setData((float) calData);
        pub.publish(msg);
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
        UltraInputNode ultraInputNode = new UltraInputNode();
        org.ros.node.NodeConfiguration nodeConfiguration = org.ros.node.NodeConfiguration.newPublic("localhost", rosCore.getUri());
        nodeMainExecutor.execute(ultraInputNode, nodeConfiguration);
    }
}
