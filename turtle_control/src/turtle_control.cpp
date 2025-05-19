#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

// グローバル変数を宣言
rclcpp::Node::SharedPtr node = nullptr;
turtlesim::msg::Pose pose_common;

void call_reset(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client){
    // リクエストの設定
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    // サービスが利用可能になるまで待機
    // if(!client->wait_for_service(std::chrono::seconds(1))){
    //     RCLCPP_ERROR(node->get_logger(), "Service /reset is not available");
    //     return;
    // }

    // サービスを呼び出し
    auto future = client->async_send_request(request);

    // サービスの応答を待機
    // if(rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS){
    //     RCLCPP_INFO(node->get_logger(), "Service /reset called successfully");
    // }
    // else {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to call service /reset");
    // }
}

void call_get_param(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client){
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

    auto future = client->async_send_request(request);
}

//　subscribe時に実行されるコールバック関数
void callback_sub_pose(const turtlesim::msg::Pose::SharedPtr msg){
    // 受信したメッセージをグローバル変数に格納
    pose_common.x = msg->x;
    pose_common.y = msg->y;
    pose_common.theta = msg->theta;
    
    RCLCPP_INFO(node->get_logger(), "Subscribe Pose: x=%f, y=%f, theta=%f", pose_common.x, pose_common.y, pose_common.theta);
}

int main(int argc, char **argv){
    // ROS2を初期化
    rclcpp::init(argc, argv);

    // ノードを作成
    node = rclcpp::Node::make_shared("turtle_control");

    node->declare_parameter<double>("Kp", 0.1);
    node->declare_parameter<double>("ref_x", 3.0);
    node->declare_parameter<double>("ref_y", 3.0);

    auto Kp = node->get_parameter("Kp").as_double();
    auto ref_x = node->get_parameter("ref_x").as_double();
    auto ref_y = node->get_parameter("ref_y").as_double();

    // publisherとsubscriber,クライアントを登録
    auto pub_twist = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    auto sub_pose = node->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, callback_sub_pose);
    auto cli_reset = node->create_client<std_srvs::srv::Empty>("/reset");
    auto cli_get_param = node->create_client<rcl_interfaces::srv::GetParameters>("/turtle_control/get_parameters");
    
    // publish用のメッセージを作成
    auto cmd = geometry_msgs::msg::Twist();

    // loop rateを設定
    rclcpp::WallRate loop(0.5); // 2Hz

    while(rclcpp::ok()){
        // Callback関数を1回だけ実行
        rclcpp::spin_some(node);

        // メッセージをpublish
        pub_twist->publish(cmd);

        // 目標位置に向けて速度を計算
        cmd.linear.x = Kp*( ref_x - pose_common.x );
        cmd.linear.y = Kp*( ref_y - pose_common.y );

        // Display the message in the console
        // RCLCPP_INFO(node->get_logger(), "Publish: vel_x=%f, vel_y=%f", cmd.linear.x, cmd.linear.y);

        // 目標位置に近づいたらリセット
        if(abs(ref_x - pose_common.x) < 0.1 && abs(ref_y - pose_common.y) < 0.1){
            call_reset(cli_reset);
        }

        call_get_param(cli_get_param);

        // loop rateまで待機
        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}