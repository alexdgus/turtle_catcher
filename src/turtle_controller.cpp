#include <math.h>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtle_catcher/msg/turtle.hpp"
#include "turtle_catcher/msg/turtle_list.hpp"
#include "turtle_catcher/srv/catch.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleController : public rclcpp::Node {
   public:
    TurtleController() : Node("turtle_controller") {
        this->declare_parameter("catch_closest_turtle_first", true);
        m_CatchClosestTurtleFirst =
            this->get_parameter("catch_closest_turtle_first").as_bool();

        m_Publisher =
            create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        m_PoseSubscription = create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleController::poseCallback, this,
                      std::placeholders::_1));

        m_TurtleListSubscription =
            create_subscription<turtle_catcher::msg::TurtleList>(
                "turtle_list", 10,
                std::bind(&TurtleController::turtleListCallback, this,
                          std::placeholders::_1));

        m_CatchClient = create_client<turtle_catcher::srv::Catch>("catch");
        while (!m_CatchClient->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),
                        "Waiting for turtle_catcher catch service");
        }

        m_Timer =
            create_wall_timer(std::chrono::milliseconds(10),
                              std::bind(&TurtleController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "TurtleController initialized");
    }

   private:
    void poseCallback(const turtlesim::msg::Pose& pose) {
        m_CurrentPose = pose;
    }

    void turtleListCallback(const turtle_catcher::msg::TurtleList& turtleList) {
        RCLCPP_INFO(get_logger(), "TurtleList message received");
        m_TurtleList = turtleList;
        if (m_TurtleList.turtles.size()) {
            if (m_CatchClosestTurtleFirst) {
                turtle_catcher::msg::Turtle closestTurtle;
                double closestDistance = std::numeric_limits<double>::max();
                for (const auto& turtle : m_TurtleList.turtles) {
                    double distance = calcDistance(turtle);
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestTurtle = turtle;
                    }
                }
                m_Target = closestTurtle;
            } else {
                m_Target = m_TurtleList.turtles[0];
            }
            m_ValidTarget = true;
        }
    }

    double calcDistance(const turtle_catcher::msg::Turtle& target) {
        // Calculate distance between current location and the target
        float xOffset = target.x - m_CurrentPose.x;
        float yOffset = target.y - m_CurrentPose.y;
        float distance = pow((pow(xOffset, 2) + pow(yOffset, 2)), 0.5);
        return distance;
    }

    double calcAngle(const turtle_catcher::msg::Turtle& target) {
        // Calculate angle between current and target
        float xOffset = target.x - m_CurrentPose.x;
        float yOffset = target.y - m_CurrentPose.y;
        float angleToTarget = atan2(yOffset, xOffset);
        float angleOffset = angleToTarget - m_CurrentPose.theta;
        if (angleOffset > 3.14159) {
            angleOffset -= 2 * 3.14159;
        } else if (angleOffset < -3.14159) {
            angleOffset += 2 * 3.14159;
        }
        return angleOffset;
    }

    void controlLoop() {
        if (m_ValidTarget) {
            auto msg = geometry_msgs::msg::Twist();

            msg.linear.x = calcDistance(m_Target);

            double angleOffset = calcAngle(m_Target);
            msg.angular.z = angleOffset * 3.0;

            m_Publisher->publish(msg);

            if (msg.linear.x < 0.5) {
                m_ValidTarget = false;
                auto request =
                    std::make_shared<turtle_catcher::srv::Catch::Request>();
                request->name = m_Target.name;

                RCLCPP_INFO(this->get_logger(),
                            "Send turtle caught request: %s",
                            request->name.c_str());
                try {
                    auto catchRequestThread = std::thread(
                        [this, request]() { catchRequest(request); });
                    catchRequestThread.detach();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to catch turtle");
                }
            }
        }
    }

    void catchRequest(
        std::shared_ptr<turtle_catcher::srv::Catch::Request> request) {
        auto future = m_CatchClient->async_send_request(request);
        try {
            RCLCPP_INFO(this->get_logger(), "Call get for caught response: %s",
                        request->name.c_str());
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Turtle caught: %s",
                        request->name.c_str());
        } catch (std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to send catch request");
        }
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_PoseSubscription;
    rclcpp::Subscription<turtle_catcher::msg::TurtleList>::SharedPtr
        m_TurtleListSubscription;
    std::shared_ptr<rclcpp::Client<turtle_catcher::srv::Catch>> m_CatchClient;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_Publisher;
    rclcpp::TimerBase::SharedPtr m_Timer;
    turtle_catcher::msg::TurtleList m_TurtleList;
    turtle_catcher::msg::Turtle m_Target;
    bool m_ValidTarget;
    turtlesim::msg::Pose m_CurrentPose;
    bool m_CatchClosestTurtleFirst;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
