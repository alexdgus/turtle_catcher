#include <random>

#include "rclcpp/rclcpp.hpp"
#include "turtle_catcher/msg/turtle.hpp"
#include "turtle_catcher/msg/turtle_list.hpp"
#include "turtle_catcher/srv/catch.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

class Spawner : public rclcpp::Node {
   public:
    Spawner() : rclcpp::Node("turtle_spawner") {
        this->declare_parameter("spawn_frequency", 2.0);
        auto spawnFrequency =
            this->get_parameter("spawn_frequency").as_double();
        m_SpawnFrequency_ms =
            std::chrono::milliseconds(static_cast<int>(spawnFrequency * 1000));

        m_SpawnThread = std::thread([this]() { spawnFunc(); });

        m_KillClient = create_client<turtlesim::srv::Kill>("kill");
        while (!m_KillClient->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),
                        "Waiting for turtlesim kill service");
        }

        m_CatchService = this->create_service<turtle_catcher::srv::Catch>(
            "catch", std::bind(&Spawner::catchTurtle, this,
                               std::placeholders::_1, std::placeholders::_2));

        m_TurtleListPublisher =
            this->create_publisher<turtle_catcher::msg::TurtleList>(
                "turtle_list", 10);

        RCLCPP_INFO(this->get_logger(), "Spawner Initialized");
    }

   private:
    void spawnFunc() {
        // Initialize the client for the spawn service
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(),
                        "Waiting for turtlesim spawn service");
        }

        // Initialize random number generation
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(0.0, 11.0);
        std::uniform_real_distribution<double> angle_dist(0.0, 2 * 3.14159);

        // Periodically spawn turtles at a random location
        while (true) {
            RCLCPP_INFO(get_logger(), "start spawn loop");
            turtle_catcher::msg::Turtle t;
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = dist(mt);
            request->y = dist(mt);
            request->theta = angle_dist(mt);
            auto fpp = client->async_send_request(request);
            try {
                auto response = fpp.get();
                t.name = response->name;
                t.x = request->x;
                t.y = request->y;
                t.theta = request->theta;
                {
                    std::unique_lock tLock(m_TurtleMutex);
                    m_Turtles.turtles.push_back(t);
                    turtle_catcher::msg::TurtleList turtleListMsg = m_Turtles;
                    m_TurtleListPublisher->publish(turtleListMsg);
                }
                RCLCPP_INFO(
                    this->get_logger(),
                    "Turtle spawned with name %s, location, x: %f, y: %f",
                    t.name.c_str(), t.x, t.y);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle");
            }
            std::this_thread::sleep_for(m_SpawnFrequency_ms);
        }
    }

    void catchTurtle(
        const turtle_catcher::srv::Catch::Request::SharedPtr catchRequest,
        [[maybe_unused]] const turtle_catcher::srv::Catch::Response::SharedPtr
            catchResponse) {
        RCLCPP_INFO(get_logger(), "Start Kill");
        auto killRequest = std::make_shared<turtlesim::srv::Kill::Request>();
        killRequest->name = catchRequest->name;
        try {
            auto killRequestThread =
                std::thread([this, killRequest]() { killFunc(killRequest); });
            killRequestThread.detach();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to catch turtle");
        }
        RCLCPP_INFO(get_logger(), "Kill Done");
    }

    void killFunc(std::shared_ptr<turtlesim::srv::Kill::Request> killRequest) {
        auto fpp = m_KillClient->async_send_request(killRequest);

        try {
            auto response = fpp.get();
            auto name = killRequest->name;
            std::unique_lock tLock(m_TurtleMutex);
            auto turtleIt =
                find_if(m_Turtles.turtles.begin(), m_Turtles.turtles.end(),
                        [name](turtle_catcher::msg::Turtle t) {
                            return t.name == name;
                        });
            if (turtleIt != m_Turtles.turtles.end()) {
                m_Turtles.turtles.erase(turtleIt);
            }

            // Broadcast updated turtle list
            turtle_catcher::msg::TurtleList turtleListMsg = m_Turtles;
            m_TurtleListPublisher->publish(turtleListMsg);
            RCLCPP_INFO(get_logger(), "Kill request successful");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to kill turtle");
        }
    }

    std::thread m_SpawnThread;
    std::chrono::milliseconds m_SpawnFrequency_ms;
    rclcpp::Service<turtle_catcher::srv::Catch>::SharedPtr m_CatchService;
    std::shared_ptr<rclcpp::Client<turtlesim::srv::Kill>> m_KillClient;
    rclcpp::Publisher<turtle_catcher::msg::TurtleList>::SharedPtr
        m_TurtleListPublisher;
    std::mutex m_TurtleMutex;
    turtle_catcher::msg::TurtleList m_Turtles;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Spawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
