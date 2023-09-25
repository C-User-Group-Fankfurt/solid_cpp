#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <functional>

struct EnvironmentModel {};
class Sensor {
 public:
  EnvironmentModel model_environment() {
    return EnvironmentModel{};
  }
};

class Trajectory {
};

class Planner {
 public:
  Trajectory plan_vehicle_behavior(const EnvironmentModel &/*environment*/) {
    return Trajectory{};
  }
};

enum class DrivingMode { normal, emergency };
class DrivingModeNotSupported : public std::logic_error {
  using std::logic_error::logic_error;
};

class Actor {
 public:
  virtual ~Actor() = default;
  virtual void control_vehicle(const Trajectory &) = 0;
};

class DrivingModeAwareActor : public Actor {
 public:
  virtual void set_driving_mode(const DrivingMode &) = 0;
};

using MetresPerSquareSecond = double;
struct Acceleration {
  MetresPerSquareSecond value{0};
};
std::ostream &operator<<(std::ostream &stream, const Acceleration &value) {
  stream << value.value << " metres per square second";
  return stream;
}

class PowerTrain final : public Actor {
 public:
  explicit PowerTrain(const Acceleration &acceleration_limit)
      : acceleration_limit(acceleration_limit) {}
  void control_vehicle(const Trajectory &) override {};
 private :
  Acceleration acceleration_limit;
};

using DecelerationLimitCallback = std::function<Acceleration()>;
class Brake final : public Actor {
 public:
  explicit Brake(DecelerationLimitCallback get_deceleration_limit) : get_current_deceleration_limit(
          std::move(get_deceleration_limit)) {};

  void control_vehicle(const Trajectory &) override {
    auto current_deceleration_limit = get_current_deceleration_limit();
    std::cout << current_deceleration_limit << std::endl;
  };

 private:
    DecelerationLimitCallback get_current_deceleration_limit;
};

using NewtonMetre = double;
struct Torque {
  NewtonMetre value{0};
};

class SteeringWheel final : public DrivingModeAwareActor {
 public:
  explicit SteeringWheel(const Torque &torque_limit)
      : torque_limit(torque_limit) {}
  void control_vehicle(const Trajectory &) override {};
  void set_driving_mode(const DrivingMode &) override {}
 private:
  Torque torque_limit;
};

class DrivingSystem {
 public:
  using Actors = std::vector<std::shared_ptr<Actor>>;
  DrivingSystem(std::shared_ptr<Sensor> sensor,
                std::shared_ptr<Planner> planner,
                Actors actors) :
      sensor(std::move(sensor)),
      planner(std::move(planner)),
      actors(std::move(actors)) {};

  void one_cycle() {
    auto environment_model = sensor->model_environment();
    auto vehicle_trajectory = planner->plan_vehicle_behavior(environment_model);

    for (auto &actor : actors)
      actor->control_vehicle(vehicle_trajectory);
  }

 private:
  std::shared_ptr<Sensor> sensor;
  std::shared_ptr<Planner> planner;
  Actors actors;
};

class DecelerationLimitHandler {
public:
    explicit DecelerationLimitHandler(const Acceleration &deceleration_limit)
            : deceleration_limit(deceleration_limit){};

    void set_driving_mode(const DrivingMode &driving_mode) {
        current_driving_mode = driving_mode;
    }

    Acceleration get_current_deceleration_limit() {
        static const Acceleration
                unlimited_deceleration{std::numeric_limits<double>::lowest()};
        if (current_driving_mode == DrivingMode::normal)
            return deceleration_limit;
        else
            return unlimited_deceleration;
    }

private:
    Acceleration deceleration_limit;
    DrivingMode current_driving_mode{DrivingMode::normal};
};

int main(int, char **) {
  auto sensor = std::make_shared<Sensor>();
  auto planner = std::make_shared<Planner>();

  auto power_train = std::make_shared<PowerTrain>(
      Acceleration{MetresPerSquareSecond(13.6)});
  auto deceleration_limiter = std::make_shared<DecelerationLimitHandler>(Acceleration{MetresPerSquareSecond{21.0}});
  auto brake = std::make_shared<Brake>([&deceleration_limiter](){return deceleration_limiter->get_current_deceleration_limit();});
  auto steering_wheel = std::make_shared<SteeringWheel>(
      Torque{NewtonMetre{3.0}});

  DrivingSystem driving_system(sensor, planner,
                               {power_train, brake, steering_wheel});

  auto driving_mode = DrivingMode::normal;
  deceleration_limiter->set_driving_mode(driving_mode);
  steering_wheel->set_driving_mode(driving_mode);

  driving_system.one_cycle();

  driving_mode = DrivingMode::emergency;
  deceleration_limiter->set_driving_mode(driving_mode);

  driving_system.one_cycle();
}
