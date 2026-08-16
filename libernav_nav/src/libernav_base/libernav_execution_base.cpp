#include "libernav_base/libernav_execution_base.h"

namespace libernav_nav
{
LiberNavExecutionBase::LiberNavExecutionBase(
  const std::string & name,
  const libernav_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
: should_exit_(false),
  outcome_(255),
  cancel_(false),
  name_(name),
  robot_info_(robot_info),
  node_(node) {}

LiberNavExecutionBase::~LiberNavExecutionBase()
{
  if (thread_.joinable()) {
    // if the user forgets to call stop(), we have to kill it
    stop();
    thread_.join();
  }
}

bool LiberNavExecutionBase::start()
{
  if (thread_.joinable()) {
    // if the user forgets to call stop(), we have to kill it
    stop();
    thread_.join();
  }

  should_exit_ = false;
  thread_ = std::thread(&LiberNavExecutionBase::run, this);
  return true;
}

void LiberNavExecutionBase::stop()
{
  RCLCPP_WARN_STREAM(
    node_->get_logger(),
    "Try to stop the plugin \""
      << name_ << "\" rigorously by notifying the thread!");

  {
    // Set the exit flag in a critical section
    std::unique_lock<std::mutex> lock(should_exit_mutex_);
    should_exit_ = true;
  }
  // Wake any thread waiting in waitForStateUpdate() so it notices should_exit_
  condition_.notify_all();
}

void LiberNavExecutionBase::join()
{
  if (thread_.joinable()) {thread_.join();}
}

std::cv_status LiberNavExecutionBase::waitForStateUpdate(
  std::chrono::microseconds const & duration)
{
  std::unique_lock<std::mutex> lock(state_wait_mutex_);
  return condition_.wait_for(lock, duration);
}

uint32_t LiberNavExecutionBase::getOutcome() const {return outcome_.load();}

std::string LiberNavExecutionBase::getMessage() const
{
  std::lock_guard<std::mutex> lock(message_mtx_);
  return message_;
}

const std::string & LiberNavExecutionBase::getName() const {return name_;}

void LiberNavExecutionBase::setOutcome(uint32_t outcome)
{
  outcome_.store(outcome);
}

void LiberNavExecutionBase::setMessage(const std::string & message)
{
  std::lock_guard<std::mutex> lock(message_mtx_);
  message_ = message;
}

void LiberNavExecutionBase::setOutcomeAndMessage(
  uint32_t outcome, const std::string & message)
{
  outcome_.store(outcome);
  setMessage(message);
}

}  // namespace libernav_nav
