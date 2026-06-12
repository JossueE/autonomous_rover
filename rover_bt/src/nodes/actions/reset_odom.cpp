#include "rover_bt/nodes/actions/reset_odom.hpp"
#include "rover_bt/shared_context.hpp"
#include "rover_bt/tts_client.hpp"

namespace rover_bt {

ResetOdom::ResetOdom(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

BT::PortsList ResetOdom::providedPorts() {
  return {
    BT::InputPort<std::string>("service", "/rtabmap/reset_odom",
        "Empty service that resets/re-initialises RTAB-Map odometry."),
    BT::InputPort<double>("delay_sec", 5.0,
        "Seconds the odometry must be lost before the single recovery attempt "
        "fires (measured from when the loss began)."),
    BT::InputPort<double>("verdict_sec", 5.0,
        "Seconds to wait after the attempt for a relocalization before declaring "
        "failure."),
    BT::InputPort<std::string>("attempt_message",
        "Intentando recuperar la odometría.",
        "Spoken once when the reset is sent."),
    BT::InputPort<std::string>("failed_message",
        "No se pudo recuperar la odometría. La parada de emergencia se mantiene.",
        "Spoken once if the robot has not relocalized verdict_sec after the "
        "attempt.")
  };
}

void ResetOdom::speak(const std::string& text) {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx) {
    return;
  }
  if (ctx->tts) {
    ctx->tts->speak(text);
  } else if (ctx->node) {
    RCLCPP_INFO(ctx->node->get_logger(), "[ResetOdom - No TTS]: %s", text.c_str());
  }
}

BT::NodeStatus ResetOdom::tick() {
  std::shared_ptr<SharedContext> ctx;
  if (!config().blackboard->get("context", ctx) || !ctx || !ctx->node) {
    return BT::NodeStatus::SUCCESS;  // passive helper: never fail the hold seq
  }

  // The episode boundary is the odom-loss emergency itself, not the raw odom
  // lost flag: it stays true through odometry recovering until the robot has
  // relocalized and AutoResumeFromOdom resumes. This lets us keep watching for a
  // relocalization even after odometry tracking is already back.
  bool emerg_odom = false;
  (void)config().blackboard->get("emergency_from_odom", emerg_odom);
  if (!emerg_odom) {
    in_episode_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  const double now = ctx->node->get_clock()->now().seconds();

  // Rising edge of the odom-loss emergency: re-arm a fresh attempt and clear any
  // stale relocalization verdict from a previous episode.
  if (!in_episode_) {
    in_episode_ = true;
    recovery_ref_ = now;
    attempted_ = false;
    verdict_done_ = false;
    attempt_time_ = 0.0;
    config().blackboard->set("odom_relocalized", false);
  }

  // ── Success: a relocalization newer than this loss ──────────────────
  // RTAB-Map stalls while odometry is lost, so any localization_pose stamped
  // after the loss began means it processed live frames and re-matched the map —
  // the robot is genuinely localized again, not just emitting fresh odometry.
  if (ctx->last_localization_time.load() > recovery_ref_) {
    config().blackboard->set("odom_relocalized", true);
    return BT::NodeStatus::SUCCESS;  // AutoResumeFromOdom takes it from here
  }

  // ── Single recovery attempt, delay_sec after the loss began ─────────
  if (!attempted_) {
    const bool lost = ctx->rtabmap_odom_lost.load();
    const double since = ctx->rtabmap_odom_lost_since.load();
    const double lost_for = (lost && since > 0.0) ? (now - since) : 0.0;
    const double delay = getInput<double>("delay_sec").value_or(5.0);
    if (!lost || lost_for < delay) {
      return BT::NodeStatus::SUCCESS;  // not yet time (or odom already tracking)
    }

    const std::string service =
      getInput<std::string>("service").value_or("/rtabmap/reset_odom");
    if (!client_) {
      client_ = ctx->node->create_client<std_srvs::srv::Empty>(service);
    }

    if (!client_->service_is_ready()) {
      // RTAB-Map's service is not up (yet). Retry on a later tick rather than
      // burning the single attempt against a service that does not exist.
      RCLCPP_WARN_THROTTLE(ctx->node->get_logger(), *ctx->node->get_clock(), 5000,
          "ResetOdom: service '%s' not available; cannot attempt odom recovery.",
          service.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    // Async fire-and-forget: the single-threaded executor cannot spin to wait
    // for the response inside a tick, and we do not need the (empty) result.
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    client_->async_send_request(req);
    attempted_ = true;
    attempt_time_ = now;
    RCLCPP_INFO(ctx->node->get_logger(),
        "ResetOdom: odom lost %.1fs — called %s to re-initialise RTAB-Map "
        "odometry; awaiting relocalization.", lost_for, service.c_str());
    speak(getInput<std::string>("attempt_message").value_or(
        "Intentando recuperar la odometría."));
    return BT::NodeStatus::SUCCESS;
  }

  // ── Failure verdict ─────────────────────────────────────────────────
  // The attempt was made but no relocalization has arrived (the success check
  // above would have returned). If verdict_sec has elapsed, announce the failure
  // once and hold the e-stop. A late relocalization still resumes us, because
  // the success check keeps running every tick.
  if (!verdict_done_) {
    const double verdict = getInput<double>("verdict_sec").value_or(5.0);
    if (now - attempt_time_ >= verdict) {
      verdict_done_ = true;
      RCLCPP_WARN(ctx->node->get_logger(),
          "ResetOdom: no relocalization %.1fs after reset_odom — recovery "
          "failed, holding emergency stop.", now - attempt_time_);
      speak(getInput<std::string>("failed_message").value_or(
          "No se pudo recuperar la odometría. La parada de emergencia se mantiene."));
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rover_bt
