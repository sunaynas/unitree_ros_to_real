
#include <pthread.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>

#include "convert.h"
#include "unitree_legged_msgs/HighCmdService.h"

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

template <typename TLCM>
void *update_loop(void *param) {
  TLCM *data = (TLCM *)param;
  while (ros::ok) {
    data->Recv();
    std::cout << "Sending robo data" << std::endl;
    usleep(2000);
  }
}

template <typename TCmd, typename TState, typename TLCM>
class Controller {
 public:
  Controller(TLCM &roslcm) : m_multiSpinner{2} {
    // m_multiExec.set_number_of_threads(2);

    m_node = ros::NodeHandle("controller_node");
    m_node.setCallbackQueue(&m_queue);

    m_service = m_node.advertiseService("control_a1",
                                        &Controller::controllerCallback, this);
    roslcm.SubscribeState();

    m_reqHighROS.forwardSpeed = 0.0f;
    m_reqHighROS.sideSpeed = 0.0f;
    m_reqHighROS.rotateSpeed = 0.0f;
    m_reqHighROS.bodyHeight = 0.0f;
    m_reqHighROS.mode = 0;
    m_reqHighROS.roll = 0;
    m_reqHighROS.pitch = 0;
    m_reqHighROS.yaw = 0;

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    // m_thread = std::thread(&Controller::runLoop, this);

    // m_multiExec.add_callback(std::bind(&Controller::runLoop, this));
    // m_multiExec.add_callback(std::bind(&Controller::updateLoop, &roslcm));

    runLoop(roslcm);

    // m_multiExec.spin();
  }

  //   ~Controller() {
  //     if (m_thread.joinable()) m_thread.join();
  //   }

 private:
  bool controllerCallback(unitree_legged_msgs::HighCmdService::Request &req,
                          unitree_legged_msgs::HighCmdService::Response &res) {
    m_reqHighROS.forwardSpeed = req.forwardSpeed;
    m_reqHighROS.sideSpeed = req.sideSpeed;
    m_reqHighROS.rotateSpeed = req.rotateSpeed;
    std::cout << "REQUEST\nmode:" << req.mode
              << "\nfwdspd: " << req.forwardSpeed
              << "\nsidespd: " << req.sideSpeed
              << "\nrotspd: " << req.rotateSpeed << std::endl;
    res.success = true;
    res.message = "";
    return res.success;
  }

  void runLoop(TLCM &roslcm) {
    ros::Rate loop_rate(500);
    // SetLevel(HIGHLEVEL);
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};

    while (ros::ok()) {
      roslcm.Get(RecvHighLCM);
      m_recvHighROS = ToRos(RecvHighLCM);

      SendHighLCM = ToLcm(m_reqHighROS, SendHighLCM);
      roslcm.Send(SendHighLCM);
      std::cout << "Sending lcm" << std::endl;

      //   ros::spinOnce();
      m_multiSpinner.spin(&m_queue);
      loop_rate.sleep();
    }
  }

  ros::MultiThreadedSpinner m_multiSpinner;
  ros::CallbackQueue m_queue;
  ros::ServiceServer m_service;
  ros::NodeHandle m_node;
  unitree_legged_msgs::HighCmd m_reqHighROS;
  unitree_legged_msgs::HighState m_recvHighROS;
  //   std::thread m_thread;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "walk_ros_mode");
  std::string firmwork;
  ros::param::get("/firmwork", firmwork);

#ifdef SDK3_1
  aliengo::Control control(aliengo::HIGHLEVEL);
  aliengo::LCM roslcm;
  Controller<aliengo::HighCmd, aliengo::HighState, aliengo::LCM> c(roslcm);
#endif

#ifdef SDK3_2
  std::string robot_name;
  UNITREE_LEGGED_SDK::LeggedType rname;
  ros::param::get("/robot_name", robot_name);
  if (strcasecmp(robot_name.c_str(), "A1") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::A1;
  else if (strcasecmp(robot_name.c_str(), "Aliengo") == 0)
    rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

  // UNITREE_LEGGED_SDK::InitEnvironment();
  UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
  Controller<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState,
             UNITREE_LEGGED_SDK::LCM>
      c(roslcm);
#endif
}