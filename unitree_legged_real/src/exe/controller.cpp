
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
    // std::cout << "Sending robo data" << std::endl;
    usleep(2000);
  }
}

template <typename TCmd, typename TState, typename TLCM>
class Controller {
 public:
  Controller(TLCM &roslcm) : m_spinner{2} {
    m_node = ros::NodeHandle("controller_node");

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
    m_spinner.start();
    runLoop(roslcm);
    m_spinner.stop();
  }

 private:
  bool controllerCallback(unitree_legged_msgs::HighCmdService::Request &req,
                          unitree_legged_msgs::HighCmdService::Response &res) {
    m_reqHighROS.forwardSpeed = req.forwardSpeed;
    m_reqHighROS.sideSpeed = req.sideSpeed;
    m_reqHighROS.rotateSpeed = req.rotateSpeed;
    m_reqHighROS.mode = req.mode;
    // std::cout << "REQUEST\nmode:" << m_reqHighROS.mode
    //          << "\nfwdspd: " << m_reqHighROS.forwardSpeed
    //          << "\nsidespd: " << m_reqHighROS.sideSpeed
    //         << "\nrotspd: " << m_reqHighROS.rotateSpeed << std::endl;
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
      //   std::cout << "\n\nmode:" << m_reqHighROS.mode
      //             << "\nfwdspd: " << m_reqHighROS.forwardSpeed
      //             << "\nsidespd: " << m_reqHighROS.sideSpeed
      //             << "\nrotspd: " << m_reqHighROS.rotateSpeed << std::endl;
      SendHighLCM = ToLcm(m_reqHighROS, SendHighLCM);
      roslcm.Send(SendHighLCM);
      loop_rate.sleep();
    }
  }

  ros::AsyncSpinner m_spinner;
  ros::ServiceServer m_service;
  ros::NodeHandle m_node;
  unitree_legged_msgs::HighCmd m_reqHighROS;
  unitree_legged_msgs::HighState m_recvHighROS;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ctrl_ros_mode");
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
