/* These files are based on https://github.com/ilpincy/argos3-examples (MIT LICENSE) */

#ifndef DIFFUSION_H
#define DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the proximity sensor */
#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_proximity_sensor.h>

#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CDiffusion : public CCI_Controller {
 public:
  /* Class constructor. */
  CDiffusion();

  /* Class destructor. */
  virtual ~CDiffusion() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><diffusion_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset() {}

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

 private:
  /* Pointer to the differential steering actuator */
  CCI_DifferentialSteeringActuator* m_pcWheels;
  /* Pointer to the proximity sensor */
  CCI_KheperaIVProximitySensor* m_pcProximity;

  CCI_LEDsActuator* m_pcLEDs;

  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><diffusion_controller> section.
   */

  /* Maximum tolerance for the angle between
   * the robot heading direction and
   * the closest obstacle detected. */
  CDegrees m_cAlpha;
  /* Maximum tolerance for the proximity reading between
   * the robot and the closest obstacle.
   * The proximity reading is 0 when nothing is detected
   * and grows exponentially to 1 when the obstacle is
   * touching the robot.
   */
  Real m_fDelta;
  /* Wheel speed. */
  Real m_fWheelVelocity;
  /* Angle tolerance range to go straight.
   * It is set to [-alpha,alpha]. */
  CRange<CRadians> m_cGoStraightAngleRange;
};

#endif
