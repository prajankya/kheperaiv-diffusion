/* These files are based on https://github.com/ilpincy/argos3-examples (MIT LICENSE) */

/* Include the controller definition */
#include "diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CDiffusion::CDiffusion()
    : m_pcWheels(NULL),
      m_pcProximity(NULL),
      m_pcLEDs(NULL),
      m_cAlpha(10.0f),
      m_fDelta(0.5f),
      m_fWheelVelocity(2.5f),
      m_cGoStraightAngleRange(-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CDiffusion::Init(TConfigurationNode& t_node) {
  m_pcWheels =
    GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity =
    GetSensor<CCI_KheperaIVProximitySensor>("kheperaiv_proximity");

  m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
  m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
  GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
  GetNodeAttributeOrDefault(
    t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CDiffusion::ControlStep() {
  /* Get readings from proximity sensor */
  const CCI_KheperaIVProximitySensor::TReadings& tProxReads =
    m_pcProximity->GetReadings();

  /* Sum them together */
  CVector2 cAccumulator;
  for (size_t i = 0; i < tProxReads.size(); ++i) {
    cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
  }
  cAccumulator /= tProxReads.size();
  /* If the angle of the vector is small enough and the closest obstacle
   * is far enough, continue going straight, otherwise curve a little
   */
  CRadians cAngle = cAccumulator.Angle();
  if (
    m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
    cAccumulator.Length() < m_fDelta) {
    /* Go straight */
    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    m_pcLEDs->SetSingleColor(0, CColor(255, 0, 0));
    m_pcLEDs->SetSingleColor(2, CColor(255, 0, 0));
    m_pcLEDs->SetSingleColor(1, CColor(0, 0, 255));
  } else {
    /* Turn, depending on the sign of the angle */
    if (cAngle.GetValue() > 0.0f) {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      m_pcLEDs->SetSingleColor(0, CColor(0, 0, 0));
      m_pcLEDs->SetSingleColor(2, CColor(0, 255, 0));
      RLOG << "Turning right" << std::endl;
    } else {
      m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      m_pcLEDs->SetSingleColor(0, CColor(0, 255, 0));
      m_pcLEDs->SetSingleColor(2, CColor(0, 0, 0));
      RLOG << "Turning left" << std::endl;
    }
  }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CDiffusion, "diffusion_controller")
