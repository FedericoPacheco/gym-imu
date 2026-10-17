#include "Constants.hpp"
#include <Auxiliary/MathUtils.hpp>
#include <Auxiliary/Quaternion.hpp>
#include <IMUOrientationFinder.hpp>
#include <StationaryDetection/IMUWindowedStationaryDetector.hpp>
#include <array>
#include <numbers>

class IMUMahonyOrientationFinder : public IMUOrientationFinder<Quaternion> {
public:
  IMUMahonyOrientationFinder(float dt) : IMUOrientationFinder<Quaternion>(dt) {
    this->reset();
  }

  void initialize(IMUSample &sample) override {
    this->transformFromSensorToBodyFrame(sample);
    initFBSum[0] += sample.a.x;
    initFBSum[1] += sample.a.y;
    initFBSum[2] += sample.a.z;
    initWBRadPerSSum[0] += sample.w.roll * std::numbers::pi_v<float> / 180.0f;
    initWBRadPerSSum[1] += sample.w.pitch * std::numbers::pi_v<float> / 180.0f;
    initWBRadPerSSum[2] += sample.w.yaw * std::numbers::pi_v<float> / 180.0f;
    sampleCount++;
  }

  Quaternion find(IMUSample &sample) override {
    this->transformFromSensorToBodyFrame(sample);
    if (q == Quaternion::ZERO) { // Set initial conditions
      sample.a.x = initFBSum[0] / sampleCount;
      sample.a.y = initFBSum[1] / sampleCount;
      sample.a.z = initFBSum[2] / sampleCount;

      std::array<float, 3> initWBRadPerS = {
          initWBRadPerSSum[0] / sampleCount - B0_RAD_PER_S[0],
          initWBRadPerSSum[1] / sampleCount - B0_RAD_PER_S[1],
          initWBRadPerSSum[2] / sampleCount - B0_RAD_PER_S[2]};
      sample.w.roll = initWBRadPerS[0] * 180.0f / std::numbers::pi_v<float>;
      sample.w.pitch = initWBRadPerS[1] * 180.0f / std::numbers::pi_v<float>;
      sample.w.yaw = initWBRadPerS[2] * 180.0f / std::numbers::pi_v<float>;

      q = Quaternion::fromDCM(this->buildReferenceFrame(
          std::array<float, 3>{sample.a.x, sample.a.y, sample.a.z}));
      prevQ = q;
      prevQRate = 0.5f * q * Quaternion(0.0f, initWBRadPerS);
    } else { // Run the filter normally
      auto gBUnit = prevQ.conjugate().rotate(G_W_UNIT);

      auto fBUnit = normalize(
          std::array<float, 3>{-sample.a.x, -sample.a.y, -sample.a.z});
      auto eB = cross(fBUnit, gBUnit);
      if (!stationaryDetector.isStationaryPreOrientation(sample)) {
        eB[0] *= KM;
        eB[1] *= KM;
        eB[2] *= KM;
      }

      bRate = {-KI * eB[0], -KI * eB[1], -KI * eB[2]};
      bBRadPerS[0] = bBRadPerS[0] + 0.5f * this->dt * (prevBRate[0] + bRate[0]);
      bBRadPerS[1] = bBRadPerS[1] + 0.5f * this->dt * (prevBRate[1] + bRate[1]);
      bBRadPerS[2] = bBRadPerS[2] + 0.5f * this->dt * (prevBRate[2] + bRate[2]);
      prevBRate = bRate;

      std::array<float, 3> processedWBRadPerS = {
          sample.w.roll * std::numbers::pi_v<float> / 180.0f - bBRadPerS[0],
          sample.w.pitch * std::numbers::pi_v<float> / 180.0f - bBRadPerS[1],
          sample.w.yaw * std::numbers::pi_v<float> / 180.0f - bBRadPerS[2]};
      std::array<float, 3> correctedWBRadPerS = {
          processedWBRadPerS[0] + KP * eB[0],
          processedWBRadPerS[1] + KP * eB[1],
          processedWBRadPerS[2] + KP * eB[2]};

      qRate = 0.5f * prevQ * Quaternion(0.0f, correctedWBRadPerS);
      q = (prevQ + 0.5f * this->dt * (prevQRate + qRate)).normalized();
      prevQ = q;
      prevQRate = qRate;

      sample.w.roll =
          processedWBRadPerS[0] * 180.0f / std::numbers::pi_v<float>;
      sample.w.pitch =
          processedWBRadPerS[1] * 180.0f / std::numbers::pi_v<float>;
      sample.w.yaw = processedWBRadPerS[2] * 180.0f / std::numbers::pi_v<float>;
    }
    return q;
  }

  void reset() override {
    this->initFBSum = {0.0f, 0.0f, 0.0f};
    this->initWBRadPerSSum = {0.0f, 0.0f, 0.0f};
    this->sampleCount = 0;
    this->q = Quaternion::ZERO;
    this->qRate = Quaternion::ZERO;
    this->prevQ = Quaternion::ZERO;
    this->prevQRate = Quaternion::ZERO;
    this->bBRadPerS = B0_RAD_PER_S;
    this->bRate = std::array<float, 3>{0.0f, 0.0f, 0.0f};
    this->prevBRate = std::array<float, 3>{0.0f, 0.0f, 0.0f};
    this->stationaryDetector.reset();
  }

private:
  static constexpr std::array<float, 3> B0_RAD_PER_S = {
      -0.09038445f, -0.02537323f, -0.05504172f};

  static constexpr float KM = 0.25f;
  static constexpr float KP = 5.0f;
  static constexpr float KI = 0.05f;

  static constexpr std::array<float, 3> Y_B = {0.0f, 1.0f, 0.0f};
  static constexpr std::array<float, 3> Z_B = {0.0f, 0.0f, 1.0f};
  static constexpr float HORIZONTAL_TOL = 0.01f;

  std::array<float, 3> initFBSum;
  std::array<float, 3> initWBRadPerSSum;
  uint32_t sampleCount;
  Quaternion q;
  Quaternion qRate;
  Quaternion prevQ;
  Quaternion prevQRate;
  std::array<float, 3> bBRadPerS;
  std::array<float, 3> bRate;
  std::array<float, 3> prevBRate;
  IMUWindowedStationaryDetector stationaryDetector;

  void transformFromSensorToBodyFrame(IMUSample &sample) {
    IMUSample temp = sample;
    sample.a.y = temp.a.z;
    sample.a.z = -temp.a.y;
    sample.w.pitch = temp.w.yaw;
    sample.w.yaw = -temp.w.pitch;
  }

  std::array<std::array<float, 3>, 3>
  buildReferenceFrame(std::array<float, 3> fBMean) {
    auto zWBUnit = normalize(fBMean);

    auto yBzWBUnitDot = dot(Y_B, zWBUnit);
    std::array<float, 3> yWB;
    yWB[0] = Y_B[0] - yBzWBUnitDot * zWBUnit[0];
    yWB[1] = Y_B[1] - yBzWBUnitDot * zWBUnit[1];
    yWB[2] = Y_B[2] - yBzWBUnitDot * zWBUnit[2];
    if (norm(yWB) < HORIZONTAL_TOL) {
      auto zBzWBUnitDot = dot(Z_B, zWBUnit);
      yWB[0] = Z_B[0] - zBzWBUnitDot * zWBUnit[0];
      yWB[1] = Z_B[1] - zBzWBUnitDot * zWBUnit[1];
      yWB[2] = Z_B[2] - zBzWBUnitDot * zWBUnit[2];
    }
    auto yWBUnit = normalize(yWB);

    auto xWBUnit = normalize(cross(yWBUnit, zWBUnit));

    return {{{xWBUnit[0], xWBUnit[1], xWBUnit[2]},
             {yWBUnit[0], yWBUnit[1], yWBUnit[2]},
             {zWBUnit[0], zWBUnit[1], zWBUnit[2]}}};
  }
};
